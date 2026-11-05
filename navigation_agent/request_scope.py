"""Bind side effects to the current user request and live saved destinations.

This is request interpretation, not a new robot readiness/acceptance gate.
"""
import re
import unicodedata


def fold(value):return re.sub(r'[\s·•，,。！!？?"“”\'‘’]','',unicodedata.normalize('NFKC',value)).casefold()
def clauses(text):return [x.strip()for x in re.split(r'[，,。；;\n]',unicodedata.normalize('NFKC',text))if x.strip()]
def imperative(clause):
    return re.sub(r'^(?:(?:请|帮我|帮忙|麻烦你?|现在|立即|立刻|马上|先|然后|并|再|接着|你|我想|我要|能不能|可以)\s*)+','',clause.strip())
def control_question(clause):
    return bool(re.search(r'(?:了[吗么没](?:有)?|过[吗么])[？?。\s]*$',clause)or
                re.match(r'^(?:为什么|为何|是否|(?:取消|停止|暂停|恢复|继续)(?:的)?原因)',clause))

def requested_controls(text):
    allowed=set()
    for clause in clauses(text):
        clause=imperative(clause)
        if control_question(clause):continue
        if re.match(r'^(?:取消|停止|停下|停一?下|停车|结束)',clause):allowed.add('cancel')
        if clause.startswith('暂停'):allowed.add('pause')
        if clause.startswith(('恢复','继续')):allowed.update(('resume','continue'))
        if re.match(r'^(?:please\s+)?(?:stop|cancel)\b',clause,re.I):allowed.add('cancel')
        if re.match(r'^(?:please\s+)?pause\b',clause,re.I):allowed.add('pause')
        if re.match(r'^(?:please\s+)?(?:resume|continue)\b',clause,re.I):allowed.update(('resume','continue'))
    return allowed

def navigation_clause(clause):
    clause=imperative(clause)
    return bool(re.match(r'^(?:去|前往|带我去|带客人去|带我到|导航到|走到|到(?!达)|按.+(?:参观|巡游|走|路线)|(?:开始|启动|执行).*(?:导航|路线|参观|巡游)|参观|巡游|go\b|navigate\b|run\b|start\b)',clause,re.I))

def only_stop(text):
    return requested_controls(text)=={'cancel'}and not any(navigation_clause(c)for c in clauses(text))

def point_mentions(text,points):
    compact=fold(text);matches=[]
    for p in points:
        name=fold(p['name']);aliases=[name]
        m=re.fullmatch(r'(?:点位|点)(\d+)',name)
        if m:aliases.extend((m[1]+'号点',m[1]+'号点位','点'+m[1]))
        positions=[]
        for alias in aliases:
            pattern=(r'(?<!\d)'if alias[:1].isdigit()else '')+re.escape(alias)+(r'(?!\d)'if alias[-1:].isdigit()else '')
            match=re.search(pattern,compact)
            if match:positions.append(match.start())
        if positions:matches.append((min(positions),p))
    return [p for _,p in sorted(matches,key=lambda item:item[0])]

def request_scope(text,points,routes,history=()):
    parts=clauses(text);controls=requested_controls(text)
    only_read=bool(re.search(r'只查询|只查|只看|仅查询|不要启动|不启动|不要移动|不要行走|不要走路|不启动',text))
    positive=any(navigation_clause(p)for p in parts)or(bool(re.match(r'^(?:启动|开始|执行)',imperative(parts[0]if parts else '')))and any(fold(r['name'])in fold(text)for r in routes))
    query=bool(re.match(r'^(?:为什么|为何|查询|看看|告诉我|现在有|当前有|刚才|之前|是否|有没有)',text.strip()))or any(control_question(p)for p in parts)
    read_only=only_read or control_question(parts[0]if parts else '')or(query and not positive)
    if read_only:return dict(read_only=True,controls=[],navigation=None,error=None,explicit_navigation=False)
    if text.strip()in('去','走吧','出发','开始','开始吧'):
        for item in reversed(history):
            previous=request_scope(item['user'],points,routes)
            if previous.get('navigation'):
                previous['controls']=[];return previous
        positive=True
    out=dict(read_only=False,controls=sorted(controls),navigation=None,error=None,explicit_navigation=positive)
    if not positive:return out
    nav_parts=[p for p in parts if navigation_clause(p)]
    if len(nav_parts)>1:
        mentioned=[point_mentions(p,points)for p in nav_parts]
        wanted=[items[0]['id']for items in mentioned if len(items)==1]
        candidates=[r for r in routes if len(wanted)==len(nav_parts)and [s['point_id']for s in r['steps']]==wanted]
        if len(candidates)==1:out['navigation']=dict(kind='route',route_id=candidates[0]['id'])
        else:out['error']='多个目标没有唯一匹配的已保存路线，请指定整条路线；未拆成部分任务执行。'
        return out
    compact=fold(text)
    # Resolve saved route names/IDs before their constituent point names.
    named=[r for r in routes if fold(r['name'])in compact or fold(r['id'])in compact]
    span=re.search(r'(?:点位|点)?(\d+)(?:号点位|号点|号)?(?:到|至|→|~|-)(?:点位|点)?(\d+)',compact)
    numeric_points={}
    for p in points:
        m=re.fullmatch(r'(?:点位|点)(\d+)|(\d+)号(?:点位|点)?',fold(p['name']))
        if m:numeric_points[int(next(x for x in m.groups()if x is not None))]=p['id']
    if span:
        first,last=map(int,span.groups())
        numbers=range(first,last+(1 if last>=first else -1),1 if last>=first else -1)
        if abs(last-first)>99:out['error']='本次点位范围超出已保存路线，请指定目录中的路线。';return out
        wanted=[numeric_points.get(n)for n in numbers]
        matching=[r for r in routes if None not in wanted and [s['point_id']for s in r['steps']]==wanted]
        named=[r for r in named if r in matching]or matching
        if len(named)!=1:out['error']='没有唯一匹配此点位顺序的已保存路线，请指定路线名称。';return out
    if len(named)==1:out['navigation']=dict(kind='route',route_id=named[0]['id']);return out
    if len(named)>1:out['error']='匹配了多条路线，请指定一个完整路线名称。';return out
    matches=point_mentions(text,points)
    if len(matches)==1:out['navigation']=dict(kind='visit',point_id=matches[0]['id']);return out
    if len(matches)>1:
        wanted=[p['id']for p in matches];matching=[r for r in routes if[s['point_id']for s in r['steps']]==wanted]
        if len(matching)==1:out['navigation']=dict(kind='route',route_id=matching[0]['id'])
        else:out['error']='这条指令包含多个点位，请指定对应的已保存路线或一个目标点。'
        return out
    if re.search(r'路线|参观|巡游',text)and len(routes)==1:out['navigation']=dict(kind='route',route_id=routes[0]['id']);return out
    out['error']='未找到唯一对应的已保存点位或路线，请提供目录中的名称。'
    return out
