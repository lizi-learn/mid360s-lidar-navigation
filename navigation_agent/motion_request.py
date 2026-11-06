"""Finite, explicit relative-motion phrases. No timing or velocity inference."""
import math
import re
import unicodedata

NUM=r'(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百千点半]+)'
def number(text):
    if text=='半':return .5
    if re.fullmatch(r'\d+(?:\.\d+)?',text):return float(text)
    digits={c:n for n,chars in enumerate(('零〇','一','二两','三','四','五','六','七','八','九'))for c in chars}
    if '点'in text:
        a,b=text.split('点',1)
        if not b or any(c not in digits for c in b):raise ValueError('小数格式无效')
        return number(a)+float('0.'+''.join(str(digits[c])for c in b))
    if not any(c in text for c in '十百千'):
        if any(c not in digits for c in text):raise ValueError('数字格式无效')
        return float(''.join(str(digits[c])for c in text))
    total=current=0;previous=10000
    for c in text:
        if c in digits:current=digits[c]
        elif c in '十百千':
            unit={'十':10,'百':100,'千':1000}[c]
            if unit>=previous:raise ValueError('数字单位顺序无效')
            total+=(current or 1)*unit;current=0;previous=unit
        else:raise ValueError('数字格式无效')
    return float(total+current)

def motion_request(clause):
    text=re.sub(r'\s','',unicodedata.normalize('NFKC',clause)).rstrip('。！!')
    forward=re.fullmatch(r'(?:向前|往前)(?:走|移动|前进)?('+NUM+r')(厘米|公分|米|cm|m)|前进('+NUM+r')(厘米|公分|米|cm|m)',text,re.I)
    if forward:
        n,unit=forward.groups()[:2]if forward.group(1)else forward.groups()[2:]
        try:meters=number(n)/(100 if unit.lower()in('厘米','公分','cm')else 1)
        except ValueError:return None,'前进距离数字无法确定，请给出如“前进1.5米”。'
        if not 0<meters<=100000:return None,'前进距离应为正的有限米数。'
        return dict(kind='forward',meters=meters),None
    spin=re.fullmatch(r'(?:原地)?(?:(向左|往左|左|逆时针|向右|往右|右|顺时针))?(?:旋转|转动|转)(?:动)?('+NUM+r')(度|圈|周)',text)
    if spin:
        direction,n,unit=spin.groups()
        try:angle=number(n)*(math.pi/180 if unit=='度'else math.tau)
        except ValueError:return None,'转角数字无法确定，请给出如“向左转90度”。'
        if direction in('向右','往右','右','顺时针'):angle=-angle
        if not 0<abs(angle)<=100000:return None,'转角应为有限非零角度。'
        return dict(kind='rotate',radians=angle),None
    if re.match(r'^(?:(?:原地)?(?:向左|往左|左|逆时针|向右|往右|右|顺时针)?(?:旋转|转动|转)|前进|往前|向前)',text):
        return None,'请明确距离或转角，例如“前进2米”“右转90度”“左转两圈”；未指定方向的转角默认左转。'
    return None,None
