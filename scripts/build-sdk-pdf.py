"""Render the public project/SDK manual. Needs ReportLab and a CJK font."""
from pathlib import Path
import html
import re
import textwrap
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle, Preformatted, KeepTogether, CondPageBreak

ROOT=Path(__file__).resolve().parents[1]
OUTPUT=ROOT/'output/pdf/P2-Navigation-V2-SDK-Manual.pdf';OUTPUT.parent.mkdir(parents=True,exist_ok=True)
font=Path('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc')
if not font.exists():raise SystemExit('Install fonts-wqy-zenhei or set this renderer to an installed CJK TrueType font')
pdfmetrics.registerFont(TTFont('CN',str(font)))
pdfmetrics.registerFont(TTFont('Mono','/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'))
ink=colors.HexColor('#173b34');green=colors.HexColor('#08766c');muted=colors.HexColor('#61736e')
styles={
 'p':ParagraphStyle('p',fontName='CN',fontSize=9.4,leading=16.4,textColor=ink,spaceAfter=9,wordWrap='CJK'),
 'h1':ParagraphStyle('h1',fontName='CN',fontSize=25,leading=35,textColor=ink,spaceAfter=22,wordWrap='CJK'),
 'h2':ParagraphStyle('h2',fontName='CN',fontSize=16,leading=25,textColor=green,spaceBefore=18,spaceAfter=13,keepWithNext=True,wordWrap='CJK'),
 'h3':ParagraphStyle('h3',fontName='CN',fontSize=12,leading=19,textColor=ink,spaceBefore=13,spaceAfter=8,keepWithNext=True,wordWrap='CJK'),
 'small':ParagraphStyle('small',fontName='CN',fontSize=8,leading=13,textColor=muted,spaceAfter=8,wordWrap='CJK'),
 'cell':ParagraphStyle('cell',fontName='CN',fontSize=8,leading=13,textColor=ink,wordWrap='CJK'),
 'code':ParagraphStyle('code',fontName='Mono',fontSize=7.2,leading=11,textColor=ink,backColor=colors.HexColor('#edf3ee'),borderPadding=9,spaceBefore=4,spaceAfter=13)}
def inline(s):
 s=html.escape(s)
 s=re.sub(r'\[([^\]]+)\]\((https?://[^)]+)\)',lambda m:'<link href="'+m[2]+'" color="#08766c">'+m[1]+'</link>',s)
 s=re.sub(r'\[([^\]]+)\]\([^)]+\)',r'\1',s)
 s=re.sub(r'`([^`]+)`',r'<font color="#08766c">\1</font>',s)
 return s.replace('**','')
def para(s,style='p'):return Paragraph(inline(s),styles[style])
flow=[]
flow+=[Spacer(1,70),para('P2 NAVIGATION / ENGINEERING MANUAL','small'),Spacer(1,14),para('导航项目与统一 SDK<br/>完整手册'.replace('<br/>','\n'),'h1'),para('V2 Release Baseline · NavBridge 0.1.0','h2'),Spacer(1,20),para('建图 · 点位 · 规定路线 · 产品讲解<br/>P2 / WATER 接入 · 运维与换机'.replace('<br/>',' · ')),Spacer(1,42),para('2026-11-05 · 公开技术文档','small'),para('当前 SDK 为已测试的导航兼容子集，不是云迹全系列/全固件认证。本文不包含私有办公室地图、机器人厂家资产或凭据。','small'),PageBreak()]
flow+=[para('阅读导航','h1'),para('01 项目与部署','h2'),para('目标与架构、V2能力、依赖安装、网络与机器身份、建图定位。'),para('02 现场与内容管理','h2'),para('点位/产品/话术、规定路线、G12控制权、避障等待、地图标注与低频学习、场地迁移。'),para('03 开发与云平台','h2'),para('统一SDK、任务状态、幂等与异常、WATER兼容矩阵、云端签名边界、软件验证。'),para('在线参考','h2'),para('https://navigation.bismih520.com/'),para('源码与固定导航基线：github.com/lizi-learn/mid360s-lidar-navigation','small'),PageBreak()]

def document(path,appendix=False):
 lines=path.read_text().splitlines();i=0;buffer=[]
 def flush():
  if buffer:flow.append(para(' '.join(buffer)));buffer.clear()
 while i<len(lines):
  line=lines[i]
  if not line.strip():flush();i+=1;continue
  if line.startswith('```'):
   flush();code=[];i+=1
   while i<len(lines)and not lines[i].startswith('```'):code.append(lines[i]);i+=1
   # Preserve Chinese comments as CJK paragraphs, ASCII examples as monospace.
   wrapped=[]
   for c in code:
    if any(ord(x)>127 for x in c):
     if wrapped:flow.append(Preformatted('\n'.join(wrapped),styles['code']));wrapped=[]
     flow.append(para(c,'small'))
    else:wrapped.extend(textwrap.wrap(c,width=99,replace_whitespace=False,drop_whitespace=False)or[''])
   if wrapped:flow.append(Preformatted('\n'.join(wrapped),styles['code']))
   i+=1;continue
  if line.startswith('# '):
   flush();flow.append(para(line[2:],'h1'));i+=1;continue
  if line.startswith('## '):
   flush()
   if not appendix and re.match(r'## \d+\.',line):flow.append(CondPageBreak(230))
   if appendix and line.startswith('## 一手资料'):flow.append(PageBreak())
   flow.append(para(line[3:],'h2'));i+=1;continue
  if line.startswith('### '):flush();flow.append(para(line[4:],'h3'));i+=1;continue
  if line.startswith('|'):
   flush();rows=[]
   while i<len(lines)and lines[i].startswith('|'):
    cells=[c.strip()for c in lines[i].strip('|').split('|')]
    if not all(re.fullmatch(r':?-+:?',c.replace(' ',''))for c in cells):rows.append([para(c,'cell')for c in cells])
    i+=1
   width=A4[0]-94;cols=len(rows[0]);widths=([width*.27,width*.73]if cols==2 else [width*.31,width*.18,width*.51]if cols==3 else [width/cols]*cols)
   table=Table(rows,colWidths=widths,repeatRows=1,hAlign='LEFT')
   table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dcebe1')),('VALIGN',(0,0),(-1,-1),'TOP'),('BOTTOMPADDING',(0,0),(-1,-1),8),('TOPPADDING',(0,0),(-1,-1),8),('LINEBELOW',(0,0),(-1,-1),.5,colors.HexColor('#d8e1db'))]))
   flow.extend([table,Spacer(1,13)]);continue
  if line.startswith('- '):flush();flow.append(para('• '+line[2:],'small'));i+=1;continue
  buffer.append(line);i+=1
 flush()

document(ROOT/'docs/sdk/PROJECT_GUIDE.md')
flow.append(PageBreak());document(ROOT/'docs/sdk/README.md',appendix=True)
flow.append(PageBreak());document(ROOT/'docs/sdk/COMPATIBILITY.md',appendix=True)
def page(canvas,doc):
 canvas.saveState();canvas.setStrokeColor(colors.HexColor('#d6e1da'));canvas.line(47,40,A4[0]-47,40)
 canvas.setFont('CN',7.5);canvas.setFillColor(muted);canvas.drawString(47,27,'P2 Navigation V2 / NavBridge SDK 0.1.0')
 canvas.drawRightString(A4[0]-47,27,str(doc.page));canvas.restoreState()
doc=SimpleDocTemplate(str(OUTPUT),pagesize=A4,rightMargin=47,leftMargin=47,topMargin=45,bottomMargin=57,title='P2 导航项目与统一 SDK 完整手册',author='P2 Navigation Project')
doc.build(flow,onFirstPage=page,onLaterPages=page)
print(str(OUTPUT))
