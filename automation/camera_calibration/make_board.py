from pathlib import Path
import json
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
out=Path(__file__).parent; (out/'output/pdf').mkdir(parents=True,exist_ok=True)
all_bits=json.loads((out/'marker_bits.json').read_text())
p=out/'output/pdf/SO_ARM4_ArUco_100percent.pdf';c=canvas.Canvas(str(p),pagesize=A4)
c.setFont('Helvetica-Bold',16);c.drawString(25*mm,270*mm,'SO-ARM4 camera calibration board')
c.setFont('Helvetica',11)
for i,t in enumerate(['Print at ACTUAL SIZE / 100%. Disable fit-to-page.','DICT_4X4_50 | IDs 0,1,2,3 | black marker edge: 45 mm','Marker gap: 10 mm | board area: 100 x 100 mm','Mount flat on a rigid, matte board. Do not cover markers.']):c.drawString(25*mm,(259-i*7)*mm,t)
# Image object frame has x right, y down; outer marker area origin 0,0.
left,top=55,215
for ident in range(4):
 bits=all_bits[ident]
 x0=(ident%2)*55;y0=(ident//2)*55
 for y in range(6):
  for x in range(6):
   if bits[y][x]<128:c.rect((left+x0+x*7.5)*mm,(top-y0-(y+1)*7.5)*mm,7.5*mm,7.5*mm,stroke=0,fill=1)
c.setFont('Helvetica',10);c.drawString(55*mm,103*mm,'Measure a BLACK square edge: exactly 45 mm.')
c.line(55*mm,90*mm,155*mm,90*mm)
for x in range(0,101,10):c.line((55+x)*mm,88*mm,(55+x)*mm,92*mm)
c.drawString(55*mm,81*mm,'This ruler must measure exactly 100 mm.')
c.drawString(25*mm,55*mm,'Keep the WHITE margin around all markers (at least 8 mm).')
c.drawString(25*mm,48*mm,'Camera calibration only. No robot or turntable motion commands.')
c.save()
print(p)
