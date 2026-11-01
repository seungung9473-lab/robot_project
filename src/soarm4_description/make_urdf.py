from pathlib import Path
import json,xml.etree.ElementTree as E
P=Path(__file__).resolve().parent

def fmt(a):return ' '.join(f'{v:.12g}' for v in a)

def build():
    m=json.loads((P/'config/model.json').read_text());r=E.Element('robot',name='soarm4_checked_v5_7')
    E.SubElement(r,'link',name='world')
    for name in m['links']:E.SubElement(r,'link',name=name)
    def fixed(name,parent,child,rpy,xyz=(0,0,0)):
        j=E.SubElement(r,'joint',name=name,type='fixed');E.SubElement(j,'parent',link=parent);E.SubElement(j,'child',link=child);E.SubElement(j,'origin',xyz=fmt(xyz),rpy=fmt(rpy))
    fixed('electronics_box_fixed','world','electronics_base_link',[0,0,0]);fixed('robot_mount_fixed','world','base_link',[0,0,m['robot_base_yaw']],m.get('robot_base_xyz',[0,0,0]))
    for v in m['visuals']:
        path=P/'meshes'/v['file'];assert path.is_file(),path
        # One visual per fixed child avoids RViz applying the first material to every mesh on a link.
        visual_link=v['name']+'_visual_link'
        link=E.SubElement(r,'link',name=visual_link)
        fixed(v['name']+'_visual_fixed',v['link'],visual_link,[0,0,0])
        for tag in ['visual','collision']:
            n=E.SubElement(link,tag,name=v['name']);E.SubElement(n,'origin',xyz='0 0 0',rpy='0 0 0');g=E.SubElement(n,'geometry');E.SubElement(g,'mesh',filename='package://soarm4_description/meshes/'+v['file'])
            if tag=='visual':mat=E.SubElement(n,'material',name=v['name']+'_color');E.SubElement(mat,'color',rgba=fmt(v['color']))
    for v in m['joints']:
        j=E.SubElement(r,'joint',name=v['name'],type='revolute');E.SubElement(j,'parent',link=v['parent']);E.SubElement(j,'child',link=v['child']);E.SubElement(j,'origin',xyz=fmt(v['origin_xyz']),rpy='0 0 0');E.SubElement(j,'axis',xyz=fmt(v['axis']));E.SubElement(j,'limit',**v['limit'])
    E.SubElement(r,'link',name='cup_tip');j=E.SubElement(r,'joint',name='cup_tip_fixed',type='fixed');E.SubElement(j,'parent',link='tool_link');E.SubElement(j,'child',link='cup_tip');E.SubElement(j,'origin',xyz=fmt(m['tcp_xyz']),rpy='0 0 0')
    def save(name):
        E.indent(r,space='  ');(P/'urdf'/name).write_text('<?xml version="1.0"?>\n'+E.tostring(r,encoding='unicode')+'\n')
    save('soarm4.urdf')
    # Only the slider's separate input gets display ranges. RobotModel/controller URDF retains prior limits.
    for j in r.findall("joint[@type='revolute']"):
        lim=j.find('limit')
        if j.get('name')=='joint2':
            continue  # J2 GUI must use the canonical limits exactly; never expand.
        lo,hi=m['display_ranges'][j.get('name')];lim.set('lower',str(lo));lim.set('upper',str(hi))
    save('soarm4_visualization.urdf')

if __name__=='__main__':build();print('Generated canonical URDF and display-only slider URDF.')
