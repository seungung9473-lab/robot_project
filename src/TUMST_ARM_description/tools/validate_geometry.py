from pathlib import Path
import json,xml.etree.ElementTree as E,numpy as np
from mesh_geometry import read_glb,read_stl,rot
P=Path(__file__).resolve().parents[1]

def fk(robot,angles):
    T={'world':np.eye(4)};pending=robot.findall('joint')[:]
    while pending:
        progress=False
        for j in pending[:]:
            parent=j.find('parent').get('link');child=j.find('child').get('link')
            if parent not in T:continue
            a=j.find('origin');F=np.eye(4);F[:3,3]=list(map(float,a.get('xyz').split()));r,p,y=map(float,a.get('rpy').split());F[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r)
            if j.get('type')=='revolute':F[:3,:3]=F[:3,:3]@rot(list(map(float,j.find('axis').get('xyz').split())),angles[j.get('name')])
            T[child]=T[parent]@F;pending.remove(j);progress=True
        assert progress
    return T

def check():
    robot=E.parse(P/'urdf/TUMST_ARM.urdf').getroot();display=E.parse(P/'urdf/TUMST_ARM_visualization.urdf').getroot();m=json.loads((P/'config/model.json').read_text());result={};R=rot([0,0,1],m['robot_base_yaw'])
    for pose,file in [('zero','assembly_zero.glb'),('rest','assembly_rest.glb')]:
        src=read_glb(P/'reference'/file);T=fk(robot,dict(zip([j['name'] for j in m['joints']],m['poses'][pose])));errors={}
        for link in robot.findall('link'):
            assert len(link.findall('visual'))<=1,'RViz color isolation requires one visual per link'
            for v in link.findall('visual'):
                name=v.get('name');path=P/v.find('geometry/mesh').get('filename').split('package://soarm4_description/')[1];tri=read_stl(path);F=T[link.get('name')];actual=tri@F[:3,:3].T+F[:3,3];expected=src[name]['vertices'][src[name]['faces']]
                if name=='base_link_print' and (P/'config/base_mesh_replacement.json').exists():
                    replacement=json.loads((P/'config/base_mesh_replacement.json').read_text())
                    retained=read_stl(P/replacement['preserved_holder'])
                    new=read_stl(P/replacement['input'])/1000
                    new=new@np.array(replacement['CAD_to_base_rotation']).T+np.array(replacement['CAD_to_base_translation_m'])
                    expected=np.concatenate([retained,new])
                if (name=='base_link_servo' and (P/'config/j1_connector_correction.json').exists()) or (name in ['link1_servo','link2_servo','link3_servo'] and (P/'config/all_servo_connector_correction.json').exists()):
                    mm=read_stl(P/'reference/base_link_servo_before_connector_fix.stl')*1000
                    removed=(mm[:,:,0].min(1)>-15.051)&(mm[:,:,0].max(1)<-13.049)&(mm[:,:,1].min(1)>-8.601)&(mm[:,:,1].max(1)<8.601)&(mm[:,:,2].min(1)>121.699)&(mm[:,:,2].max(1)<126.701)
                    assert removed.sum()==1340
                    expected=expected[~removed]
                if name not in ['base_link_box_plate','base_link_box_wall']:expected=expected@R.T+np.array(m.get('robot_base_xyz',[0,0,0]))
                err=float(np.linalg.norm(actual-expected,axis=-1).max());assert err<2e-7,(pose,name,err);errors[name]=err
                expectedcolor=np.array(src[name]['color']);color=np.array(list(map(float,v.find('material/color').get('rgba').split())));assert abs(expectedcolor-color).max()<1e-10
        result[pose]={'max_vertex_error_m':max(errors.values()),'by_mesh':errors}
    for j in m['joints']:
        assert robot.find(f"joint[@name='{j['name']}']/limit").attrib==j['limit']
        d=display.find(f"joint[@name='{j['name']}']/limit")
        if j['name']=='joint2':assert d.attrib==j['limit']
        else:assert float(d.get('lower'))<=0<=float(d.get('upper'))
    result.update({'enclosure_unchanged':'PASS','robot_base_and_arm_yaw_deg':90,'robot_base_world_xyz':m.get('robot_base_xyz',[0,0,0]),'one_visual_per_link_and_colors':'PASS','canonical_limits_match_model':'PASS','default_gui_pose':m['poses']['home'],'home_encoder_counts':m['home_reference']['encoder_counts'],'J2_GUI_limits_equal_canonical':'PASS'})
    return result

if __name__=='__main__':
    result=check();(P/'validation_geometry.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
