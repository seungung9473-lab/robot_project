from pathlib import Path
import struct,json,numpy as np

def read_glb(path):
    d=Path(path).read_bytes();n=struct.unpack_from('<I',d,12)[0];j=json.loads(d[20:20+n]);off=20+n;length,kind=struct.unpack_from('<II',d,off);b=d[off+8:off+8+length]
    def acc(k):
        a=j['accessors'][k];v=j['bufferViews'][a['bufferView']];dtype={5126:'<f4',5125:'<u4',5123:'<u2',5121:'u1'}[a['componentType']];dim={'VEC3':3,'VEC4':4,'SCALAR':1}[a['type']];assert 'byteStride' not in v
        return np.frombuffer(b,dtype=dtype,count=a['count']*dim,offset=v.get('byteOffset',0)+a.get('byteOffset',0)).reshape(-1,dim).copy()
    result={}
    for node in j['nodes']:
        assert not any(k in node for k in ['matrix','translation','rotation','scale','children'])
        q=j['meshes'][node['mesh']]['primitives'][0];vertices=acc(q['attributes']['POSITION']).astype(float);faces=acc(q['indices']).reshape(-1,3)
        c=acc(q['attributes']['COLOR_0']);color=(c.mean(axis=0)/255 if c.dtype==np.uint8 else c.mean(axis=0)).tolist()
        result[node['name']]={'vertices':vertices,'faces':faces,'color':color}
    return result

def fit(x,y):
    X=x-x.mean(0);Y=y-y.mean(0);u,s,vh=np.linalg.svd(X.T@Y);D=np.eye(3);D[-1,-1]=np.linalg.det(vh.T@u.T);R=vh.T@D@u.T;t=y.mean(0)-R@x.mean(0)
    return R,t

def write_stl(path,vertices,faces):
    tri=vertices[faces];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);norm=np.linalg.norm(normal,axis=1);normal/=np.where(norm>0,norm,1)[:,None]
    a=np.zeros(len(tri),dtype=[('n','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]);a['n']=normal;a['v']=tri
    Path(path).write_bytes(b'SOARM4 local mesh in meters'.ljust(80,b' ')+struct.pack('<I',len(a))+a.tobytes())

def read_stl(path):
    d=Path(path).read_bytes();n=struct.unpack_from('<I',d,80)[0];assert len(d)==84+50*n
    return np.ndarray((n,),dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]),buffer=d,offset=84)['v'].astype(float)

def rot(axis,q):
    v=np.array(axis,dtype=float);v/=np.linalg.norm(v);K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+np.sin(q)*K+(1-np.cos(q))*(K@K)
