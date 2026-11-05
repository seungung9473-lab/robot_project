"""Convex observed-object collider, in a specified local frame."""
import numpy as np
from scipy.spatial import ConvexHull

def padded_hull_points(points,padding_m=0.):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():raise ValueError('Finite Nx3 geometry required')
    if not np.isfinite(padding_m) or not 0<=padding_m<=.01:raise ValueError('Padding must be0..10mm; geometry cannot be shrunk')
    hull=ConvexHull(points);vertices=points[hull.vertices]
    if padding_m:
        # Minkowski sum with a small cube: retain every measured surface point.
        offsets=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])*padding_m
        vertices=(vertices[:,None,:]+offsets).reshape(-1,3)
    return vertices

def convex_mesh(points,origin,padding_m=0.):
    from shape_msgs.msg import Mesh,MeshTriangle
    from geometry_msgs.msg import Point
    points=padded_hull_points(points,padding_m)-np.asarray(origin,float);hull=ConvexHull(points)
    if hull.volume<1e-9:raise ValueError('Insufficient 3D object geometry')
    ids=np.unique(hull.simplices);mapping={int(v):i for i,v in enumerate(ids)};mesh=Mesh()
    mesh.vertices=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in points[ids]]
    for face in hull.simplices:
        a,b,c=points[face];order=face if np.dot(np.cross(b-a,c-a),(a+b+c)/3-points[hull.vertices].mean(0))>0 else face[::-1]
        mesh.triangles.append(MeshTriangle(vertex_indices=[mapping[int(v)] for v in order]))
    return mesh
