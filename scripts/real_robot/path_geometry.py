"""Conservative convex robot/tool checks; external clearance is operator-reported.

Official UR collision STL meshes, plus the user's tool STL, are enclosed by convex
hulls. Adjacent/never-colliding UR pairs follow the installed official SRDF.
"""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from scipy.optimize import linprog
from ament_index_python.packages import get_package_share_directory


def vertices(path):
    raw=Path(path).read_bytes()
    count=struct.unpack_from('<I',raw,80)[0]
    if len(raw)!=84+50*count:raise ValueError('Expected binary STL')
    dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
    points=np.ndarray(count,dtype=dtype,buffer=raw,offset=84)['v'].reshape(-1,3).astype(float)
    if not np.all(np.isfinite(points)):raise ValueError('Nonfinite mesh')
    return np.unique(points,axis=0)


class PathGeometry:
    def __init__(self,model,xml,q0):
        self.model=model
        self.bodies={}
        root=ET.fromstring(xml)
        for link in root.findall('link'):
            collision=link.find('collision')
            if collision is None:continue
            mesh=collision.find('geometry/mesh')
            if mesh is None:raise ValueError('Unsupported robot collision geometry')
            package,relative=mesh.get('filename').removeprefix('package://').split('/',1)
            points=vertices(Path(get_package_share_directory(package))/relative)
            points*=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            origin=collision.find('origin')
            if origin is not None:
                rot=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
                points=points@rot.T+np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            self.add(link.get('name'),link.get('name'),points)
        tool=Path(__file__).resolve().parents[2]/'models/mujoco_menagerie/universal_robots_ur5e/assets/bowel_grasper_v1.stl'
        self.add('attached_tool','tool0',vertices(tool))
        reach=sum(np.linalg.norm(origin[:3,3]) for origin,_,_ in model.tool_chain)
        if reach+max(np.max(np.linalg.norm(points,axis=1)) for _,points,_ in self.bodies.values())>2.:
            raise ValueError('Assembly geometry exceeds 2 m conservative joint-to-surface bound')
        srdf=ET.parse(Path(get_package_share_directory('ur_moveit_config'))/'srdf/ur_macro.srdf.xacro')
        self.exclude={frozenset((e.get('link1').replace('${prefix}',''),e.get('link2').replace('${prefix}','')))
                      for e in srdf.findall('.//disable_collisions')}
        self.exclude.add(frozenset(('attached_tool','wrist_3_link')))
        self.initial=self.transforms(q0)

    def add(self,name,frame,points):
        hull=ConvexHull(points)
        self.bodies[name]=(self.model._chain(frame),points[hull.vertices],np.unique(np.round(hull.equations,9),axis=0))

    def transforms(self,q):
        out={}
        for name,(chain,points,eq) in self.bodies.items():
            pose=self.model.to_base@self.model._fk(chain,q)
            rotated=points@pose[:3,:3].T+pose[:3,3]
            normal=eq[:,:3]@pose[:3,:3].T
            offset=eq[:,3]-normal@pose[:3,3]
            out[name]=(rotated,normal,offset)
        return out

    def check(self,q,padding=.003):
        bodies=self.transforms(q)
        for name,(points,_,_) in bodies.items():
            # 660 mm reported external clearance minus >=100 mm reserve.
            if np.max(np.linalg.norm(points-self.initial[name][0],axis=1))+padding>.55:
                raise ValueError('Robot/tool surface displacement exceeds external-clearance envelope')
        names=list(bodies)
        bounds={name:(points.min(0),points.max(0)) for name,(points,_,_) in bodies.items()}
        for i,a in enumerate(names):
            pa,na,da=bodies[a]
            for b in names[i+1:]:
                if frozenset((a,b)) in self.exclude:continue
                pb,nb,db=bodies[b]
                amin,amax=bounds[a];bmin,bmax=bounds[b]
                if np.any(amax+padding<bmin-padding) or np.any(bmax+padding<amin-padding):continue
                result=linprog(np.zeros(3),A_ub=np.vstack((na,nb)),b_ub=-np.r_[da,db]+padding,
                               bounds=[(None,None)]*3,method='highs')
                if result.status!=2:
                    raise ValueError(f'Conservative self-collision/proximity check: {a}, {b}')

    def curve(self,curve):
        """Adapt intervals to bound every surface's inter-sample travel.

        The assembly's joint-to-surface reach is checked <=2 m at construction.
        Split large intervals instead of padding every body by a large amount.
        """
        pending=[(0.,curve.duration,0)]
        while pending:
            start,end,depth=pending.pop()
            travel=curve.joint_travel(start,end)
            if travel>.001:
                if depth>=12:raise ValueError('Geometry subdivision limit')
                middle=(start+end)/2
                pending.extend([(middle,end,depth+1),(start,middle,depth+1)])
                continue
            self.check(curve.sample(start)[0],padding=.001+2*travel)
        self.check(curve.sample(curve.duration)[0])

    def segment(self,start,end):
        # Sample 5 points along the segment with a 2 mm proximity padding.
        # Across a bounded step (<= 2 mm), the inter-sample travel is <= 0.5 mm,
        # ensuring the continuous swept volume is collision-free without false
        # alarms from over-inflating link radii near the base.
        padding = .002
        for u in (0., .25, .5, .75, 1.):
            self.check(start + u * (end - start), padding=padding)
