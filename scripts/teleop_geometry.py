"""Shared geometry without a simulator or robot transport dependency."""
import numpy as np

def rotation_to_xyzw(rotation):
    r=np.asarray(rotation,dtype=float)
    if r.shape!=(3,3) or not np.all(np.isfinite(r)) or not np.allclose(r.T@r,np.eye(3),atol=.02) or abs(np.linalg.det(r)-1)>.02:
        raise ValueError('Invalid rotation matrix')
    q=np.zeros(4)
    trace=np.trace(r)
    if trace>0:
        s=2*np.sqrt(trace+1)
        q[:3]=[r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]]
        q[:3]/=s;q[3]=s/4
    else:
        i=int(np.argmax(np.diag(r)));j=(i+1)%3;k=(i+2)%3
        s=2*np.sqrt(max(0,1+r[i,i]-r[j,j]-r[k,k]))
        if s<1e-12:raise ValueError('Degenerate rotation')
        q[i]=s/4;q[j]=(r[j,i]+r[i,j])/s;q[k]=(r[k,i]+r[i,k])/s
        q[3]=(r[k,j]-r[j,k])/s
    q/=np.linalg.norm(q)
    if q[3]<0:q=-q
    return q
