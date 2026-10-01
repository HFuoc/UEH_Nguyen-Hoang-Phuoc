"""Camera-only local curve fitting on a calibrated ground plane."""
import cv2
import numpy as np

def fit_curve(mask, camera, previous=0.):
    """Fit paired 0.35 m lane boundaries to locally projected image paint.

    No route coordinates; intended only to resolve sharp-curve ambiguities.
    """
    p = camera
    resolution=.004
    forward=np.arange(.13,1.05,resolution)
    lateral=np.arange(-.85,.85,resolution)
    x,y=np.meshgrid(forward,lateral,indexing='ij')
    u=p.cx-y*p.fx/(x-p.offset);v=p.cy+p.height*p.fy/(x-p.offset)
    bird=cv2.remap(mask,u.astype('float32'),v.astype('float32'),cv2.INTER_NEAREST)
    distance=cv2.distanceTransform(255-bird,cv2.DIST_L2,3)*resolution
    curvatures=np.arange(-2.2,2.21,.1)
    offsets=np.arange(-.10,.101,.02)
    k,d=np.meshgrid(curvatures,offsets,indexing='ij');k=k.flatten();d=d.flatten()
    s=np.linspace(.25,.8,26)[None,:]
    theta=k[:,None]*s
    xc=np.where(abs(k[:,None])<1e-6,s,np.sin(theta)/np.where(abs(k[:,None])<1e-6,1,k[:,None]))
    yc=(1-np.cos(theta))/np.where(abs(k[:,None])<1e-6,1,k[:,None])+d[:,None]
    supports=[];coverages=[]
    for sign in (-1,1):
        xe=xc-sign*.175*np.sin(theta);ye=yc+sign*.175*np.cos(theta)
        pu=p.cx-ye*p.fx/(xe-p.offset);pv=p.cy+p.height*p.fy/(xe-p.offset)
        valid=(xe>=forward[0])&(xe<forward[-1])&(ye>=lateral[0])&(ye<lateral[-1])&(pu>2)&(pu<mask.shape[1]-2)&(pv>p.cy+15)&(pv<mask.shape[0]-2)
        rows=np.clip(np.round((xe-forward[0])/resolution).astype(int),0,len(forward)-1)
        cols=np.clip(np.round((ye-lateral[0])/resolution).astype(int),0,len(lateral)-1)
        support=np.exp(-(distance[rows,cols]/.017)**2)*valid
        supports.append(support.sum(axis=1)/np.maximum(valid.sum(axis=1),1))
        coverages.append(valid.sum(axis=1)/26)
    a,b=supports;ca,cb=coverages
    score=.7*np.maximum(a,b)+.3*np.minimum(a,b)-.2*abs(d)-.01*abs(k-previous)
    score[(ca<.35)|(cb<.35)]=-1
    idx=np.argmax(score)
    return float(k[idx]),float(d[idx]),float(score[idx]),(float(a[idx]),float(b[idx]))

