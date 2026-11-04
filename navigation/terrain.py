"""Conservative local floor model for levelled LiDAR clouds (no ground truth claim)."""
import math
import numpy as np

def local_floor(points, center, radius=4., expected_height=0.):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3:raise ValueError('XYZ array required')
    distance=np.linalg.norm(points[:,:2]-np.asarray(center),axis=1)
    near=points[np.isfinite(points).all(axis=1)&(distance>.45)&(distance<radius)]
    if len(near)<80:raise ValueError('insufficient nearby ground observations')
    # Low points in XY cells are candidates; high walls and furniture do not vote.
    cells=np.floor((near[:,:2]-np.asarray(center))/.35).astype(int)
    order=np.argsort(near[:,2])
    # Select exactly the same first/lowest point and preserve the old insertion
    # order. Avoid a Python tuple/dictionary operation for every nearby return;
    # the unchanged order also preserves the seeded RANSAC samples below.
    _,first=np.unique(cells[order],axis=0,return_index=True)
    candidates=near[order[np.sort(first)]]
    # Restrict to the expected ground band in the already gravity-levelled frame.
    candidates=candidates[(candidates[:,2]>expected_height-.45)&(candidates[:,2]<expected_height+.45)]
    if len(candidates)<25:raise ValueError('nearby ground is not sufficiently observed')
    a=np.column_stack((candidates[:,:2]-np.asarray(center),np.ones(len(candidates))))
    z=candidates[:,2];best=None;rng=np.random.default_rng(7)
    for _ in range(100):
        idx=rng.choice(len(z),3,replace=False)
        try:plane=np.linalg.solve(a[idx],z[idx])
        except np.linalg.LinAlgError:continue
        if np.linalg.norm(plane[:2])>math.tan(math.radians(10)):continue
        inliers=np.abs(a@plane-z)<.045
        if best is None or inliers.sum()>best.sum():best=inliers
    if best is None or best.sum()<20 or best.mean()<.45:raise ValueError('local floor plane is ambiguous')
    plane=np.linalg.lstsq(a[best],z[best],rcond=None)[0]
    residual=np.sqrt(np.mean((a[best]@plane-z[best])**2))
    if residual>.04 or abs(plane[2]-expected_height)>.35:raise ValueError('ground residual or height outside validated envelope')
    return plane,{'candidate_cells':len(candidates),'inlier_cells':int(best.sum()),'rmse_m':float(residual),
        'slope_deg':math.degrees(math.atan(np.linalg.norm(plane[:2])))}

def classify(points,center,plane):
    points=np.asarray(points,dtype=float)
    dz=points[:,2]-(points[:,:2]-np.asarray(center))@np.asarray(plane[:2])-plane[2]
    return np.abs(dz)<.07,(dz>.12)&(dz<1.8)

def observed_front_support(points,center,yaw,plane):
    ground,_=classify(points,center,plane)
    local=(points[ground,:2]-np.asarray(center))@np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    # Require observed points in each of six future leading-edge patches.
    return all(np.count_nonzero((abs(local[:,0]-x)<.18)&(abs(local[:,1]-y)<.18))>=2
               for x in (.7,1.05) for y in (-.25,0.,.25))


def forward_lower_surface(points,center,yaw,plane):
    """Actual below-floor returns in the forward corridor; unknown stays unknown."""
    points=np.asarray(points,dtype=float)
    local=(points[:,:2]-np.asarray(center))@np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    dz=points[:,2]-(points[:,:2]-np.asarray(center))@np.asarray(plane[:2])-plane[2]
    return bool(np.any(np.isfinite(points).all(axis=1)&(local[:,0]>0)&(local[:,0]<1.23)&
                       (abs(local[:,1])<.43)&(dz<-.12)))


def ground_decision(policy,observed_support,lower_surface):
    """Called only after a valid, fresh local floor fit, never on fit errors.

    flat_floor is an explicit operating assumption for the requested flat-floor
    test. It does not certify unobserved terrain or change support observations.
    """
    if policy not in ('observed_support','flat_floor'):
        raise ValueError('Unknown ground policy')
    return not lower_surface and (policy=='flat_floor' or observed_support)
