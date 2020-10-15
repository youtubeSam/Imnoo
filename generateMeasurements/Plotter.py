
#!/usr/bin/env python
import numpy as np

def plotAxis(ax,x0,dir,col):
    scaler=0.5
    X=np.array([x0,x0 + dir*scaler])
    X=np.reshape(X.transpose(),(3,2)).tolist()
    ax.plot(*X,linewidth=2,c=col)

def plotAxis2d(ax,x0,dir,col):
    scaler=1
    X=np.array([x0,x0 + dir*scaler])
    X=np.reshape(X.transpose(),(2,2)).tolist()
    ax.plot(*X,linewidth=2,c=col)
 
def plotCoordinateFrame(ax,x0,rot):
    col = ["r","g","b"]
    for i in range(3):
        plotAxis(ax,x0,rot[:,i],col[i])

def plotCoordinateFrame2d(ax,x0,rot):
    col = ["r","g","b"]
    for i in range(3):
        plotAxis2d(ax,x0[0:2],rot[0:2,i],col[i])

def plotQuadCoordinateFrame(ax,x0,q):
    R = quadToRot(q)
    plotCoordinateFrame(ax,x0,R)

def showImage(ax,img):
    ax.imshow(img);

def plotTrajectory3d(ax, XYZ):
    h = ax.plot(XYZ[:,0],XYZ[:,1],XYZ[:,2])
    return h

def plotTrajectory2d(ax, XY):
    h = ax.plot(XY[:,0],XY[:,1],'k')
    return h

def plot2dMeasurementPoints(ax, XY):
    h = ax.scatter(XY[:,0],XY[:,1],s=10,marker='x')
    return h

def plot2dMeasurements(ax, pos, measurements):
    h = None
    for m in measurements:
        coords = np.array([pos,m])
        h = ax.plot(coords[:,0],coords[:,1],'r')
    if h == None:
        h = ax.plot(0, 0, 'r')
    return h

def plotFieldOfView(ax,x0,q):
    R = quadToRot(q)
    K = np.array( [[283.6180612179999, 0.0               , 324.01932299260216],
                   [0.0              , 283.59733652086277, 230.77136448609954],
                   [0.0              , 0.0               , 1.0]])
    K_inv = np.linalg.inv(K)

    origin = np.array([x0])
    width  = 640
    height = 480
    depthRange = 5

    vec = np.array([[0,0,1],[0,height,1],[width,0,1],[width,height,1]])
    dirvec = np.array([width/2,height/2,1])
    vectors = R.dot(K_inv.dot(vec.T))+origin.T
    vecDir = (R.dot(K_inv.dot(dirvec.T))+origin)
    line = np.concatenate((origin, vecDir), axis=0).T.tolist()
    ax.plot(*(line), c='r')
    for i in range(4):
        vec = vectors[:,i]
        vec = vec/np.linalg.norm(vec)*depthRange
        transform = np.array([vec.tolist()])
        line = np.concatenate((origin,transform),axis=0).T.tolist()
        ax.plot(*(line),c='k')
