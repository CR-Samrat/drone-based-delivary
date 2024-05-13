import numpy as np

def distance(u,v):
    d=u-v
    return (np.sqrt(np.dot(d,d.T)))

def centroid(XY):
    x=[i for i,j in XY]
    y=[j for i,j in XY]
    mxy=np.zeros((2))
    mxy[0]=np.mean(x)
    mxy[1]=np.mean(y)
    return(mxy)


def gradient(c,XY):
    dxy=np.zeros((2))
    for i in range(len(XY)):
        d=distance(c,XY[i])
        dxy =dxy+(1/d)*(c-XY[i])
    return(dxy)

def total(c,XY):
    T=0
    for x in XY:
        T=T+distance(c,x)
    return (T)

def descent(XY, M): #find a point M such that sum of the distances of the points in XY from M is minimum
    #M=np.array([10,20])
    lr=0.001
    for i in range(50):
        print ("total",total(M,XY),"M",M)
        delta =lr*gradient(M,XY)
        if (np.dot(delta,delta.T))<.01:
            break
        M=M-delta
    return M

def optimize_dp_using_gradient_method(xy):
    M=centroid(xy)
    # M = np.array(dp)

    print (M)
    print (gradient(M,xy))

    C=descent(xy, M)
    print ("minimum point after gradient descent", C)

    return C