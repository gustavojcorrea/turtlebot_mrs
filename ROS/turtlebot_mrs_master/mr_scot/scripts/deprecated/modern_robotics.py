import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

'''
*** BASIC HELPER FUNCTIONS ***
'''

def NearZero(z):
#Takes a scalar.
#Checks if the scalar is small enough to be neglected.
    '''
Example Input:
z = -1e-7
Output:
True
    '''
    return abs(z) < 1e-6
   
def Normalize(V):
#Takes a vector.
#Scales it to a unit vector.
    '''
Example Input: 
V = [1, 2, 3]
Output:
[0.2672612419124244, 0.5345224838248488, 0.8017837257372732]
    '''
    return V / np.linalg.norm(V)

'''
*** CHAPTER 3: RIGID-BODY MOTIONS ***
'''

def RotInv(R):
#Takes a 3x3 rotation matrix.
#Returns the inverse (transpose).
    '''
Example Input: 
R = [[0, 0, 1],
     [1, 0, 0],
     [0, 1, 0]]
Output:
[[0, 1, 0], 
 [0, 0, 1],
 [1, 0, 0]]
    '''
    return np.array(R).T

def VecToso3(omg):
#Takes a 3-vector (angular velocity).
#Returns the skew symmetric matrix in so3.
    '''
Example Input: 
omg = [1, 2, 3]
Output:
[[ 0, -3,  2],
 [ 3,  0, -1],
 [-2,  1,  0]]
    '''
    return [[0,      -omg[2],  omg[1]], 
	    [omg[2],       0, -omg[0]], 
	    [-omg[1], omg[0],       0]]

def so3ToVec(so3mat):
#Takes a 3x3 skew-symmetric matrix (an element of so(3)).
#Returns the corresponding vector (angular velocity).
    '''
Example Input: 
so3mat = [[ 0, -3,  2],
          [ 3,  0, -1],
          [-2,  1,  0]]
Output:
[1, 2, 3]
    '''
    return [so3mat[2][1], so3mat[0][2], so3mat[1][0]]

def AxisAng3(expc3):
#Takes A 3-vector of exponential coordinates for rotation.
#Returns unit rotation axis omghat and the corresponding rotation angle
#theta.
    '''
Example Input: 
expc3 = [1, 2, 3]
Output:
([0.2672612419124244, 0.5345224838248488, 0.8017837257372732],
 3.7416573867739413) 
    '''
    return (Normalize(expc3),np.linalg.norm(expc3))

def MatrixExp3(so3mat):
#Takes a so(3) representation of exponential coordinates.
#Returns R in SO(3) that is achieved by rotating about omghat by theta from
#an initial orientation R = I.
    '''
Example Input: 
so3mat = [[ 0, -3,  2],
	  [ 3,  0, -1],
          [-2,  1,  0]]
Output:
[[-0.69492056,  0.71352099,  0.08929286],
 [-0.19200697, -0.30378504,  0.93319235],
 [ 0.69297817,  0.6313497 ,  0.34810748]]
    '''
    omgtheta = so3ToVec(so3mat)
    if NearZero(np.linalg.norm(omgtheta)):
        return np.eye(3)
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = so3mat / theta
        return np.eye(3) + np.sin(theta) * omgmat \
               + (1 - np.cos(theta)) * np.dot(omgmat,omgmat)

def MatrixLog3(R):
#Takes R (rotation matrix).
#Returns the corresponding so(3) representation of exponential coordinates.
    '''
Example Input: 
R = [[0, 0, 1],
     [1, 0, 0],
     [0, 1, 0]]
Output:
[[          0, -1.20919958,  1.20919958],
 [ 1.20919958,           0, -1.20919958],
 [-1.20919958,  1.20919958,           0]]
    '''
    if NearZero(np.linalg.norm(R - np.eye(3))):
        return np.zeros(3,3)
    elif NearZero(np.trace(R) + 1):
        if not NearZero(1 + R[2][2]):
            omg = (1.0 / sqrt(2 * (1 + R[2][2]))) \
                  * np.array([R[0][2], R[1][2], 1 + R[2][2]])
        elif not NearZero(1 + R[1][1]): 
            omg = (1.0 / sqrt(2 * (1 + R[1][1]))) \
                  * np.array([R[0][1], 1 + R[1][1], R[2][1]])
        else:
            omg = (1.0 / sqrt(2 * (1 + R[0][0]))) \
                  * np.array([1 + R[0][0], R[1][0], R[2][0]])
        return VecToso3(pi*omg)
    else:
        acosinput = (np.trace(R) - 1) / 2.0
        if acosinput > 1:
            acosinput = 1
        elif acosinput < -1:
            acosinput = -1		
        theta = acos(acosinput)
        return theta / 2.0 / sin(theta) * (R - np.array(R).T)

def RpToTrans (R,p):
#Takes rotation matrix R and position p. 
#Returns corresponding homogeneous transformation matrix T in SE(3).
    '''
Example Input: 
R = [[1, 0,  0], 
     [0, 0, -1], 
     [0, 1,  0]]
p = [1, 2, 5]
Output:
[[1, 0,  0, 1],
 [0, 0, -1, 2],
 [0, 1,  0, 5],
 [0, 0,  0, 1]]
    '''
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]    

def TransToRp (T):
#Takes transformation matrix T in SE(3). 
#Returns R: The corresponding rotation matrix,
#        p: The corresponding position vector.
    '''
Example Input: 
T = [[1, 0,  0, 0],
     [0, 0, -1, 0],
     [0, 1,  0, 3],
     [0, 0,  0, 1]]
Output:
([[1, 0,  0], 
  [0, 0, -1], 
  [0, 1,  0]],  
[0, 0, 3])
    '''
    R = [[T[0][0], T[0][1], T[0][2]],
         [T[1][0], T[1][1], T[1][2]],
         [T[2][0], T[2][1], T[2][2]]]
    return R, [T[0][3], T[1][3], T[2][3]]

def TransInv(T):
#Takes a transformation matrix T. 
#Returns its inverse.
#Uses the structure of transformation matrices to avoid taking a matrix
#inverse, for efficiency.
    '''
Example Input: 
T = [[1, 0,  0, 0],
     [0, 0, -1, 0],
     [0, 1,  0, 3],
     [0, 0,  0, 1]]
Output:
[[1,  0, 0,  0],
 [0,  0, 1, -3],
 [0, -1, 0,  0],
 [0,  0, 0,  1]]
    '''
    R,p = TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]
    
def VecTose3(V):
#Takes a 6-vector (representing a spatial velocity). 
#Returns the corresponding 4x4 se(3) matrix.
    '''
Example Input: 
V = [1, 2, 3, 4, 5, 6]
Output:
[[ 0, -3,  2, 4], 
 [ 3,  0, -1, 5], 
 [-2,  1,  0, 6], 
 [ 0,  0,  0, 0]]
    '''
    return np.r_[np.c_[VecToso3([V[0], V[1], V[2]]), [V[3], V[4], V[5]]],
                 np.zeros((1,4))]

def se3ToVec(se3mat):
#Takes se3mat a 4x4 se(3) matrix.
#Returns the corresponding 6-vector (representing spatial velocity).
    '''
Example Input: 
se3mat = [[ 0, -3,  2, 4], 
          [ 3,  0, -1, 5], 
          [-2,  1,  0, 6], 
          [ 0,  0,  0, 0]]
Output:
[1, 2, 3, 4, 5, 6]
    '''
    return np.r_[[se3mat[2][1], se3mat[0][2], se3mat[1][0]],
                 [se3mat[0][3], se3mat[1][3], se3mat[2][3]]]

def Adjoint(T):
#Takes T a transformation matrix SE(3).
#Returns the corresponding 6x6 adjoint representation [AdT].
    '''
Example Input: 
T = [[1, 0,  0, 0], 
     [0, 0, -1, 0], 
     [0, 1,  0, 3], 
     [0, 0,  0, 1]]
Output:
[[1, 0,  0, 0, 0,  0],
 [0, 0, -1, 0, 0,  0],
 [0, 1,  0, 0, 0,  0],
 [0, 0,  3, 1, 0,  0],
 [3, 0,  0, 0, 0, -1],
 [0, 0,  0, 0, 1,  0]]
    '''
    R,p = TransToRp(T)
    return np.r_[np.c_[R, np.zeros((3,3))],
                 np.c_[np.dot(VecToso3(p),R), R]]

def ScrewToAxis(q,s,h):
#Takes q: A point lying on the screw axis, 
#      s: A unit vector in the direction of the screw axis,
#      h: The pitch of the screw axis.
#Returns the corresponding normalized screw axis.
    '''
Example Input: 
q = [3, 0, 0]
s = [0, 0, 1]
h = 2
Output:
[0, 0, 1, 0, -3, 2]
    '''
    return np.r_[s, np.cross(q,s) + np.dot(h,s)]

def AxisAng6(expc6):
#Takes a 6-vector of exponential coordinates for rigid-body motion S*theta.
#Returns S: The corresponding normalized screw axis,
#        theta: The distance traveled along/about S.
    '''
Example Input: 
expc6 = [1, 0, 0, 1, 2, 3]
Output:
([1.0, 0.0, 0.0, 1.0, 2.0, 3.0], 
1.0)
    '''
    theta = np.linalg.norm([expc6[0], expc6[1], expc6[2]])
    if NearZero(theta):
        theta = np.linalg.norm([expc6[3], expc6[4], expc6[5]])
    return (expc6 / theta,theta)

def MatrixExp6(se3mat):
#Takes a se(3) representation of exponential coordinates.
#Returns a T matrix SE(3) that is achieved by traveling along/about the
#screw axis S for a distance theta from an initial configuration T = I.
    '''
Example Input: 
se3mat = [[0,                 0,                  0,                 0],
          [0,                 0, -1.570796326794897, 2.356194490192345],
          [0, 1.570796326794897,                  0, 2.356194490192345],
          [0,                 0,                  0,                 0]]
Output:
[[1.0, 0.0,  0.0, 0.0],
 [0.0, 0.0, -1.0, 0.0],
 [0.0, 1.0,  0.0, 3.0],
 [  0,   0,    0,   1]]
    '''  
    omgtheta = so3ToVec(np.array(se3mat)[0:3:1,0:3:1])
    if NearZero(np.linalg.norm(omgtheta)):
        return np.r_[np.c_[np.eye(3),
                           [se3mat[0][3],se3mat[1][3],se3mat[2][3]]],
                     [[0, 0, 0, 1]]]
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = np.array(se3mat)[0:3,0:3] / theta
        return np.r_[np.c_[MatrixExp3(np.array(se3mat)[0:3:1,0:3:1]),
                           np.dot(np.eye(3) * theta \
                                  + (1 - np.cos(theta)) * omgmat \
                                  + (theta - np.sin(theta)) \
                                    * np.dot(omgmat,omgmat),
                                  [se3mat[0][3],
                                   se3mat[1][3],
                                   se3mat[2][3]]) / theta],
                     [[0, 0, 0, 1]]]

def MatrixLog6(T):
#Takes a transformation matrix T in SE(3).
#Returns the corresponding se(3) representation of exponential coordinates.
    '''
Example Input: 
T = [[1,0,0,0], [0,0,-1,0], [0,1,0,3], [0,0,0,1]]
Output:
[[ 0.          0.          0.          0.        ]
 [ 0.          0.         -1.57079633  2.35619449]
 [ 0.          1.57079633  0.          2.35619449]
 [ 0.          0.          0.          0.        ]]

    '''
    R,p = TransToRp(T)
    if NearZero(np.linalg.norm(R - np.eye(3))):
        return np.r_[np.c_[np.zeros((3,3)),
                           [T[0][3], T[1][3], T[2][3]]],
                     [[0, 0, 0, 0]]]
    else: 
        acosinput = (np.trace(R) - 1) / 2.0
        if acosinput > 1:
            acosinput = 1
        elif acosinput < -1:
            acosinput = -1		
        theta = acos(acosinput)       
        omgmat = MatrixLog3(R) 
        return np.r_[np.c_[omgmat, 
                           np.dot(np.eye(3) - omgmat / 2.0 \
                           + (1.0 / theta - 1.0 / tan(theta / 2.0) / 2) \
                             * np.dot(omgmat,omgmat) / theta,[T[0][3], 
                                                              T[1][3], 
                                                              T[2][3]])], 
                     [[0, 0, 0, 0]]]
