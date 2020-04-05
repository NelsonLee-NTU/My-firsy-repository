import numpy as np
import scipy.io as sio
from liegroups import SE3
import matplotlib.pyplot as plt

def test_poses(poses):
    trans = np.empty((2,poses.shape[0]))
    rot = np.empty((poses.shape[0],1))
    for i in range(poses.shape[0]):
        se3mat = SE3.from_matrix(poses[i,:,:])
        trans[:, i] = -se3mat.inv().trans[0:2]
        rot[i, :] = se3mat.rot.log()[2]
    
    plt.figure()
    plt.plot(rot, c='k')
    plt.plot(trans[0, :], c='r')
    plt.plot(trans[1, :], c='g')
    plt.show()

    return
