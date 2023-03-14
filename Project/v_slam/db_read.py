import msgpack
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as R

with open("map.msg", "rb") as f:
    u = msgpack.Unpacker(f)
    msg = u.unpack()


keyfrms = msg["keyframes"]
landmarks = msg["landmarks"]

keyfrms_tum = []
for keyfrm in keyfrms.values():
    # get conversion from world to camera
    trans_cw = np.matrix(keyfrm["trans_cw"]).T
    rot_cw = R.from_quat(keyfrm["rot_cw"]).as_matrix()
    # compute conversion from camera to world
    rot_wc = rot_cw.T
    trans_wc = rot_wc * trans_cw
    # The following conversion should actually be correct, but it has been flipped.                  
    # trans_wc = - rot_wc * trans_cw                                                                 
    keyfrms_tum.append((keyfrm["ts"], trans_wc.tolist(), R.from_matrix(rot_wc).as_quat().tolist()))
keyfrms_tum.sort(key=lambda k: k[0])
for keyfrm in keyfrms_tum:
    print("{} {} {} {} {} {} {} {}".format(keyfrm[0], keyfrm[1][0][0], keyfrm[1][1][0], keyfrm[1][2]\
[0], keyfrm[2][0], keyfrm[2][1], keyfrm[2][2], keyfrm[2][3]))
