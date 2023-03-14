import open3d as o3d
from mpl_toolkits import mplot3d
import numpy as np
import math
import random
import matplotlib.pyplot as plt

##Enter no. of iteration
N = 500

# read demo point cloud provided by Open3D
pcd_point_cloud = o3d.data.PCDPointCloud()
pcd_original = o3d.io.read_point_cloud(pcd_point_cloud.path)
pcd = np.asarray(pcd_original.points)

#method to calculate coefficients of plane equation
def plane_equation(rand_pts):
    #Finding vector A = pointA - pointO
    a1 =  rand_pts[1][0] - rand_pts[0][0]
    a2 =  rand_pts[1][1] - rand_pts[0][1]
    a3 =  rand_pts[1][2] - rand_pts[0][2]
    A = np.array([[a1,a2,a3]])

    #Finding vector B = pointB - pointO
    b1 =  rand_pts[2][0] - rand_pts[0][0]
    b2 =  rand_pts[2][1] - rand_pts[0][1]
    b3 =  rand_pts[2][2] - rand_pts[0][2]
    B = np.array([[b1,b2,b3]])

    # Finding the plane coefficients a,b,c and D

    C = np.cross(A,B)  ##vector normal to A and B
    D = -(C[0][0]*rand_pts[0][0] + C[0][1]*rand_pts[0][1] + C[0][2]*rand_pts[0][2])
    # D = -(C[0][0]*rand_pts[1][0] + C[0][1]*rand_pts[1][1] + C[0][2]*rand_pts[1][2])
    # D = -(C[0][0]*rand_pts[2][0] + C[0][1]*rand_pts[2][1] + C[0][2]*rand_pts[2][2])
    [a,b,c] = [C[0][0],C[0][1],C[0][2]]
    return a,b,c,D

#calculate the distance from the plane 
def distance(a,b,c,d, pcd):
    
    num = a*pcd[:,0] + b*pcd[:,1] + c*pcd[:,2] + d
    denom = math.sqrt((a*a)+(b*b)+(c*c))
    dist = num/denom

    return a,b,c,d,dist

 #select the inliers based on the threshold distance(0.02)
def inliers(a,b,c,d, pcd):
    a,b,c,d,dist = distance(a,b,c,d,pcd)
    plane_coeff = [a,b,c,d]
    index = np.where(np.abs(dist) <= 0.02)[0]
    inlier_points = pcd[index]

    return inlier_points, plane_coeff

# ransac method
def ransac(N):
    max_inlier = 0
    high_score_coeff = []
    final_inliers = []
    #ransac loop
    for i in range(N):
        rand_pts = np.zeros((3,3))
        # take 3 random points from the point cloud data 
        for j in range(3):
            random_init = random.randint(0,len(pcd)-1)
            rand_pts[j]=pcd[random_init] 
        
        # calculate the plane coefficients
        a,b,c,d = plane_equation(rand_pts)
        inlier_points,plane_coeff = inliers(a,b,c,d,pcd)

        if len(inlier_points) > len(final_inliers):
            final_inliers = inlier_points
            high_score_coeff = plane_coeff
 
    
    # final inliers
    final_inliers = np.array(final_inliers)
    print(final_inliers)

    return final_inliers

final_inliers = ransac(N)


# function to visualize the point cloud
p = o3d.geometry.PointCloud()
p.points = o3d.utility.Vector3dVector(final_inliers)
p.paint_uniform_color([1,0,0])

o3d.visualization.draw_geometries([pcd_original, p],
                                    zoom=1,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])