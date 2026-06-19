import json
import numpy as np
import pandas as pd
from dateutil import parser
import cv2
from skspatial.objects import Plane, Points

#############################################################################
###                                Options                                ###
#############################################################################
# If False, asuumes that the LH camera matrix is the identity.
# If True, uses Mat_A and Mat_B
USE_CAMERA_MATRIX = False    

# Define the intrinsic camera matrices of the LH2 basestations
Mat_A = np.array([[ 1.     ,  0.    , 0.],
                  [ 0.     ,  1.    , 0.],
                  [ 0.     ,  0.    , 1.]])

Mat_B = np.array([[ 1.      , 0.    , 0.],
                  [ 0.      , 0.    , 0.],
                  [ 0.      , 0.    , 1.]])

#############################################################################
###                                Functions                              ###
#############################################################################

def LH2_count_to_pixels(count_1, count_2, mode):
    """
    Convert the sweep count from a single lighthouse into pixel projected onto the LH2 image plane
    ---
    count_1 - int - polinomial count of the first sweep of the lighthouse
    count_2 - int - polinomial count of the second sweep of the lighthouse
    mode - int [0,1] - mode of the LH2, let's you know which polynomials are used for the LSFR. and at which speed the LH2 is rotating.
    """
    periods = [959000, 957000]

    # Translate points into position from each camera
    a1 = (count_1*8/periods[mode])*2*np.pi  # Convert counts to angles traveled in the weird 40deg planes, in radians
    a2 = (count_2*8/periods[mode])*2*np.pi   

    # Transfor sweep angles to azimuth and elevation coordinates
    azimuth   = (a1+a2)/2 
    elevation = np.pi/2 - np.arctan2(np.sin(a2/2-a1/2-60*np.pi/180),np.tan(np.pi/6)) 

    # Project the angles into the z=1 image plane
    pts_lighthouse = np.zeros((len(count_1),2))
    for i in range(len(count_1)):
        pts_lighthouse[i,0] = -np.tan(azimuth[i])
        pts_lighthouse[i,1] = -np.sin(a2[i]/2-a1[i]/2-60*np.pi/180)/np.tan(np.pi/6) * 1/np.cos(azimuth[i])

    # Return the projected points
    return pts_lighthouse

def LH2_angles_to_pixels(azimuth, elevation):
    """
    Project the Azimuth and Elevation angles of a LH2 basestation into the unit image plane.
    """
    pts_lighthouse = np.array([np.tan(azimuth),         # horizontal pixel  
                               np.tan(elevation) * 1/np.cos(azimuth)]).T    # vertical   pixel 
    return pts_lighthouse

def solve_3d_scene(pts_a, pts_b):
    """
    Use the projected LH2-camera points to triangulate the position of the LH2  basestations and of the LH2 receiver
    """
    # Obtain translation and rotation vectors
    F, mask = cv2.findFundamentalMat(pts_a, pts_b, cv2.FM_LMEDS)
    if USE_CAMERA_MATRIX:
        points, R_star, t_star, mask = cv2.recoverPose(Mat_A @ F @ Mat_B, pts_a, pts_b)
    else:
        points, R_star, t_star, mask = cv2.recoverPose(F, pts_a, pts_b)

    # Triangulate the points
    R_1 = np.eye(3,dtype='float64')
    t_1 = np.zeros((3,1),dtype='float64')

    # Calculate the Projection Matrices P
    # The weird transpose everywhere is  because cv2.recover pose gives you the Camera 2 to Camera 1 transformation (which is the backwards of what you want.)
    # To get the Cam1 -> Cam2 transformation, we need to invert this.
    # R^-1 => R.T  (because rotation matrices are orthogonal)
    # inv(t) => -t 
    # That's where all the transpositions and negatives come from.
    # Source: https://stackoverflow.com/a/45722936
    if USE_CAMERA_MATRIX:
        P1 = Mat_A @ np.hstack([R_1.T, -R_1.T.dot(t_1)])
        P2 = Mat_B @ np.hstack([R_star.T, -R_star.T.dot(t_star)])  
    else:
        P1 = np.hstack([R_1.T, -R_1.T.dot(t_1)])
        P2 = np.hstack([R_star.T, -R_star.T.dot(t_star)])  
    # The projection matrix is the intrisic matrix of the camera multiplied by the extrinsic matrix.
    # When the intrinsic matrix is the identity.
    # The results is the [ Rotation | translation ] in a 3x4 matrix

    # FIXME (Claude): point sets are SWAPPED relative to their projection
    # matrices. P1 belongs to camera A (built from R_1, t_1) and P2 belongs to
    # camera B (built from R_star, t_star), so pts_a must be paired with P1 and
    # pts_b with P2. Passing pts_b with P1 and pts_a with P2 makes
    # triangulatePoints solve an inconsistent system: the reconstruction
    # collapses toward a plane (one axis loses almost all its extent).
    # Verified with a known scene + known poses: correct order -> 0.00 mm MAE,
    # this swapped order -> ~165 mm MAE.
    # Fix: cv2.triangulatePoints(P1, P2, pts_a.T, pts_b.T)
    point3D = cv2.triangulatePoints(P1, P2, pts_b.T, pts_a.T).T
    point3D = point3D[:, :3] / point3D[:, 3:4]

    # Return the triangulated 3D points
    # Return the position and orientation of the LH2-B wrt LH2-A
    return point3D, t_star, R_star

def solve_3d_scene_get_Rt(pts_a, pts_b):
    """
    Use the projected LH2-camera points to triangulate the position of the LH2  basestations and of the LH2 receiver
    """
    # Obtain translation and rotation vectors
    F, mask = cv2.findFundamentalMat(pts_a, pts_b, cv2.FM_LMEDS)
    if USE_CAMERA_MATRIX:
        points, R_star, t_star, mask = cv2.recoverPose(Mat_A @ F @ Mat_B, pts_a, pts_b)
    else:
        points, R_star, t_star, mask = cv2.recoverPose(F, pts_a, pts_b)

    return t_star, R_star

def solve_3d_scene_triangulate_points(pts_a, pts_b, t_star, R_star):

   # Triangulate the points
    R_1 = np.eye(3,dtype='float64')
    t_1 = np.zeros((3,1),dtype='float64')

    # Calculate the Projection Matrices P
    # The weird transpose everywhere is  because cv2.recover pose gives you the Camera 2 to Camera 1 transformation (which is the backwards of what you want.)
    # To get the Cam1 -> Cam2 transformation, we need to invert this.
    # R^-1 => R.T  (because rotation matrices are orthogonal)
    # inv(t) => -t 
    # That's where all the transpositions and negatives come from.
    # Source: https://stackoverflow.com/a/45722936
    if USE_CAMERA_MATRIX:
        P1 = Mat_A @ np.hstack([R_1.T, -R_1.T.dot(t_1)])
        P2 = Mat_B @ np.hstack([R_star.T, -R_star.T.dot(t_star)])  
    else:
        P1 = np.hstack([R_1.T, -R_1.T.dot(t_1)])
        P2 = np.hstack([R_star.T, -R_star.T.dot(t_star)])  
    # The projection matrix is the intrisic matrix of the camera multiplied by the extrinsic matrix.
    # When the intrinsic matrix is the identity.
    # The results is the [ Rotation | translation ] in a 3x4 matrix

    # FIXME (Claude): same point-ordering bug as in solve_3d_scene above.
    # pts_a goes with P1, pts_b goes with P2 -- here they are swapped, which
    # corrupts the triangulated points (near-planar collapse, ~165 mm error
    # on a known test scene that should reconstruct to ~0 mm).
    # Fix: cv2.triangulatePoints(P1, P2, pts_a.T, pts_b.T)
    point3D = cv2.triangulatePoints(P1, P2, pts_b.T, pts_a.T).T
    point3D = point3D[:, :3] / point3D[:, 3:4]

    # Return the triangulated 3D points
    # Return the position and orientation of the LH2-B wrt LH2-A
    return point3D

def scale_scene_to_real_size(df):
    """
    Code takes the solved 3D scene and scales the scene so that the distance between the gridpoints is indeed 40mm

    --- Input
    df: dataframe with the triangulated position of the grid-points and the real position of the grid-points
    --- Output
    df: dataframe with the updated scaled-up scene
    """
    # Grab the point at (0,0,0) mm and (40,0,0) mm and use them to calibrate/scale the system.
    scale_p1 = df.loc[(df['real_x_mm'] == 0)  & (df['real_y_mm'] == 0) & (df['real_z_mm'] == 0), ['LH_x', 'LH_y', 'LH_z']].values.mean(axis=0)
    scale_p2 = df.loc[(df['real_x_mm'] == 40) & (df['real_y_mm'] == 0) & (df['real_z_mm'] == 0), ['LH_x', 'LH_y', 'LH_z']].values.mean(axis=0)
    scale = 40 / np.linalg.norm(scale_p2 - scale_p1)
    # Scale all the points
    df['LH_x'] *= scale
    df['LH_y'] *= scale
    df['LH_z'] *= scale

    # Return scaled up scene
    return df

def compute_distance_between_grid_points(df):
    """
    Code that calculates the mean error and std deviation of the distance between grid points.

    --- Input
    df: dataframe with the scaled triangulated position of the grid-points and the real position of the grid-points
    --- Output
    x_dist: array float - X-axis distances between adjacent grid-points 
    y_dist: array float - Y-axis distances between adjacent grid-points 
    z_dist: array float - Z-axis distances between adjacent grid-points 
    """

    ##################### GET X AXIS DISTANCES
    x_dist = []
    for y in [0, 40, 80, 120]:
        for z in [0, 40, 80, 120, 160]:
            for x in [0, 40, 80, 160, 200]:  # We are missing x=240 because we only want the distance between the points, not the actual points.
                # Grab all the points
                p1 = df.loc[(df['real_x_mm'] == x)  & (df['real_y_mm'] == y) & (df['real_z_mm'] == z), ['LH_x', 'LH_y', 'LH_z']].values
                p2 = df.loc[(df['real_x_mm'] == x+40)  & (df['real_y_mm'] == y) & (df['real_z_mm'] == z), ['LH_x', 'LH_y', 'LH_z']].values

                # Now permute all the distances between all the points in each position
                for v in p1:
                    for w in p2:
                        x_dist.append(np.linalg.norm(v-w))

    ##################### GET Y AXIS DISTANCES
    y_dist = []
    for y in [0, 40, 80]:        # We are missing y=120 because we only want the distance between the points, not the actual points.
        for z in [0, 40, 80, 120, 160]:
            for x in [0, 40, 80, 160, 200, 240]: 
                # Grab all the points
                p1 = df.loc[(df['real_x_mm'] == x)  & (df['real_y_mm'] == y) & (df['real_z_mm'] == z), ['LH_x', 'LH_y', 'LH_z']].values
                p2 = df.loc[(df['real_x_mm'] == x)  & (df['real_y_mm'] == y+40) & (df['real_z_mm'] == z), ['LH_x', 'LH_y', 'LH_z']].values

                # Now permute all the distances between all the points in each position
                for v in p1:
                    for w in p2:
                        y_dist.append(np.linalg.norm(v-w))

    ##################### GET Z AXIS DISTANCES
    z_dist = []
    for y in [0, 40, 80, 120]:
        for z in [0, 40, 80, 120]:       # We are missing z=160 because we only want the distance between the points, not the actual points.
            for x in [0, 40, 80, 160, 200, 240]: 
                # Grab all the points
                p1 = df.loc[(df['real_x_mm'] == x)  & (df['real_y_mm'] == y) & (df['real_z_mm'] == z), ['LH_x', 'LH_y', 'LH_z']].values
                p2 = df.loc[(df['real_x_mm'] == x)  & (df['real_y_mm'] == y) & (df['real_z_mm'] == z+40), ['LH_x', 'LH_y', 'LH_z']].values

                # Now permute all the distances between all the points in each position
                for v in p1:
                    for w in p2:
                        z_dist.append(np.linalg.norm(v-w))

    # At the end, put all the distances together in an array and calculate mean and std
    x_dist = np.array(x_dist)
    y_dist = np.array(y_dist)
    z_dist = np.array(z_dist)
    # Remove ouliers, anything bigger than 1 meters gets removed.
    x_dist = x_dist[x_dist <= 500]
    y_dist = y_dist[y_dist <= 500]
    z_dist = z_dist[z_dist <= 500]

    return x_dist, y_dist, z_dist

def correct_perspective(df):
    """
    Create a rotation and translation vector to move the reconstructed grid onto the origin for better comparison.
    Using an SVD, according to: https://nghiaho.com/?page_id=671
    """
    
    B = np.unique(df[['real_x_mm','real_y_mm','real_z_mm']].to_numpy(), axis=0)
    A = np.empty_like(B, dtype=float)
    for i in range(B.shape[0]):
        A[i] = df.loc[(df['real_x_mm'] == B[i,0])  & (df['real_y_mm'] == B[i,1]) & (df['real_z_mm'] == B[i,2]), ['LH_x', 'LH_y', 'LH_z']].values.mean(axis=0)

    # Get  all the reconstructed points
    A2 = df[['LH_x','LH_y','LH_z']].to_numpy().T

    # Convert the point to column vectors,
    # to match twhat the SVD algorithm expects
    A = A.T
    B = B.T

    # Get the centroids
    A_centroid = A.mean(axis=1).reshape((-1,1))
    B_centroid = B.mean(axis=1).reshape((-1,1))

    # Get H
    H = (A - A_centroid) @ (B - B_centroid).T

    # Do the SVD
    U, S, V = np.linalg.svd(H)

    # Get the rotation matrix
    R = V @ U.T

    # check for errors, and run the correction
    if np.linalg.det(R) < 0:
        U, S, V = np.linalg.svd(R)
        V[:,2] = -1*V[:,2]
        R = V @ U.T

    # Get the ideal translation
    t = B_centroid - R @ A_centroid

    correct_points = (R@A2 + t)
    correct_points = correct_points.T

    # Update dataframe
    df['Rt_x'] = correct_points[:,0]
    df['Rt_y'] = correct_points[:,1]
    df['Rt_z'] = correct_points[:,2]
    return df

def compute_errors(df):
    """Calculate MAE, RMS and Precision for a particular reconstruction"""
    # Extract needed data from the main dataframe
    points = df[['Rt_x','Rt_y','Rt_z']].to_numpy()
    ground_truth = df[['real_x_mm','real_y_mm','real_z_mm']].to_numpy()

    # Calculate distance between points and their ground truth
    errors =  np.linalg.norm(ground_truth - points, axis=1)
    # print the mean and standard deviation
    mae = errors.mean()
    rmse = np.sqrt((errors**2).mean())
    std = errors.std()

    return mae, rmse, std

def is_coplanar(points):
    """
    taken from the idea here: https://stackoverflow.com/a/72384583
    returns True or False depending if the points are too coplanar or not.
    """

    best_fit = Plane.best_fit(points)
    distances = np.empty((points.shape[0]))

    for i in range(points.shape[0]):
        distances[i] = best_fit.distance_point(points[i])

    error = distances.mean()
    return error

def compute_mad(points):
    """ Get a list of 3d points and calculate the Median Absolute Deviation"""

    centroid = points.mean(axis=0)
    return np.linalg.norm(points - centroid, axis=1).mean()
