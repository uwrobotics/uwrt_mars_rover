######################
# init
######################
import os
import sys
ROOT_DIR = os.path.abspath("../")
sys.path.append(ROOT_DIR)
print("********* cwd {} *********".format(ROOT_DIR))

import numpy as np
import scipy.io as scio
import matplotlib.pyplot as plt

DEBUG = False
SHOW_PLOT = True

if __name__ == '__main__':

    ##################
    # INIT
    ##################
    mat_file = ROOT_DIR + '/MATLAB/uwrt_arm_matlab_sim_1000000.mat'
    min_mesh_file = ROOT_DIR + '/meshes/min/workspace_markers.txt'
    mean_mesh_file = ROOT_DIR + '/meshes/mean/workspace_markers.txt'
    max_mesh_file = ROOT_DIR + '/meshes/max/workspace_markers.txt'

    # assuming your vector is in a text file
    # xyz_points = np.genfromtxt(ROOT_DIR + '/meshes/mean/mean_workspace_markers.xyz', dtype=np.float)
    xyz_points = np.asarray(scio.loadmat(mat_file)['end_effector_points'])

    z = xyz_points[:, -1]
    idx = np.argsort(z)

    sorted_xyz_points = xyz_points[idx, :]

    z_slice = 5e-6 # 1e-3 = 1 [mm]
    z_interval = (max(z) - min(z)) / z_slice
    z_increments = int(len(z) / z_interval)

    xyz_min, xyz_max, xyz_mean = [], [], []
    for i, _ in enumerate(range(0, len(z), z_increments)):
        z_idxs = np.arange((i)*z_increments, (i+1)*z_increments, 1)
        # check if we've gone past last idx
        if len(z) in z_idxs:
            break

        if DEBUG:
            print("Z Indexes: ", z_idxs)

        sorted_xy_points = sorted_xyz_points[z_idxs, :]

        x_idxs = np.argsort(sorted_xy_points[:, 0])
        sorted_x_points = sorted_xy_points[x_idxs, :]

        mean_x, mean_y = np.mean(sorted_xy_points[:, 0]), np.mean(sorted_xy_points[:, 1])
        min_from_origin, max_from_origin, mean_from_origin, min_idx, max_idx, mean_idx \
            = np.inf, -np.inf, np.inf, None, None, None
        for x_idx in x_idxs:
            xy_pair = sorted_xy_points[x_idx, :][0:2]
            diff_origin = np.linalg.norm((0-xy_pair[0], 0-xy_pair[1]))
            diff_mean = np.linalg.norm((mean_x-xy_pair[0], mean_y-xy_pair[1]))

            # min
            if diff_origin < min_from_origin:
                min_from_origin = diff_origin
                min_idx = x_idx

            # max
            if diff_origin > max_from_origin:
                max_from_origin = diff_origin
                max_idx = x_idx

            # mean
            if diff_mean < mean_from_origin:
                mean_from_origin = diff_mean
                mean_idx = x_idx

        min_point = sorted_xy_points[min_idx, :].tolist()
        max_point = sorted_xy_points[max_idx, :].tolist()
        mean_point = np.mean(np.array([min_point, max_point]), axis=0)

        xyz_min.append(min_point)
        xyz_max.append(max_point)
        xyz_mean.append(mean_point)

    xyz_min = np.array(xyz_min)
    # xyz_min_idx = [np.where(sorted_xyz_points == xyz_min[i, :])[0] for i in range(len(xyz_min))]

    xyz_max = np.array(xyz_max)
    # xyz_max_idx = [np.where(sorted_xyz_points == xyz_max[i, :])[0] for i in range(len(xyz_max))]

    # xyz_mean = np.delete(sorted_xyz_points, xyz_min_idx).reshape(-1, 3)
    # xyz_mean = np.delete(xyz_mean, xyz_max_idx).reshape(-1, 3)
    xyz_mean = np.array(xyz_mean)

    ### save outputs
    np.savetxt(min_mesh_file, xyz_min)
    np.savetxt(mean_mesh_file, xyz_mean)
    np.savetxt(max_mesh_file, xyz_max)

    if SHOW_PLOT:
        ################
        # plot
        ################
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(xyz_min[:, 0], xyz_min[:, 1], xyz_min[:, 2], c='r', marker='x', alpha=0.75)
        ax.scatter(xyz_max[:, 0], xyz_max[:, 1], xyz_max[:, 2], c='m', marker='x', alpha=0.75)
        ax.scatter(xyz_mean[:, 0], xyz_mean[:, 1], xyz_mean[:, 2], c='g', marker='.', alpha=0.25)

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        plt.show()
