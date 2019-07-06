#!/usr/bin/python3
import sys
import argparse
import numpy as np
import multiprocessing as mp
import math
import shapely.geometry
import matplotlib.pyplot as plt
import matplotlib.colors as clr
import matplotlib.cm as cm

BASE_POS = np.array([500, 500])
L_1, L_2 = 200, 200

_O_1 = (270, 620)
R_1 = 50

_O_2 = (250, 200)
R_2 = 200


def compute_fk(theta_1, theta_2, a_1, a_2):
    """ Compute forward kinematics for 2D planar manipulator. 
        Input angles in degrees.
    """
    theta_1_rad = math.radians(theta_1)
    theta_2_rad = math.radians(theta_2)

    # Position of TCP w.r.t base link
    x = a_1 * math.cos(theta_1_rad) + a_2 * math.cos(theta_1_rad + theta_2_rad)
    y = a_1 * math.sin(theta_1_rad) + a_2 * math.sin(theta_1_rad + theta_2_rad)

    return np.array([x, y])


def tf_base_to_world(position):
    """ Transform base link coordinates to world coordinates. """
    return position + BASE_POS


def calc_cspace(theta_1_fullrange, theta_2_fullrange, precision, c_obstacles, queue, id, numproc):
    t1_start = math.floor(theta_1_fullrange / numproc) * id
    t2_start = math.floor(theta_1_fullrange / numproc) * id
    t1_range = math.floor(theta_1_fullrange / numproc)
    t2_range = math.floor(theta_1_fullrange / numproc)
    if id == numproc - 1:
        t1_range = t1_range
        t2_range = t2_range

    t1_start = 0
    t1_range = theta_1_fullrange

    print("Process %d: t1_start: %3d\tt1_range: %3d\tt2_start: %3d\tt2_range: %3d" % (id, t1_start, t1_range, t2_start, t2_range))

    c_space = np.zeros((theta_1_fullrange, theta_2_fullrange))
    print(range(t2_start, t2_start + t2_range))
    for theta_2 in range(t2_start, t2_start + t2_range):
        for theta_1 in range(t1_start, t1_start + t1_range):
            base_tcp = compute_fk(theta_1 * precision, theta_2 * precision, L_1, L_2)
            world_tcp = tf_base_to_world(base_tcp)

            # Since we assume the TCP is a circle with radius 2
            tcp = shapely.geometry.Point(world_tcp).buffer(2)

            c_space[theta_2][theta_1] = 255

            # Check if TCP is in collision
            for c_obstacle in c_obstacles:
                if c_obstacle.intersects(tcp):
                    c_space[theta_2][theta_1] = 0
                    # print("Configuration collision: {}".format((theta_1 * precision, theta_2 * precision)))
    queue.put((c_space))


def main(args):
    precision = args.precision
    # Build C-Obstacles by adding radius of TCP to the
    # obstacle radius
    O_1 = shapely.geometry.Point(_O_1).buffer(R_1 + 2)
    O_2 = shapely.geometry.Point(_O_2).buffer(R_2 + 2)
    c_obstacles = [O_1, O_2]

    theta_1_range = int(360 / precision) + 1 # include upper bound
    theta_2_range = int(360 / precision) + 1

    # Build configuration space
    # FIXME: Slow as fuck, it needs to be run on multiple processors
    c_space = np.zeros((theta_1_range, theta_2_range))
    print(c_space)
    queue = mp.Queue()
    p0 = mp.Process(target=calc_cspace, args=(theta_1_range, theta_2_range, precision, c_obstacles, queue, 0, 4))
    p1 = mp.Process(target=calc_cspace, args=(theta_1_range, theta_2_range, precision, c_obstacles, queue, 1, 4))
    p2 = mp.Process(target=calc_cspace, args=(theta_1_range, theta_2_range, precision, c_obstacles, queue, 2, 4))
    p3 = mp.Process(target=calc_cspace, args=(theta_1_range, theta_2_range, precision, c_obstacles, queue, 3, 4))
    p0.start()
    p1.start()
    p2.start()
    p3.start()
    np.add(c_space, queue.get(), c_space)
    np.add(c_space, queue.get(), c_space)
    np.add(c_space, queue.get(), c_space)
    np.add(c_space, queue.get(), c_space)
    p0.join()
    p1.join()
    p2.join()
    p3.join()
    print(c_space.shape)

    # Need to flip the array because numpy access them row first
    # are accessed row first.
    c_space = np.flip(c_space, 0)
    c_space = np.flip(c_space, 1)

    # Plot configuration space
    # TODO: Better plotting with right axes
    colormap = clr.ListedColormap(['gray', 'white'])
    plt.imshow(c_space, cmap=colormap)
    plt.show()

    # TODO: Plot work and configuration space with
    #       start areas.

    # TODO: Find path and plot it in the configuration space

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description = "Compute and plot configuration space for 2D manipulator for exec 6.1.")
    parser.add_argument(
		  "precision",
                  type=int,
                  nargs="?",
                  default=1,
		  help = "pass precision for configuration space",
		  metavar = "P")
    args = parser.parse_args() 

    main(args)

