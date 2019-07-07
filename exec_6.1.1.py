#!/usr/bin/python3
import sys, argparse
import numpy as np
import multiprocessing as mp
import math
import shapely.geometry
import matplotlib.pyplot as plt
import matplotlib.colors as clr

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
    # Initially everything is free space
    c_space = np.full((theta_1_range, theta_2_range), 255, dtype = int)
    print("Computing configuration space...")
    for theta_1 in range(theta_1_range):
        for theta_2 in range(theta_2_range):
            base_tcp = compute_fk(theta_1 * precision, theta_2 * precision, L_1, L_2)
            world_tcp = tf_base_to_world(base_tcp)
            tcp = shapely.geometry.Point(world_tcp)

            # Check if TCP is in collision
            for c_obstacle in c_obstacles:
                if c_obstacle.contains(tcp):
                    c_space[theta_1][theta_2] = 0

    print("Configuration space built!")

    # Add manipulator point at the center of the configuration space
    c_space[int(theta_1_range / 2)][int(theta_2_range / 2)] = 100

    # Plot configuration space
    bounds = [0, 9, 19, 254, 255]
    colormap = clr.ListedColormap(['gray', 'green', 'red', 'white'])
    norm = clr.BoundaryNorm(bounds, colormap.N)
    plt.xlabel("Theta 2")
    plt.ylabel("Theta 1")
    plt.imshow(c_space, cmap=colormap, norm=norm)
    plt.show()

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
