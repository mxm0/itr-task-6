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

_G_1 = (580, 150)
_G_2 = (230, 470)
_S = (900, 500)
R_SG = 10

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

    # Build the start point and two goal points
    # substract radius of TCP so that TCP is fully in area
    G_1 = shapely.geometry.Point(_G_1).buffer(R_SG - 2)
    G_2 = shapely.geometry.Point(_G_2).buffer(R_SG - 2)
    S = shapely.geometry.Point(_S).buffer(R_SG - 2)

    link1_line = shapely.geometry.LineString([(500, 500 + 20), (500, 500 + L_1 - 5)])
    link2_line = shapely.geometry.LineString([(500, 500 + L_1 + 5), (500, 500 + L_1 + L_2 - 5)])

    link1 = link1_line.buffer(5)
    link2 = link2_line.buffer(5)

    theta_1_range = int(360 / precision) + 1 # include upper bound
    theta_2_range = int(360 / precision) + 1

    # Build configuration space
    # FIXME: Slow as fuck, it needs to be run on multiple processors
    c_space = np.zeros((theta_1_range, theta_2_range))
    for theta_2 in range(theta_2_range):
        for theta_1 in range(theta_1_range):
            base_tcp = compute_fk(theta_1 * precision, theta_2 * precision, L_1, L_2)
            world_tcp = tf_base_to_world(base_tcp)

            manipulator_joint = compute_fk(theta_1 * precision, 0, L_1, 0)
            world_joint = tf_base_to_world(manipulator_joint)

            link1_line.coords = ((500, 500), world_joint)
            link1 = link1_line.buffer(5)

            link2_line.coords = (world_joint, world_tcp)
            link2 = link2_line.buffer(5)

            # Since we assume the TCP is a circle with radius 2
            tcp = shapely.geometry.Point(world_tcp).buffer(2)

            c_space[theta_2][theta_1] = 255

            # Check if TCP is in collision
            for c_obstacle in c_obstacles:
                if link1.intersects(c_obstacle) or link2.intersects(c_obstacle):
                    c_space[theta_2][theta_1] = 30
                if c_obstacle.intersects(tcp):
                    c_space[theta_2][theta_1] = 0
                    # print("Configuration collision: {}".format((theta_1 * precision, theta_2 * precision)))

            if G_1.intersects(tcp) or G_2.intersects(tcp):
                c_space[theta_2][theta_1] = 10
                # print("Found G")
            elif S.intersects(tcp):
                c_space[theta_2][theta_1] = 20
                # print("Found S")

    # Need to flip the array because numpy access them row first
    # are accessed row first.
    c_space = np.flip(c_space, 0)
    c_space = np.flip(c_space, 1)

    # Plot configuration space
    # TODO: Better plotting with right axes
    bounds = [0, 9, 19, 29, 254, 255]
    colormap = clr.ListedColormap(['0.7', 'green', 'red', '0.4', 'white'])
    norm = clr.BoundaryNorm(bounds, colormap.N)
    plt.imshow(c_space, cmap=colormap, norm=norm)
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
