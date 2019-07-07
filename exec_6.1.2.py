#!/usr/bin/python3
import sys, argparse
import numpy as np
import multiprocessing as mp
import math
import shapely.geometry
import matplotlib.pyplot as plt
import matplotlib.colors as clr
import networkx as nk

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

def find_path(road_map, paths_start, paths_goal):
    """ Search for a path from the start area to the goal area
        Since we have area we take lists of possible configurations.
    """
    # Find shortest path from Start to goal positions in c-space
    for path_start in paths_start:
        for path_goal in paths_goal:
            try:
                s_to_g = nk.shortest_path(road_map, path_start, path_goal)
                return True, s_to_g
            except:
                pass
                #print("Path from: {} to {} not found. Trying next path.".format(path_start, path_goal))
    return False, None 

def draw_circle_obstacle(workspace, obstacle_origin, radius, color_value):
    inflated_radius = radius + 1
    for i in range(obstacle_origin[0]-inflated_radius, obstacle_origin[0] + inflated_radius):
        for j in range(obstacle_origin[1]-inflated_radius, obstacle_origin[1] + inflated_radius):
            if (obstacle_origin[0]-i)**2 + (obstacle_origin[1]-j)**2 <= inflated_radius**2:
                workspace[i, j] = color_value

    return workspace


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

    theta_1_range = int(360 / precision) + 1 # include upper bound
    theta_2_range = int(360 / precision) + 1

    # Build configuration space
    c_space = np.full((theta_1_range, theta_2_range), 255)
    road_map = nk.DiGraph()
    path_start = []
    path_goal_g1 = []
    path_goal_g2 = []
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
                    road_map.remove_node((theta_1, theta_2))
                else:
                    node_1 = (theta_1, theta_2)

                    node_2 = (theta_1, (theta_2 + 1) % theta_2_range)
                    road_map.add_edge(node_1, node_2)

                    node_2 = ((theta_1 + 1) % theta_1_range, (theta_2 + 1) % theta_2_range)
                    road_map.add_edge(node_1, node_2)

                    node_2 = ((theta_1 + 1) % theta_1_range, theta_2)
                    road_map.add_edge(node_1, node_2)

            if G_1.contains(tcp):
                c_space[theta_1][theta_2] = 10
                path_goal_g1.append((theta_1, theta_2))
            elif G_2.contains(tcp):
                c_space[theta_1][theta_2] = 200
                path_goal_g2.append((theta_1, theta_2))
            elif S.contains(tcp):
                c_space[theta_1][theta_2] = 150 
                path_start.append((theta_1, theta_2))


    print("Configuration space built!")

    path_goals = {_G_1 : path_goal_g1, _G_2 : path_goal_g2}
    
    for path_goal in path_goals.items():
        pathIsFound, found_path = find_path(road_map, path_start, path_goal[1])
        if pathIsFound:
            print("Path to {} found!".format(path_goal[0]))
            # Draw path
            for node in found_path:
                if path_goal[0] == _G_1:
                    c_space[node[0]][node[1]] = 100
                else:
                    c_space[node[0]][node[1]] = 50

        else:
            print("Could not find path from {} to {}".format(path_goal[0], _S))

    # Plot configuration space
    bounds = [0, 9, 19, 51, 101, 151, 254, 255]
    colormap = clr.ListedColormap(['gray', 'green', 'purple', 'orange', 'red', 'yellow', 'white'])
    norm = clr.BoundaryNorm(bounds, colormap.N)

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    c_space_image = np.flipud(c_space) # we flip because we want the y axis to go from 0 to 360 instead of 360 to 0
    plt.imshow(c_space_image, cmap=colormap, norm=norm)
    plt.xlabel("Theta 2")
    plt.ylabel("Theta 1")
    plt.xticks(np.arange(0, theta_2_range, theta_2_range//36)) # we want exactly 36 ticks for the x axis
    plt.yticks(np.arange(0, theta_1_range, theta_1_range//36)) # we want exactly 36 ticks for the y axis
    ax.set_xticklabels(np.arange(0, 360, 10), rotation=-90) # normalize the labels for x so that we see angles instead of array indexes
    ax.set_yticklabels(np.arange(360, 0, -10)) # normalize the labels for y so that we see angles instead of array indexes
    plt.title("Configuration Space")
    plt.show()



# Draw workspace:

    workspace = np.full((1000, 1000), 255)
    workspace = draw_circle_obstacle(workspace, (270, 620), 50, 0)
    workspace = draw_circle_obstacle(workspace, (250, 200), 200, 0)

    workspace = draw_circle_obstacle(workspace, (900, 500), 10, 150)
    workspace = draw_circle_obstacle(workspace, (580, 150), 10, 10)
    workspace = draw_circle_obstacle(workspace, (230, 470), 10, 200)

    fig2 = plt.figure()
    workspace_image = workspace # dummy variable in case we want to manipulate the image and live the original matrix intact
    plt.imshow(workspace_image, cmap=colormap, norm=norm)
    plt.ylim(0, 1000)
    plt.title("Workspace")
    plt.show()

# End drawing workspace


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description = "Compute and plot configuration space for 2D manipulator for exec 6.1.")
    parser.add_argument(
		  "precision",
                  type=float,
                  nargs="?",
                  default=10,
		  help = "pass precision for configuration space",
		  metavar = "P")
    args = parser.parse_args() 

    main(args)
