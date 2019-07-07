#!/usr/bin/python3
import sys, argparse
import numpy as np
import multiprocessing as mp
import math
import shapely.geometry
import matplotlib.pyplot as plt
import matplotlib.colors as clr
import matplotlib.patches as mpatches
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

def draw_circle(workspace, origin, radius, color_value):
    inflated_radius = radius + 1
    for i in range(origin[0] - inflated_radius, origin[0] + inflated_radius):
        for j in range(origin[1] - inflated_radius, origin[1] + inflated_radius):
            if (origin[0] - i)**2 + (origin[1] - j)**2 <= inflated_radius**2:
                workspace[i, j] = color_value

    return workspace

def draw_square(workspace, origin, half_length, color_value):
    inflated_half_length = half_length + 1
    for i in range(origin[0] - inflated_half_length, origin[0] + inflated_half_length):
        for j in range(origin[1] - inflated_half_length, origin[1] + inflated_half_length):
            workspace[i, j] = color_value

    return workspace

def draw_path(workspace, path, color_value):
    for angle in path:
        pos = compute_fk(angle[0], angle[1], 200, 200)
        pos = tf_base_to_world(pos)
        workspace[math.floor(pos[0])][math.floor(pos[1])] = color_value
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
    colormap = clr.ListedColormap(['gray', 'green', 'blue', 'orange', 'red', 'purple', 'white'])
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
    workspace = draw_circle(workspace, (270, 620), 50, 0)
    workspace = draw_circle(workspace, (250, 200), 200, 0)

    workspace = draw_circle(workspace, (900, 500), 10, 150)
    workspace = draw_circle(workspace, (580, 150), 10, 10)
    workspace = draw_circle(workspace, (230, 470), 10, 200)

    workspace = draw_square(workspace, (500, 500), 10, 20)

    path1 = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7), (8, 8), (9, 9), (10, 10), (11, 11),
             (12, 12), (13, 13), (14, 14), (15, 15), (16, 16), (17, 17), (18, 18), (19, 19), (20, 20), (21, 21),
             (22, 22), (23, 23), (24, 24), (25, 25), (26, 26), (27, 27), (28, 28), (29, 29), (30, 30), (31, 31),
             (32, 32), (33, 33), (34, 34), (35, 35), (36, 36), (37, 37), (38, 38), (39, 39), (40, 40), (41, 41),
             (42, 42), (43, 43), (44, 44), (45, 45), (46, 46), (47, 47), (48, 48), (49, 49), (50, 50), (51, 51),
             (52, 52), (53, 53), (54, 54), (55, 55), (56, 56), (57, 57), (58, 58), (59, 59), (60, 60), (61, 61),
             (62, 62), (63, 63), (64, 64), (65, 65), (66, 66), (67, 67), (68, 68), (69, 69), (70, 70), (71, 71),
             (72, 72), (73, 73), (74, 74), (75, 75), (76, 76), (77, 77), (78, 77), (79, 77), (80, 77), (81, 77),
             (82, 77), (83, 77), (84, 77), (85, 77), (86, 77), (87, 77), (88, 77), (89, 77), (90, 77), (91, 77),
             (92, 77), (93, 77), (94, 77), (95, 77), (96, 77), (97, 77), (98, 77), (99, 77), (100, 77), (101, 77),
             (102, 77), (103, 77), (104, 77), (105, 77), (106, 77), (107, 77), (108, 77), (109, 77), (110, 77),
             (111, 77), (112, 77), (113, 77), (114, 77), (115, 78), (116, 78), (117, 79), (117, 80), (118, 81),
             (118, 82), (119, 83), (120, 84), (121, 85), (122, 86), (123, 87), (124, 88), (125, 89), (126, 90),
             (127, 91), (128, 92), (129, 93), (130, 94), (131, 95), (132, 96), (133, 97), (134, 98), (135, 99),
             (136, 100), (137, 101), (138, 102), (139, 103), (140, 104), (141, 105), (142, 106), (143, 107), (144, 108),
             (145, 109), (146, 110), (147, 111), (148, 112), (149, 113), (150, 114), (151, 115), (152, 116), (153, 117),
             (154, 118), (155, 119), (156, 120), (157, 121), (158, 122), (159, 122), (160, 123), (161, 123), (162, 123),
             (163, 124), (164, 124), (165, 124), (166, 124), (167, 124), (168, 124), (169, 124), (170, 124), (171, 124),
             (172, 124), (173, 124), (174, 124), (175, 124), (176, 124), (177, 124), (178, 124), (179, 124), (180, 124),
             (181, 124), (182, 124), (183, 124), (184, 124), (185, 124), (186, 124), (187, 124), (188, 124), (189, 125),
             (190, 126), (191, 127), (192, 128), (193, 129), (194, 130), (195, 131), (196, 132), (197, 133), (198, 134),
             (199, 135), (200, 136), (201, 137), (202, 138), (203, 139), (204, 140), (205, 141), (206, 142), (207, 143),
             (208, 144), (209, 145), (210, 146), (211, 147), (212, 148), (213, 149), (214, 150), (215, 151), (216, 152),
             (217, 153), (218, 154), (219, 155), (220, 156), (221, 157), (222, 158), (223, 159), (224, 160), (225, 161),
             (226, 162), (227, 163), (228, 164), (229, 165), (230, 166), (231, 167), (232, 168), (233, 169), (234, 170),
             (235, 171), (236, 172), (237, 173), (238, 174), (239, 175), (240, 176), (241, 177), (242, 178), (243, 179),
             (244, 180), (245, 181), (246, 182), (247, 183), (248, 184), (249, 185), (250, 186), (251, 187), (252, 188),
             (253, 189), (254, 190), (255, 191), (256, 192), (257, 193), (258, 194), (259, 195), (260, 196), (261, 197),
             (262, 198), (263, 199), (264, 200), (265, 201), (266, 202), (267, 203), (268, 204), (269, 205), (270, 206),
             (271, 207), (272, 208), (273, 209), (274, 210), (275, 211), (276, 212), (277, 213), (278, 214), (279, 215),
             (280, 216), (281, 217), (282, 218), (283, 219), (284, 220), (285, 221), (286, 222), (287, 223), (288, 224),
             (289, 225), (290, 226), (291, 227), (292, 228), (293, 229), (294, 230), (295, 231), (296, 232), (297, 233),
             (298, 234), (299, 235), (300, 236), (301, 237), (302, 238), (303, 239), (304, 240), (305, 241), (306, 242),
             (307, 243), (307, 244), (308, 245), (308, 246), (309, 247), (309, 248), (309, 249), (309, 250), (309, 251),
             (309, 252), (309, 253), (309, 254), (309, 255), (309, 256), (309, 257), (309, 258), (309, 259), (309, 260),
             (309, 261), (309, 262), (309, 263), (309, 264), (309, 265), (309, 266), (309, 267), (309, 268), (309, 269),
             (309, 270), (309, 271), (309, 272), (309, 273), (309, 274), (309, 275), (309, 276), (309, 277), (309, 278),
             (309, 279), (309, 280), (309, 281), (309, 282), (309, 283), (309, 284), (309, 285), (309, 286), (309, 287),
             (309, 288), (309, 289), (309, 290), (309, 291), (309, 292), (309, 293), (309, 294), (309, 295), (309, 296),
             (309, 297), (309, 298), (309, 299), (309, 300), (309, 301), (309, 302), (309, 303), (309, 304), (309, 305),
             (309, 306)]
    path2 = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7), (8, 8), (9, 9), (10, 10), (11, 11),
             (12, 12), (13, 13), (14, 14), (15, 15), (16, 16), (17, 17), (18, 18), (19, 19), (20, 20), (21, 21),
             (22, 22), (23, 23), (24, 24), (25, 25), (26, 26), (27, 27), (28, 28), (29, 29), (30, 30), (31, 31),
             (32, 32), (33, 33), (34, 34), (35, 35), (36, 36), (37, 37), (38, 38), (39, 39), (40, 40), (41, 41),
             (42, 42), (43, 43), (44, 44), (45, 45), (46, 46), (47, 47), (48, 48), (49, 49), (50, 50), (51, 51),
             (52, 52), (53, 53), (54, 54), (55, 55), (56, 56), (57, 57), (58, 58), (59, 59), (60, 60), (61, 61),
             (62, 62), (63, 63), (64, 64), (65, 64), (66, 64), (67, 64), (68, 64), (69, 64), (70, 64), (71, 64),
             (72, 64), (73, 64), (74, 64), (75, 64), (76, 64), (77, 64), (78, 64), (79, 64), (80, 64), (81, 64),
             (82, 64), (83, 64), (84, 64), (85, 64), (86, 64), (87, 64), (88, 64), (89, 64), (90, 64), (91, 64),
             (92, 64), (93, 64), (94, 64), (95, 64), (96, 64), (97, 64), (98, 64), (99, 64), (100, 64), (101, 64),
             (102, 64), (103, 64), (104, 64), (105, 64), (106, 65), (107, 66), (108, 67), (109, 68), (110, 69),
             (111, 70), (112, 71), (113, 72), (114, 73), (115, 74), (116, 75), (117, 76), (118, 77), (119, 78),
             (120, 79), (121, 80), (122, 81), (123, 82), (124, 83), (125, 84), (126, 85), (127, 86), (128, 87),
             (129, 88), (130, 89), (131, 90), (132, 91), (133, 92), (134, 93), (135, 94), (136, 95), (137, 96)]

    workspace = draw_path(workspace, path1, 10)
    workspace = draw_path(workspace, path2, 200)

    # workspace_image = np.flipud(workspace)
    workspace_image = workspace.T
    fig2 = plt.figure()

    plt.imshow(workspace_image, cmap=colormap, norm=norm)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.ylim(0, 1000)

    red_patch = mpatches.Patch(color='red', label='Start Position')
    green_patch = mpatches.Patch(color='green', label='Goal 1')
    yellow_patch = mpatches.Patch(color='purple', label='Goal 2')
    grey_patch = mpatches.Patch(color='grey', label='Obstacle')
    purple_patch = mpatches.Patch(color='blue', label='Manipulator Base')
    plt.legend(handles=[red_patch, green_patch, yellow_patch, grey_patch, purple_patch])

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
