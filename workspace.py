


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as clr

def draw_circle_obstacle(workspace, obstacle_origin, radius, color_value):
    inflated_radius = radius + 1
    for i in range(obstacle_origin[0]-inflated_radius, obstacle_origin[0] + inflated_radius):
        for j in range(obstacle_origin[1]-inflated_radius, obstacle_origin[1] + inflated_radius):
            if (obstacle_origin[0]-i)**2 + (obstacle_origin[1]-j)**2 <= inflated_radius**2:
                workspace[i, j] = color_value

    return workspace

workspace = np.full((1000, 1000), 255)
workspace = draw_circle_obstacle(workspace, (270, 620), 50, 0)
workspace = draw_circle_obstacle(workspace, (250, 200), 200, 0)

workspace = draw_circle_obstacle(workspace, (900, 500), 10, 150)
workspace = draw_circle_obstacle(workspace, (580, 150), 10, 10)
workspace = draw_circle_obstacle(workspace, (230, 470), 10, 200)


bounds = [0, 9, 19, 51, 101, 151, 254, 255]
colormap = clr.ListedColormap(['gray', 'green', 'purple', 'orange', 'red', 'yellow', 'white'])
norm = clr.BoundaryNorm(bounds, colormap.N)
fig2 = plt.figure()
plt.imshow(workspace, cmap=colormap, norm=norm)
plt.show()