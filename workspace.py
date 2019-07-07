import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as clr
import matplotlib.patches as mpatches

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

workspace = np.full((1000, 1000), 255)
workspace = draw_circle(workspace, (270, 620), 50, 0)
workspace = draw_circle(workspace, (250, 200), 200, 0)

workspace = draw_circle(workspace, (900, 500), 10, 150)
workspace = draw_circle(workspace, (580, 150), 10, 10)
workspace = draw_circle(workspace, (230, 470), 10, 200)

workspace = draw_square(workspace, (500, 500), 10, 20)

#workspace_image = np.flipud(workspace)
workspace_image = workspace.T

bounds = [0, 9, 19, 51, 101, 151, 254, 255]
colormap = clr.ListedColormap(['gray', 'green', 'purple', 'orange', 'red', 'yellow', 'white'])
norm = clr.BoundaryNorm(bounds, colormap.N)
fig2 = plt.figure()

plt.imshow(workspace_image, cmap=colormap, norm=norm)
plt.xlabel("X")
plt.ylabel("Y")
plt.ylim(0, 1000)

red_patch = mpatches.Patch(color='red', label='Start Position')
green_patch = mpatches.Patch(color='green', label='Goal 1')
yellow_patch = mpatches.Patch(color='yellow', label='Goal 2')
grey_patch = mpatches.Patch(color='grey', label='Obstacle')
purple_patch = mpatches.Patch(color='purple', label='Manipulator Base')
plt.legend(handles=[red_patch, green_patch, yellow_patch, grey_patch, purple_patch])

plt.title("Workspace")

plt.show()