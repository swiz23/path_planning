#!/usr/bin/env python
import numpy as np
from scipy.misc import imread
import time
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

# To program this algorithm, the following resources were used:
# https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
# https://www.youtube.com/watch?v=U3Vw0XhkD9M
# https://www.youtube.com/watch?v=76gp2IAazV4

def main():
    FNAME = "N_map.png"
    world = imread(FNAME, mode='L')
    world = np.flipud(world)
    world = np.invert(world)
    Xmax = world.shape[0]
    Ymax = world.shape[1]
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim([-0.5, Xmax-0.5])
    ax.set_ylim([-0.5, Ymax-0.5])
    ax.set_title('RRT 0 Iterations')

    plt.imshow(world, cmap=plt.cm.Purples, interpolation='nearest', origin='lower',
              extent=[-0.5,Xmax-0.5,-0.5,Ymax-0.5])

    inc_dist = input("Enter incremental distance: ")
    max_dist = np.sqrt(Xmax**2 + Ymax**2)
    np.random.seed(1)
    plt.show(block=False)

    print("Look at plot to find an obstacle-free starting and ending position.")
    collision = True
    while collision:
        q_init_x = input("Enter x coordinate (integer) of q_init: ")
        q_init_y = input("Enter y coordinate (integer) of q_init: ")
        q_init = [q_init_x, q_init_y]
        if vacant_space(q_init, world):
            collision = False
    plt.scatter(q_init_x, q_init_y, c='green')
    plt.draw()
    collision = True
    while collision:
        q_goal_x = input("Enter x coordinate (integer) of q_goal: ")
        q_goal_y = input("Enter y coordinate (integer) of q_qoal: ")
        q_goal = [q_goal_x, q_goal_y]
        if vacant_space(q_goal, world):
            collision = False
    plt.scatter(q_goal_x, q_goal_y, c='red')
    plt.draw()

    tree = [q_init]
    verts = []
    codes = []
    cntr = 0
    keep_iterating = True
    goal_reached = False
    while keep_iterating:
        collision = True
        # check to see if it's a clear path to the goal
        if no_collision(tree[-1], q_goal, world):
            collision = False
            goal_reached = True
            q_new = q_goal
            q_near = tree[-1]
        else:
            cntr+=1
        while collision:
            # get random point in plot
            q_rand = list((int(np.random.randint(0,Xmax,1)), int(np.random.randint(0,Ymax,1))))
            # find nearest vertex in the tree to the random point
            q_near = nearest_vertex(q_rand, tree, max_dist)
            # find point along the line from q_near to q_rand that is at most inc_dist away
            q_new = new_config(q_near, q_rand, inc_dist)
            # # check to see if the new point collides with a colored pixel
            if no_collision(q_near, q_new, world):
                collision = False

        tree.append(q_new)
        verts.append(q_near)
        verts.append(q_new)
        codes.append(Path.MOVETO)
        codes.append(Path.LINETO)
        # make a plot and show RRT
        path = Path(verts[-2:], codes[-2:])
        patch = patches.PathPatch(path)
        ax.add_patch(patch)
        ax.set_title('RRT ' + str(cntr) + ' Iterations')
        plt.draw()
        fig.canvas.flush_events()
        time.sleep(0.0005)
        if goal_reached:
            keep_iterating = False

    print("Goal achieved in " + str(cntr) + " iterations.")
    q_curr = q_goal
    verts_path = []
    codes_path = []
    while q_curr != q_init:
        verts_path.append(q_curr)
        verts_path.append(verts[verts.index(q_curr) - 1])
        codes_path.append(Path.MOVETO)
        codes_path.append(Path.LINETO)
        # make a plot and show RRT
        path = Path(verts_path[-2:], codes_path[-2:])
        patch = patches.PathPatch(path, color='orange', lw=3)
        ax.add_patch(patch)
        plt.draw()
        fig.canvas.flush_events()
        time.sleep(0.0005)
        q_curr = verts_path[-1]
    print("Done")
    plt.show()

def nearest_vertex(q_rand, tree, max_dist):
    min_dist = max_dist
    q_near = None
    for v in tree:
        dist = distance(v, q_rand)
        if dist < min_dist:
            min_dist = dist
            q_near = v
    return q_near

def new_config(q_near, q_rand, inc_dist):
    dist = np.sqrt((q_rand[0] - q_near[0])**2 + (q_rand[1] - q_near[1])**2)
    if dist <= inc_dist:
        return q_rand
    else:
        v = [q_rand[0] - q_near[0], q_rand[1] - q_near[1]]
        v_mag = np.sqrt(v[0]**2 + v[1]**2)
        v_unit = [v[0]/v_mag, v[1]/v_mag]
        q_new_x = q_near[0] + v_unit[0]*inc_dist
        q_new_y = q_near[1] + v_unit[1]*inc_dist
        return [int(round(q_new_x)), int(round(q_new_y))]

def no_collision((x0, y0), (x1, y1), world):
    # check if q_new is in a colored pixel or if the point already exists in the tree
    if world[y1, x1] == 255 or (x0, y0) == (x1, y1):
        return False
    # check if the line to be drawn is vertical
    elif (x1 - x0) == 0:
        return plotLineVert(x0, y0, x1, y1, world)
    # check if the line to be drawn is horizontal
    elif (y1 - y0) == 0:
        return plotLineHorz(x0, y0, x1, y1, world)
    else:
        # check if slope is less than +/-45 degrees steep
        # (i.e. within Octants 0, 3, 4, and 7)
        if abs(y1 - y0) < abs(x1 - x0):
            # check if slope is within Octant 3 and 4.
            # if yes, reverse start and end points so
            # the slope is within Octant 0 and 7
            if x0 > x1:
                return plotLineLow(x1, y1, x0, y0, world)
            else:
                return plotLineLow(x0, y0, x1, y1, world)
        # slope must be steeper than +/- 45 degrees
        # (i.e. within Octants 1, 2, 5, and 6)
        else:
            # check if slope is within Octants 5 and 6.
            # if yes, reverse start and end points so
            # the slope is within Octants 1 and 2
            if y0 > y1:
                return plotLineHigh(x1, y1, x0, y0, world)
            else:
                return plotLineHigh(x0, y0, x1, y1, world)

# draws lines using pixels for slopes in Octant 0 and 7
def plotLineLow(x0, y0, x1, y1, world):
    dx = x1 - x0
    dy = y1 - y0
    # the next two lines allow the function to handle Octant 7
    yi = np.sign(dy)
    dy = dy * yi
    D = 2*dy - dx
    y = y0

    for x in range(x0, x1):
        if world[y, x] == 255:
            return False
        if D > 0:
            y = y + yi
            D = D - 2*dx
        D = D + 2*dy
    return True

# draws lines using pixels for slopes in Octant 1 and 2
def plotLineHigh(x0, y0, x1, y1, world):
    dx = x1 - x0
    dy = y1 - y0
    # the next two lines allow the function to handle Octant 2
    xi = np.sign(dx)
    dx = dx * xi
    D = 2*dx - dy
    x = x0

    for y in range(y0, y1):
        if world[y, x] == 255:
            return False
        if D > 0:
            x = x + xi
            D = D - 2*dy
        D = D + 2*dx
    return True

# draws vertical lines using pixels
def plotLineVert(x0, y0, x1, y1, world):
    dy = y1 - y0
    yi = np.sign(dy)
    y = y0
    while y != y1:
        if world[y, x0] == 255:
            return False
        y = y + yi
    return True

# draws horizontal lines using pixels
def plotLineHorz(x0, y0, x1, y1, world):
    dx = x1 - x0
    xi = np.sign(dx)
    x = x0
    while x != x1:
        if world[y0, x] == 255:
            return False
        x = x + xi
    return True

def vacant_space(q_pnt, world):
    if world[q_pnt[1], q_pnt[0]] == 255:
        print("That point lies within an obstacle. Please choose a different one.")
        return False
    return True

def distance(pnt1, pnt2):
    return np.sqrt((pnt2[0] - pnt1[0])**2 + (pnt2[1] - pnt1[1])**2)

if __name__=='__main__':
    main()
