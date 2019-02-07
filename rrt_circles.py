#!/usr/bin/env python
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

def main():
    map_size = input("Enter map size (map will be n x n): ")
    inc_dist = input("Enter incremental distance: ")
    tolerance = input("Enter tolerance at goal: ")
    world = generate_circles(10, 8, 3, map_size)
    max_dist = map_size * np.sqrt(2)
    np.random.seed(1)

    # make a plot and show circles
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim([0, map_size])
    ax.set_ylim([0, map_size])
    ax.set_title('RRT 0 Iterations')

    # now let's plot the circles:
    fcirc = lambda x: patches.Circle((x[1],x[2]), radius=x[0], fill=True, alpha=1, fc='k', ec='k')
    circs = [fcirc(x) for x in world]
    for c in circs:
        ax.add_patch(c)
    ax.set_aspect('equal')
    plt.show(block=False)

    print("Look at plot to find an obstacle-free starting and ending position.")
    collision = True
    while collision:
        q_init_x = input("Enter x coordinate of q_init: ")
        q_init_y = input("Enter y coordinate of q_init: ")
        q_init = [q_init_x, q_init_y]
        if vacant_space(q_init, world):
            collision = False
    plt.scatter(q_init_x, q_init_y, c='green')
    plt.draw()
    collision = True
    while collision:
        q_goal_x = input("Enter x coordinate of q_goal: ")
        q_goal_y = input("Enter y coordinate of q_qoal: ")
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
    while keep_iterating:
        cntr+=1
        collision = True
        while collision:
            # get random point in plot and make sure it's not already in the tree
            q_rand = list(np.random.random(2)*map_size)
            # find nearest vertex in the tree to the random point
            q_near = nearest_vertex(q_rand, tree, max_dist)
            # find point along the line from q_near to q_rand that is inc_dist away
            q_new = new_config(q_near, q_rand, inc_dist)
            # check to see if the new point collides with a circle
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
        if abs(q_new[0] - q_goal[0]) < tolerance and abs(q_new[1] - q_goal[1]) < tolerance:
            keep_iterating = False

    print("Goal achieved at " + str(cntr) + " iterations at point " + str(q_new) + ".")
    q_curr = q_new
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
    q_near = [0,0]
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
        return [q_new_x, q_new_y]

def no_collision(q_near, q_new, world):
    for circ in world:
        u = ((circ[1] - q_near[0]) * (q_new[0] - q_near[0]) + (circ[2] - q_near[1]) * (q_new[1] - q_near[1])) / distance(q_near, q_new)**2

        x_perp = q_near[0] + u * (q_new[0] - q_near[0])
        y_perp = q_near[1] + u * (q_new[1] - q_near[1])
        dist = distance(circ[1:],[x_perp, y_perp])
        if dist <= circ[0]:
            return False
    return True

def vacant_space(q_pnt, world):
    for circ in world:
        dist = distance(circ[1:], q_pnt)
        if dist <= circ[0]:
            print("That point lies within an obstacle. Please choose a different one.")
            return False
    return True

def distance(pnt1, pnt2):
    return np.sqrt((pnt2[0] - pnt1[0])**2 + (pnt2[1] - pnt1[1])**2)

def generate_circles(num, mean, std, map_size):
    """
    This function generates /num/ random circles with a radius mean defined by
    /mean/ and a standard deviation of /std/.

    The circles are stored in a num x 3 sized array. The first column is the
    circle radii and the second two columns are the circle x and y locations.
    """
    circles = np.zeros((num,3))
    # generate circle locations using a uniform distribution:
    circles[:,1:] = np.random.uniform(mean, map_size-mean, size=(num,2))
    # generate radii using a normal distribution:
    circles[:,0] = np.random.normal(mean, std, size=(num,))
    return circles

if __name__=='__main__':
    main()
