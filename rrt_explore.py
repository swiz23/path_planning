#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

def main():
    map_size = input("Enter map size (map will be n x n): ")
    q_init_x = input("Enter x coordinate of q_init: ")
    q_init_y = input("Enter y coordinate of q_init: ")
    num_verts = input("Enter number of vertices: ")
    inc_dist = input("Enter incremental distance: ")

    q_init = [q_init_x, q_init_y]
    max_dist = map_size * np.sqrt(2)
    np.random.seed(1)

    tree = [q_init]
    verts = []
    codes = []

    for i in range(num_verts):
        # get random point in plot and make sure it's not already in the tree
        q_rand = list(np.random.random(2)*map_size)
        while q_rand in tree:
            q_rand = list(np.random.random(2)*map_size)
        # find nearest vertex in the tree to the random point
        q_near = nearest_vertex(q_rand, tree, max_dist)
        # find point along the line from q_near to q_rand that is inc_dist away
        q_new = new_config(q_near, q_rand, inc_dist)
        tree.append(q_new)
        verts.append(q_near)
        verts.append(q_new)
        codes.append(Path.MOVETO)
        codes.append(Path.LINETO)
    # make a plot and show RRT
    fig = plt.figure()
    path = Path(verts, codes)
    patch = patches.PathPatch(path)
    ax = fig.add_subplot(111)
    ax.add_patch(patch)
    ax.set_xlim([0, map_size])
    ax.set_ylim([0, map_size])
    ax.set_title('RRT ' + str(num_verts) + ' Iterations')
    plt.show()

def nearest_vertex(q_rand, tree, max_dist):
    min_dist = max_dist
    q_near = [0,0]
    for v in tree:
        dist = np.sqrt((q_rand[0] - v[0])**2 + (q_rand[1] - v[1])**2)
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

if __name__=='__main__':
    main()
