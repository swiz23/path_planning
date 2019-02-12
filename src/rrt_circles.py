#!/usr/bin/env python
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

def main():
    map_size = input("Enter map size (map will be n x n): ")
    inc_dist = input("Enter incremental distance: ")
    alg = input("Enter algorithm ('custom'/'online'): ")
    world = generate_circles(10, .08*map_size, 0.03*map_size, map_size)
    max_dist = map_size * np.sqrt(2)
    # np.random.seed(1)

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
    goal_reached = False
    while keep_iterating:
        collision = True
        # check to see if it's a clear path to the goal
        if no_collision(tree[-1], q_goal, world, alg):
            collision = False
            goal_reached = True
            q_new = q_goal
            q_near = tree[-1]
        else:
            cntr+=1
        while collision:
            # get random point in plot
            q_rand = list(np.random.random(2)*map_size)
            # find nearest vertex in the tree to the random point
            q_near = nearest_vertex(q_rand, tree, max_dist)
            # find point along the line from q_near to q_rand that is at most inc_dist away
            q_new = new_config(q_near, q_rand, inc_dist)
            # check to see if the new point collides with a circle
            if no_collision(q_near, q_new, world, alg):
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
        return [q_new_x, q_new_y]

def no_collision(q_near, q_new, world, alg):
    # This algorithm was designed using logic
    if alg == "custom":
        # point is already in the tree. also, if this is true, there will be a divide by zero error in the next step
        if q_new == q_near:
            return False
        # find slope of line created between q_near and q_new
        m = (q_new[1] - q_near[1]) / float(q_new[0] - q_near[0])
        # a line is y = mx+b so find b by solving b = y - mx
        b = q_new[1] - m*q_new[0]
        # The closest distance to a line is perpendicular to it. So, that's the negative reciprocal of the slope
        m_perp = -1/m
        # find b of this perpendicular line by plugging in the center of each circle.
        for circ in world:
            b_perp = circ[2] - m_perp*circ[1]
            # find the point where the two lines intersect => m*x + b = m_perp*x + b_perp
            # This equation reduces to x(m - m_perp) = b_perp - b
            # Finally, solve for x to get x = (b_perp - b) / (m - m_perp)
            x_perp = (b_perp - b) / (m - m_perp)
            # Find y_perp by arbitrarily plugging x_perp into the line equation between q_new and q_near
            y_perp = m*x_perp + b
            # Now check to see if the intersection occurs outside the line segment
            lower_x = min(q_near[0], q_new[0])
            upper_x = max(q_near[0], q_new[0])
            lower_y = min(q_near[1], q_new[1])
            upper_y = max(q_near[1], q_new[1])
            if (x_perp < lower_x or x_perp > upper_x) or (y_perp < lower_y or y_perp > upper_y):
                # if it does, then the closest point is q_new, so check that it doesn't overlap any circle
                dist = distance(circ[1:], q_new)
            else:
                # find distance between the circle center and the point of intersection
                dist = distance(circ[1:], [x_perp, y_perp])
            # check to see if distance is less than radius of circle
            if dist <= circ[0]:
                return False
        # point must be in the clear so return True
        return True

    # This algorithm was designed using this page: http://paulbourke.net/geometry/pointlineplane/
    if alg == "online":
        if q_near == q_new:
            return False
        for circ in world:
            u = ((circ[1] - q_near[0]) * (q_new[0] - q_near[0]) + (circ[2] - q_near[1]) * (q_new[1] - q_near[1])) / distance(q_near, q_new)**2
            if u < 0 or u > 1:
                dist = distance(circ[1:], q_new)
            else:
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
    # np.random.seed(1)
    # generate circle locations using a uniform distribution:
    circles[:,1:] = np.random.uniform(mean, map_size-mean, size=(num,2))
    # generate radii using a normal distribution:
    circles[:,0] = np.random.normal(mean, std, size=(num,))
    return circles

if __name__=='__main__':
    main()
