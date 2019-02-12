# path_planning

Created by Solomon Wiznitzer for the Master of Science in Robotics program at Northwestern University.

## Description

This project implements three slightly different RRT algorithms. To understand them in more detail, please look at my [Portfolio post](https://swiz23.github.io/Portfolio/rrt.html). In short, they are:
1. [rrt_explore.py](src/rrt_explore.py): The RRT expands from an initial point defined by the user until a certain number of vertices are reached.
2. [rrt_circles.py](src/rrt_circles.py): The RRT expands from an initial point defined by the user until it reaches a user defined 'goal' point while avoiding circular obstacles.
3. [rrt_logo.py](src/rrt_logo.py): The RRT expands from an initial point defined by the user until it reaches a user defined 'goal' point while avoiding colored pixels.

The rest of this README is focused on how to run the files. Note that this was made using Python version 2.7.12

### Runtime Instructions

To run any of the files, just type `python <file_name>` in the terminal. For *rrt_explore.py*, you will be prompted for the following things:
1. **map size:** this dictates the size of the world; for example, an entry of `100` will create a world that is 100x100.
2. **x/y of q_init:** the initial root of the tree where the RRT algorithm will start exploring. The coordinate can be a 'float' or an 'integer'; for example, `50` and `32.3` are valid entries.
3. **vertices:** the number of points that should be in the tree until the RRT algorithm stops exploring; for example, `200` works.
4. **incremental distance:** the maximum length of a line that can be drawn from one vertex to the next. If the randomly generated point is further than this distance, then the line will be drawn with the length specified here but in the direction of the randomly generated point. If the map size is 100x100, a value of `5` works pretty well.

For *rrt_circles.py*, you will be prompted for the following new items:
1. **algorithm type:** there are two values that can be entered - either `'custom'` or `'online'`. They both should work exactly the same. The 'custom' one finds the minimum distance between a point and a line following the logic shown [here](https://www.khanacademy.org/math/geometry-home/analytic-geometry-topic/distance-between-a-point-and-a-line/v/distance-between-a-point-and-a-line) while the 'online' one was implemented following the slightly obtuse instructions [here](http://paulbourke.net/geometry/pointlineplane/).
2. **x/y of q_goal:** once the RRT finds an obstacle-free path to this point, the algorithm stops. The user should make sure to pick a point that does not coincide with an obstacle. This applies to **q_init** as well.

Note that once the goal position is found, the path from **q_init** to **q_goal** will be highlighted in orange. Note also that there are no new prompts for *rrt_logo.py* that have not already been discussed in the above two scenarios. However, in *rrt_logo.py*, **q_init** and **q_goal** should both be integers.
