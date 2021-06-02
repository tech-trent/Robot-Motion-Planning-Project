# Robot-Motion-Planning-Project
## Plot and Navigate a Virtual Maze

### **Overview**
This project is based off the micromouse competitions and is, in fact, a virtual version of it. Where a virtual robot mouse is placed in an unknown maze and firstly tries to map out the maze to figure out the optimal path from the corner of a maze to its center. In the 2nd run, the virtual robot mouse attempts to reach the center in the fastest time possible by using what it learned previously. 


### **Simulation Instructions**

`python tester.py <maze_file_name>  <controller_name>`

For example, you can run the random controller with maze 02:

`python tester.py test_maze_02.txt random`

Each time after the first run, the robot map the maze based on the collected data from the first run, the mapped maze is stored in the file `maze.txt`. If you want to see the path the robot mapped, please follow command line the instruction, in the mapped maze, closed blocks means the robot has not its goal:

`python showmaze.py maze.txt`

### **Controllers Used In Simulation**

- Random
- Dead-end
- Counter
- Heuristic
