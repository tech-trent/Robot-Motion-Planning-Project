# Robot-Motion-Planning-Project
## **Plot and Navigate a Virtual Maze**

### **Overview:**
This project is based off the micromouse competitions and is, in fact, a virtual version of it. Where a virtual robot mouse is placed in an unknown maze and firstly tries to map out the maze to figure out the optimal path from the corner of a maze to its center. In the 2nd run, the virtual robot mouse attempts to reach the center in the fastest time possible by using what it learned previously. 


### **Requirements Are as Follows:**

Python 2.7

IPython Notebook

numpy


### **Display a graphical representation of a maze:**
`$ python tester.py test_maze_01.txt\`


### **Run the robot through a maze:**
`$ python tester.py test_maze_01.txt`

### **Run the code in IPython Notebook:** 
Change the code in line 28 of tester.py 
`testmaze = Maze( str(sys.argv[1]) )`
to 
`testmaze = Maze('file name for a test maze')` , where 'file name for a test maze' can be 'test_maze_01.txt', 'test_maze_02.txt' and so on.


### **Batch run the robot through the randomly generated maze:**
$ python batch_maze_runner.py
