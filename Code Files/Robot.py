# -*- coding:utf-8 -*-
# from __future__ import division
import numpy as np
import random
import sys
import time
import operator

# global dictionaries for robot movement and sensing
dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
            'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}
dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}
dir_heading = {'u': 'A', 'r': '>', 'd': 'V', 'l': '<',
               'up': 'A', 'right': '>', 'down': 'V', 'left': '<'}
map_dic = {'up': 3, 'right': 0, 'down': 1, 'left': 2,
           'u': 3, 'r': 0, 'd': 1, 'l': 2,}

exploration_model = str(sys.argv[2])
# 'heuristic' # 'deadend'  'random'  'heuristic' 'counter'


class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim
        self.maze_area = maze_dim ** 2
        self.goal_bounds = [maze_dim / 2 - 1, maze_dim / 2]
        self.base_rotation = [-90, 0, 90]
        self.base_movement = [-3, -2, -1, 0, 1, 2, 3]

        # more information to learn
        self.run = 0
        self.move_time = 0
        self.goal_position = [0, 0]         # remember the goal_position
        self.dead_zone = False              # check if the robot in the dead_zone
        self.remember_goal = False          # check if the robot hit_goal
        self.last_move_backward = False     # check last step robot move_backward
        self.last_movement = 0
        self.path_length = 0

        self.map_location = [[' ' for row in range(maze_dim)] for col in range(maze_dim)]

        self.map_dead_zone = [[' ' for row in range(maze_dim)] for col in range(maze_dim)]

        self.map_count = [[0 for row in range(maze_dim)] for col in range(maze_dim)]
        self.map_count[0][0] = 1

        self.map_heuristic = [[min(abs(row-maze_dim/2+1), abs(row-maze_dim/2))+min(abs(col-maze_dim/2+1),
                            abs(col-maze_dim/2)) for row in range(maze_dim)] for col in range(maze_dim)]

        self.map_maze = [[[0,0,0,0] for row in range(maze_dim)] for col in range(maze_dim)]

        self.mapped_maze = [[15 for row in range(maze_dim)] for col in range(maze_dim)]

        self.value = [[99 for row in range(self.maze_dim)] for col in range(self.maze_dim)]

        self.exploration_model = exploration_model

        # exploring model
        self.random_fuc = True if self.exploration_model in ['random', 'deadend'] else False
        self.dead_end_fuc = True if self.exploration_model in ['counter', 'deadend', 'heuristic'] else False
        self.counter_fuc = True if self.exploration_model in ['counter', 'heuristic'] else False

        self.map_count = False if self.exploration_model in ['heuristic'] else False

        self.map_count = [[0 for row in range(maze_dim)] for col in range(maze_dim)]
        self.map_count[0][0] += 1

        self.map_heuristic = [[min(abs(row-maze_dim/2+1), abs(row-maze_dim/2))+min(abs(col-maze_dim/2+1),
                        abs(col-maze_dim/2)) for row in range(maze_dim)] for col in range(maze_dim)]

    def map(self, sensors):

        for i in range(3):
            if sensors[i] > 0:
                self.map_maze[self.location[0]][self.location[1]][(i+map_dic[self.heading]) % 4] += 1
        if self.last_move_backward is False:
            self.map_maze[self.location[0]][self.location[1]][(3 + map_dic[self.heading]) % 4] += 1
        else:
            self.map_maze[self.location[0]+dir_move[self.heading][0]][
                self.location[1]+dir_move[self.heading][1]][(3 + map_dic[self.heading]) % 4] += 1
        if self.last_movement > 1:
            n = self.last_movement
            while n > 1:
                self.map_maze[self.location[0] - (n-1)*dir_move[self.heading][0]][
                    self.location[1] - (n-1)*dir_move[self.heading][1]][(3 + map_dic[self.heading]) % 4] += 1
                self.map_maze[self.location[0] - (n-1)*dir_move[self.heading][0]][
                    self.location[1] - (n-1)*dir_move[self.heading][1]][(1 + map_dic[self.heading]) % 4] += 1
                n -= 1

    def show(self, something):
        if something == 'map_count':
            show_map_count = [[0 for row in range(self.maze_dim)] for col in range(self.maze_dim)]
            for i in range(self.maze_dim):
                for j in range(self.maze_dim):
                    show_map_count[self.maze_dim - 1 - j][i] = self.map_count[i][j]
            for i in range(self.maze_dim):
                print ('show_map_count[i]')
        elif something == 'map_dead_zone':
            show_map_dead_zone = [[0 for row in range(self.maze_dim)] for col in range(self.maze_dim)]
            for i in range(self.maze_dim):
                for j in range(self.maze_dim):
                    show_map_dead_zone[self.maze_dim - 1 - j][i] = self.map_dead_zone[i][j]
            for i in range(self.maze_dim):
                print ('show_map_dead_zone[i]')
        elif something == 'map_location':
            show_map_location = [[0 for row in range(self.maze_dim)] for col in range(self.maze_dim)]
            for i in range(self.maze_dim):
                for j in range(self.maze_dim):
                    show_map_location[self.maze_dim - 1 - j][i] = self.map_location[i][j]
            for i in range(self.maze_dim):
                print ('show_map_location[i]')
        elif something == 'map_maze':
            show_map_maze = [[0 for row in range(self.maze_dim)] for col in range(self.maze_dim)]
            for i in range(self.maze_dim):
                for j in range(self.maze_dim):
                    show_map_maze[self.maze_dim - 1 - j][i] = self.map_maze[i][j]
            for i in range(self.maze_dim):
                print ('show_map_maze[i]')
        elif something == 'value':
            show_value = [[0 for row in range(self.maze_dim)] for col in range(self.maze_dim)]
            for i in range(self.maze_dim):
                for j in range(self.maze_dim):
                    show_value[self.maze_dim - 1 - j][i] = self.value[i][j]
            for i in range(self.maze_dim):
                print ('show_value[i]')
        elif something == 'map_heuristic':
            for i in range(self.maze_dim):
                print ('self.map_heuristic')
        else:
            print ('I do not have what you want')

    def coverage(self):
        uncov = 0
        for i in range(self.maze_dim):
            uncov += self.map_count[i].count(0)
        return 1-float(uncov)/self.maze_area

    def in_goal_bounds(self):
        return self.location[0] in self.goal_bounds and self.location[1] in self.goal_bounds

    def check_backward(self, sensors):
        # check if need to move_backward
        # 1.first time move to the end of the dead zone,now all three sensor value are 0
        if self.last_move_backward is False and max(sensors) == 0:
            return True
        # 2.last time it moves backward, now left and right sensor value are still 0
        if self.last_move_backward is True and sensors[0] == 0 and sensors[2] == 0:
            return True
        # 3.hit the goal, need to remember the goal position and then move backward to get out
        if self.in_goal_bounds():
            self.goal_position[0] = self.location[0]
            self.goal_position[1] = self.location[1]
            self.remember_goal = True
            return True
        return False

    def update(self, rotation, movement):
        # update local position
        if movement > 0:
            self.last_move_backward = False
            if rotation == -90:
                self.heading = dir_sensors[self.heading][0]
            elif rotation == 90:
                self.heading = dir_sensors[self.heading][2]
            else:
                pass
            self.location[0] += dir_move[self.heading][0] * movement
            self.location[1] += dir_move[self.heading][1] * movement
        else:
            self.last_move_backward = True
            self.map_dead_zone[self.location[0]][self.location[1]] = self.heading
            rev_heading = dir_reverse[self.heading]
            self.location[0] += dir_move[rev_heading][0]
            self.location[1] += dir_move[rev_heading][1]
        self.map_count[self.location[0]][self.location[1]] += 1

    def check_dead_zone(self, sensors):
        if self.in_goal_bounds():
            self.goal_position[0] = self.location[0]
            self.goal_position[1] = self.location[1]
            self.remember_goal = True
            self.map_dead_zone[self.location[0]][self.location[1]] = self.heading
            return [[0, -1]]
        possible_rotation = []
        possible_move = []
        valid_move = []
        for i in range(3):
            if sensors[i] > 0:
                possible_rotation.append(self.base_rotation[i])
                possible_move.append(min(sensors[i], 3))
        #print possible_rotation, possible_move

        for i in range(len(possible_rotation)):
            for j in range(1, possible_move[i]+1):
                rotation = possible_rotation[i]
                movement = j
                if rotation == -90:
                    check_heading = dir_sensors[self.heading][0]
                elif rotation == 90:
                    check_heading = dir_sensors[self.heading][2]
                else:
                    check_heading = dir_sensors[self.heading][1]
                check_location_x = self.location[0] + dir_move[check_heading][0] * movement
                check_location_y = self.location[1] + dir_move[check_heading][1] * movement

                if self.map_dead_zone[check_location_x][check_location_y] == ' ':
                    valid_move.append([rotation, movement])
        return valid_move

    def rectify_maze(self):
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                if y - 1 >= 0 and self.map_maze[x][y - 1][0] > 0:
                    self.map_maze[x][y][2] += 1
                if x + 1 < self.maze_dim and self.map_maze[x + 1][y][3] > 0:
                    self.map_maze[x][y][1] += 1
                if y + 1 < self.maze_dim and self.map_maze[x][y + 1][2] > 0:
                    self.map_maze[x][y][0] += 1
                if x - 1 >= 0 and self.map_maze[x - 1][y][1] > 0:
                    self.map_maze[x][y][3] += 1
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                for i in range(4):
                    self.map_maze[x][y][i] = 1 if self.map_maze[x][y][i] > 0 else 0
                self.mapped_maze[x][y] = self.map_maze[x][y][0] + 2 * self.map_maze[x][y][1] \
                                         + 4 * self.map_maze[x][y][2] + 8 * self.map_maze[x][y][3]
        f = open('maze.txt', 'w')
        f.write(str(self.maze_dim) + '\n')
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                if y == self.maze_dim -1:
                    f.write(str(self.mapped_maze[x][y]) + '\n')
                else:
                    f.write(str(self.mapped_maze[x][y]) + ',')
        f.close()

    def dynamic(self):
        change = True
        while change:
            change = False
            for x in range(self.maze_dim):
                for y in range(self.maze_dim):
                    if self.goal_position[0] == x and self.goal_position[1] == y:
                        if self.value[x][y] > 0:
                            self.value[x][y] = 0
                            change = True
                    elif sum(self.map_maze[x][y]) > 0:
                        V2 = []
                        for a in range(4):
                            if self.map_maze[x][y][a] > 0:
                                if 0 <= x - (a - 2) * (a % 2)<self.maze_dim and 0 <= y - (a - 1) * ((a + 1) % 2)<self.maze_dim:
                                    V2.append(self.value[x - (a - 2) * (a % 2)][y - (a - 1) * ((a + 1) % 2)])
                        V2 = min(V2) + 1  # if len()
                        if V2 < self.value[x][y]:
                            change = True
                            self.value[x][y] = V2

    def next_move(self, sensors):

        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.
        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.
        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        #time.sleep(1)

        if self.run == 0:
            # check if need to stop first run and return 'Reset', 'Reset'
            self.map(sensors)
            if self.move_time > 998:
                print ('self.move_time')
                print ('self.coverage()')
                self.show('map_count')
                self.rectify_maze()

            if self.remember_goal and (self.coverage() > 0.5 or self.move_time > 900):
                print ('Map coverage :{:4.2f}').format(self.coverage())
                print ('1 run moves :') + str(self.move_time)
                #print 'How many times each position has been visited :'
                #self.show('map_count')
                #print 'The dead zone map:'
                #self.show('map_dead_zone')
                self.run = 1
                self.rectify_maze()
                self.dynamic()
                self.location = [0, 0]
                self.heading = 'up'
                # self.show('value')
                self.move_time = 0

                return 'Reset', 'Reset'

            self.move_time += 1

            ##### random_fuc
            if self.random_fuc:
            # check if need to move_backward, if True, return rotation, movement = 0, -1
            # learn the map_dead_zone
                if self.check_backward(sensors):
                    #print 'move back'
                    rotation, movement = 0, -1
                    # update local position
                    self.update(rotation, movement)
                    self.last_movement = movement
                    return rotation, movement

                valid_rotation = []
                # if this time no need to move backward, check if last time it moves backward,
                # if True, rotation should be left or right，change self.last_move_backward to False
                if self.last_move_backward:
                    for i in [0, 2]:
                        if sensors[i] > 0:
                            valid_rotation.append(self.base_rotation[i])
                    self.last_move_backward = False
                else:
                    for i in range(3):
                        if sensors[i] > 0:
                            valid_rotation.append(self.base_rotation[i])
                if len(valid_rotation) == 0:
                    rotation, movement = 0, -1
                    # update local position
                    self.update(rotation, movement)
                    self.last_movement = movement
                    return rotation, movement

                rotation = random.sample(valid_rotation, 1)[0]
                if rotation == -90:
                    valid_movement = range(1, min(3, sensors[0]) + 1)
                elif rotation == 90:
                    valid_movement = range(1, min(3, sensors[2]) + 1)
                else:
                    valid_movement = range(1, min(3, sensors[1]) + 1)
                movement = random.sample(valid_movement, 1)[0]
                self.update(rotation, movement)
                self.last_movement = movement
                return rotation, movement
            # check and avoid move to the dead zone
            ##### dead_end_fuc
            if self.dead_end_fuc:
                valid_move = self.check_dead_zone(sensors)
                if len(valid_move) == 0:
                    rotation, movement = 0, -1
                    # update local position
                    self.update(rotation, movement)
                    self.last_movement = movement
                    return rotation, movement
                elif self.random_fuc:
                    rotation = random.sample(valid_move, 1)[0]
                    movement = random.sample(valid_move, 1)[1]
                    self.update(rotation, movement)
                    self.last_movement = movement
                    return rotation, movement
                elif self.counter_fuc:
                    valid_position_count = []
                    for p in valid_move:
                        heading = dir_sensors[self.heading][p[0] / 90 + 1]
                        x = self.location[0] + dir_move[heading][0] * p[1]
                        y = self.location[1] + dir_move[heading][1] * p[1]
                        valid_position_count.append([self.map_count[x][y], self.map_heuristic[x][y], p])
                    valid_position_count.sort(key = operator.itemgetter(0,1))
                    min_count = valid_position_count[0][0]
                    min_h = valid_position_count[0][1]
                    min_count_move = []
                    min_h_move = []
                    best_move = []
                    for i in range(len(valid_position_count)):
                        if valid_position_count[i][0] == min_count:
                            min_count_move.append([valid_position_count[i][1], valid_position_count[i][2]])

                    if len(min_count_move) == 1:
                        best_move = min_count_move[0][1]
                    elif self.exploration_model != 'heuristic':
                            best_move = random.sample(min_count_move, 1)[0][1]
                    else:
                        for i in range(len(min_count_move)):
                            if min_count_move[i][0] == min_h:
                                min_h_move.append(min_count_move[i][1])
                        if len(min_h_move) == 1:
                            best_move = min_h_move[0]
                        else:
                            for i in range(len(min_h_move)):
                                if min_h_move[i][0] == 0:
                                    best_move = min_h_move[i]
                        if len(best_move) == 0:
                            best_move = random.sample(min_h_move, 1)[0]
                    rotation = best_move[0]
                    movement = best_move[1]
                    self.update(rotation, movement)
                    #self.show('map_location')
                    #self.show('map_dead_zone')
                    #print valid_move
                    #print rotation, movement
                    self.last_movement = movement
                    return rotation, movement

        possible_rotation = []
        possible_move = []
        best_move = [0, 0]

        for i in range(3):
            if sensors[i] > 0:
                possible_rotation.append(self.base_rotation[i])
                possible_move.append(min(sensors[i], 3))
        min_value = self.value[self.location[0]][self.location[1]]
        for i in range(len(possible_rotation)):
            for j in range(1, possible_move[i] + 1):
                rotation = possible_rotation[i]
                movement = j
                if rotation == -90:
                    check_heading = dir_sensors[self.heading][0]
                elif rotation == 90:
                    check_heading = dir_sensors[self.heading][2]
                else:
                    check_heading = dir_sensors[self.heading][1]
                check_location_x = self.location[0] + dir_move[check_heading][0] * movement
                check_location_y = self.location[1] + dir_move[check_heading][1] * movement

                if self.value[check_location_x][check_location_y] < min_value:
                    min_value = self.value[check_location_x][check_location_y]
                    best_move = [rotation, movement]
        rotation, movement = best_move[0], best_move[1]
        self.path_length += movement
        if rotation == -90:
            self.heading = dir_sensors[self.heading][0]
        elif rotation == 90:
            self.heading = dir_sensors[self.heading][2]
        else:
            pass
        self.map_location[self.location[0]][self.location[1]] = dir_heading[self.heading]
        self.location[0] += dir_move[self.heading][0] * movement
        self.location[1] += dir_move[self.heading][1] * movement
        self.move_time += 1


        if self.in_goal_bounds():
            self.map_location[0][0] = 'S'
            self.map_location[self.location[0]][self.location[1]] = 'G'
            print ('2 run moves :') + str(self.move_time)
            print ('2 run path length :') + str(self.path_length)
            print ('2 run moves as below: (up:A, right:>, down:V, left:<)')
            print ('(start location:S, goal location:G, up:A, right:>, down:V, left:<,)')
            self.show('map_location')
        return rotation, movement


# python tester.py test_maze_03.txt random
# python tester.py test_maze_03.txt deadend
# python tester.py test_maze_03.txt counter
#python tester.py test_maze_03.txt heuristic