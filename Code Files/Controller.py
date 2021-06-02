import numpy as np
import random


class Controller(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''


    def next_move(self, sensors, model):
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
        base_rotation = [-90, 0, 90]
        base_movement = [-3, -2, -1, 0, 1, 2, 3]
# model = 0 random model
        # move based on sensors
        # for example,
        # if sensors get [3,0,2], then choose to turn left or right randomly,valid_rotation = [-90,90],
        #   if  moves to right, valid_movement = [1, 2]
        #   if  moves to left, valid_movement = [1, 2 ,3]
        # if sensors get [0,0,2], it has to turn right,valid_rotation = [90],valid_movement = [1, 2]
        # if sensors get [0,0,0], which means it runs into a dead end,it has to go back,valid_rotation = [0],valid_movement = [-3,-2,-1]
        rotation, movement = 0, 0

        if model == 0:  # random exploring model
            valid_rotation =[]
            print ('sensors')
            for i in range(len(sensors)):
                if sensors[i] > 0:
                    valid_rotation.append(base_rotation[i])
            if not len(valid_rotation):
                valid_rotation = [0]
            rotation = random.sample(valid_rotation, 1)[0]
            if rotation == -90:
                valid_movement = range(1, min(3, sensors[0])+1)
            elif rotation == 90:
                valid_movement = range(1, min(3, sensors[2])+1)
            elif rotation == 0 and sensors[1] > 0:
                valid_movement = range(1, min(3, sensors[1])+1)
            else:
                valid_movement = [-1]
            movement = random.sample(valid_movement, 1)[0]
            print ('rotation, movement')
            return rotation, movement



        elif model == 1:  # random exploring model plus dead end
            valid_rotation = []
            print ('sensors')
            for i in range(len(sensors)):
                if sensors[i] > 0:
                    valid_rotation.append(base_rotation[i])
            if not len(valid_rotation):
                valid_rotation = [0]
            if reverse is True:
                if len(valid_rotation) == 1:
                    movement = 1
                    rotation = valid_rotation[0]
                    return rotation, movement
                robot_pos

            rotation = random.sample(valid_rotation, 1)[0]
            if rotation == -90:
                valid_movement = range(1, min(3, sensors[0]) + 1)
            elif rotation == 90:
                valid_movement = range(1, min(3, sensors[2]) + 1)
            elif rotation == 0 and sensors[1] > 0:
                valid_movement = range(1, min(3, sensors[1]) + 1)
            else:
                valid_movement = [-1]
                reverse = True




            movement = random.sample(valid_movement, 1)[0]
            print ('rotation, movement')
            return rotation, movement