import threading
from asyncio import wait_for
from enum import Enum
from queue import PriorityQueue

try:
    from ev3dev2.motor import LargeMotor, MediumMotor , OUTPUT_A, OUTPUT_C ,OUTPUT_D , SpeedPercent
    from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
    from ev3dev2.sensor.lego import TouchSensor, ColorSensor
except ImportError:
    print("Not running on EvDev3!")
import board
import time

#Magic numbers
MOVEMENT_SPEED_PERCENT = 15
ROTATION_360 = 53

ROTATIONS_TO_MOVE_ONE_CELL = 2.55*1.22
ROTATIONS_TO_ROTATE_90_DEGREES = 1.35

LEFT_MOTOR_CALIBRATION = 1.0
RIGHT_MOTOR_CALIBRATION = 1.04
CRANE_MOTOR_CALIBRATION = 1.09

"""
0 - cartolina preta
1 - verde escuro
2 - ciano
3 -roxo
4 - lilaz
5 - azul claro
6 - verde alface
7 - vermelho (papel)
8 - rosa
9- branco
"""
COLOR_VALUES = [15, 19, 26, 36, 49, 54, 79, 93, 100, 100]
BOARD_VALUE = 82

#Aux Enums
class Side(Enum):
    RIGHT = 0
    LEFT = 1
class Direction(Enum):
    FORWARD = 0
    BACKWARD = 1
class CardinalDir(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3


# Class representing the physical Lego crane
# Contains all the
class Crane:
    is_in_simulation_mode = False
    current_rotation = 0

    #If any of the motors are not connected/detected simulation mode kicks in
    try:
        motor = MediumMotor(OUTPUT_C)
        colorSensor = ColorSensor(INPUT_3)
    except:
        print("CRANE: motor & sensor not connected -- SIMULATION MODE")
        is_in_simulation_mode = True

    #Public function to fix crane orientation
    #rotations = # of rotations (1 = 360º)
    def publicAdjust(self, rotations):
        self.motor.on_for_rotations(SpeedPercent(100), rotations)

    # Transforms the values obtained from the color sensor into meaningful values
    # First 4 are the walls (0 OR 1)
    # Last 4 are the quadrants (0 TO 8 + "null")
    # outputArray = Arr[8] with ints range(101)
    def __sensorValuesToMeaningfullValues(self, output_array):
        meaningful_values = [-1 for i in range(8)]

        # Process the first 4 values which are related to walls
        for i  in range(4):
            if output_array[i] < 50:
                meaningful_values[i] = 1
            else:
                meaningful_values[i] = 0

        # Process the next 4 values, related to each quadrant
        for i in range(4):
            if (BOARD_VALUE + COLOR_VALUES[7])/2 > output_array[i + 4] > (BOARD_VALUE + COLOR_VALUES[6])/2:
                print("Reading value: NULL")
            else:
                for j in range (9):
                    if output_array[i + 4] < (COLOR_VALUES[8 - j] + COLOR_VALUES[8 - j + 1])/2:
                        meaningful_values[i+4] = 8-j
                if meaningful_values[i + 4] == -1:
                    print("WARNING - VALUE WAS STILL -1 ")
                    meaningful_values[i+4] = 9

        print(meaningful_values)
        return meaningful_values

    # Makes Crane rotate and read all necessary values from color sensor
    # Every 45º
    def readCell(self):
        if not self.is_in_simulation_mode:
            values_pass_1 = [-1 for i in range(8)]
            values_pass_2 = [-1 for i in range(8)]

            """  Array position references
            index:
            0 parede atrás
            1 parede esquerda
            2 parede frente
            3 parede direita
            4 quadrante 1
            5 quadrante 2
            6 quadrante 3
            7 quadrante 4
            """
            values_pass_1[0] = self.readSensor() # Back Wall (North)
            self.__rotate45Degrees(Side.LEFT)
            values_pass_1[6] = self.readSensor() # Quad 3 (Toaster Distance)
            self.__rotate45Degrees(Side.LEFT)
            values_pass_1[1] = self.readSensor() # Left Wall (East)
            self.__rotate45Degrees(Side.LEFT)
            values_pass_1[5] = self.readSensor() # Quad 2 (Butter Distance)
            self.__rotate45Degrees(Side.LEFT)
            values_pass_1[2] = self.readSensor() # Front Wall (South)
            #Return back
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_2[5] = self.readSensor() # 2nd pass - Q2
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_2[1] = self.readSensor() # 2nd pass - LW
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_2[6] = self.readSensor() # 2nd pass - Q3
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_2[0] = self.readSensor() # 2nd pass - BW
            #Reset -- to the right (from reset)
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_1[7]=self.readSensor() # Quad 4 (Object in Cell)
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_1[3]=self.readSensor() # Right Wall (West)
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_1[4] = self.readSensor() # Quad 1 (NOT USED (used to be mold distance))
            #+45º to get front wall the the second time
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_2[2] = self.readSensor()# 2nd pass - FW
            #Return back
            self.__rotate45Degrees(Side.LEFT)
            values_pass_2[4] = self.readSensor()# 2nd pass - Q1
            self.__rotate45Degrees(Side.LEFT)
            values_pass_2[3] = self.readSensor()# 2nd pass - RW
            self.__rotate45Degrees(Side.LEFT)
            values_pass_2[7] = self.readSensor()# 2nd pass - Q4
            self.__rotate45Degrees(Side.LEFT)
            #Reset

            # We read twice for each values and do the average to be more accurate
            values_average = [(values_pass_1[i]+values_pass_2[i])/2 for i in range(8)]
            return self.__sensorValuesToMeaningfullValues(values_average)

    # Rotate Crane 45º
    # side = Enum Side
    def __rotate45Degrees(self, side):
        sign = 1
        if side == Side.LEFT:
            sign = -1

        self.motor.on_for_rotations(SpeedPercent(100), ROTATION_360 / 8 * sign * CRANE_MOTOR_CALIBRATION)
        self.current_rotation += 45 * sign

    # Reset Crane Rotations to 0º
    def __resetRotation(self):
        while self.current_rotation != 0:
            if self.current_rotation >= 45:
                self.__rotate45Degrees(Side.LEFT)
            else:
                self.__rotate45Degrees(Side.RIGHT)

    def readSensor(self):
        return self.colorSensor.reflected_light_intensity

# Class representing the physical Robot
# Contains the methods to move the robot, calibrate the light sensor and play the game
class Robot:
    crane = Crane()
    is_in_simulation = False
    turn_counter=0

    finding_toaster = False
    toaster_stun = True
    
    path = []
    turns = 0

    # If any of the motors are not connected/detected simulation mode kicks in
    try:
        touch_sensor = TouchSensor(INPUT_2)
   
        motor_left = LargeMotor(OUTPUT_A)
        motor_right = LargeMotor(OUTPUT_D)
    except:
        print("ROBOT: touchSensor & motors not connected -- SIMULATION MODE")
        is_in_simulation = True
    
    def __init__(self, board, simulationBoard = None):
        self.board = board
        self.simulationBoard = simulationBoard
        self.current_x = 0
        self.current_y = 0

        self.hasButter = False
        self.toaster_found = False

        self.column_search_count = 0
        self.moving_right = False
        self.first_time_left_on_1_toaster = False

    # Basic SINGLE Motor movement
    def __moveMotor(self,side, rotation):
        if self.is_in_simulation:
            return
        if side == Side.LEFT:
            self.motor_left.on_for_rotations(SpeedPercent(MOVEMENT_SPEED_PERCENT),
                                             rotation * LEFT_MOTOR_CALIBRATION)
        else:
            self.motor_right.on_for_rotations(SpeedPercent(MOVEMENT_SPEED_PERCENT),
                                              rotation * RIGHT_MOTOR_CALIBRATION)

    # Robot moves one Cell Forward/Backward
    # direction = Enum Direction
    def __moveOneCell(self, direction = Direction.FORWARD):
        sign = 1
        if direction == Direction.BACKWARD:
            sign = -1

        thread_move_left_motor = threading.Thread(target=self.__moveMotor, args=( Side.LEFT, ROTATIONS_TO_MOVE_ONE_CELL * sign))
        thread_move_right_motor = threading.Thread(target=self.__moveMotor, args=(Side.RIGHT, ROTATIONS_TO_MOVE_ONE_CELL * sign))

        thread_move_left_motor.start()
        thread_move_right_motor.start()

        thread_move_left_motor.join()
        thread_move_right_motor.join()
        
    # Robot moves one cell to the side that is chosen
    # side = Enum Side
    def __moveOneCellToTheSide(self, side = Side.RIGHT):
        def __rotate90Degrees(self, side):
            sign = 1
            if side == Side.LEFT:
                sign = -1

            thread_move_left_motor = threading.Thread(target=self.__moveMotor,
                                                    args=(Side.LEFT, ROTATIONS_TO_ROTATE_90_DEGREES * sign))
            thread_move_right_motor = threading.Thread(target=self.__moveMotor,
                                                    args=(Side.RIGHT, ROTATIONS_TO_ROTATE_90_DEGREES * -sign))

            thread_move_left_motor.start()
            thread_move_right_motor.start()
            thread_move_left_motor.join()
            thread_move_right_motor.join()

        if self.is_in_simulation:
            return

        if side == Side.RIGHT:
            # Check if there is no wall to the right (due to the pathing algorith this should never be hit)
            if self.board.getCell(self.current_x, self.current_y).right_border.has_wall:
                print("ERROR - Cannot move right, there is a wall!")
                return
            
            __rotate90Degrees(self, Side.RIGHT)
            self.__moveOneCell()
            __rotate90Degrees(self, Side.LEFT)
        else:
            # Check if there is no wall to the left (due to the pathing algorith this should never be hit)
            if self.board.getCell(self.current_x, self.current_y).left_border.has_wall:
                print("ERROR - Cannot move left, there is a wall!")
                return  
            
            __rotate90Degrees(self, Side.LEFT)
            self.__moveOneCell()
            __rotate90Degrees(self, Side.RIGHT)

    # Easy Calibration of colors for the color sensor
    # Colors are read one at a time and at the end the global color array is updated
    def calibrateLightSensor(self):
        global COLOR_VALUES
        global BOARD_VALUE
        calibration_done = False

        while not calibration_done:
            print("Calibration process started:")
            for i in range(10):
                print("Color " + str(i) + ": ")
                self.__waitNewTurn()
                COLOR_VALUES[i] = self.crane.readSensor()
                time.sleep(1)
            print("Calibrate NULL (Board color):")
            self.__waitNewTurn()
            BOARD_VALUE = self.crane.readSensor()
            time.sleep(1)
            print("Values :" + str(COLOR_VALUES) + " Board value: " + str(BOARD_VALUE))
            input1 = input("CALIBRATE AGAIN?")
            if(input1 != "1"):
                break
            
    # Updates the current Cell with wall, objects inside it and distances
    def __updateCurrentCell(self, sensorReadings = None):
        # Simulation way to update the cell
        if self.is_in_simulation:
            # Get the current cell on the Robot "memory"
            currentCell = self.board.getCell(self.current_x, self.current_y)

            # Get the corresponding cell on the simulation board
            simCell = self.simulationBoard.getCell(self.current_x, self.current_y)
            if not simCell:
                print("Simulation Error: Unable to get cell from Simulation Board!")
                return

            # "READ" Cell -- Update the main board's current cell with the simulation data
            currentCell.top_border.has_wall = simCell.top_border.has_wall
            currentCell.right_border.has_wall = simCell.right_border.has_wall
            currentCell.bottom_border.has_wall = simCell.bottom_border.has_wall
            currentCell.left_border.has_wall = simCell.left_border.has_wall
            currentCell.butter_distance = simCell.butter_distance
            currentCell.toaster_distance = simCell.toaster_distance

        # Real way to update the cell
        else:
            currentCell = self.board.getCell(self.current_x, self.current_y)
            """
                index:
                0 parede atrás
                1 parede esquerda
                2 parede frente
                3 parede direita
                4 quadrante 1
                5 quadrante 2
                6 quadrante 3
                7 quadrante 4
                """

            # Checks the first 4 values from the sensorReadings which are walls
            print(sensorReadings)
            if sensorReadings[0] == 1:
                currentCell.top_border.has_wall= True
            if sensorReadings[1] == 1:
                currentCell.right_border.has_wall = True
            if sensorReadings[2] == 1:
                currentCell.bottom_border.has_wall = True
            if sensorReadings[3] == 1:
                currentCell.left_border.has_wall = True

            currentCell.butter_distance = int(sensorReadings[5])

            # Only knows the toaster distance when 1 distance away
            try:
                currentCell.toaster_distance = int(sensorReadings[6])
            except:
                currentCell.toaster_distance = None

            if(sensorReadings[7] == 1): # Quad 3 (Object in cell)
                currentCell.is_butter_here = True
            elif(sensorReadings[7] == 3):
                currentCell.toaster_distance = 0

        # Marks current cell as explored
        currentCell.has_been_explored = True

        self.board.update_possible_butter_cells(self.current_x, self.current_y) #Recalculate where Butter may exist

    # Called when a new turn starts, makes robot wait for user input before proceeding
    def __waitNewTurn(self):
        print("TURN: " + str(self.turns))
        self.turns += 1

        if self.is_in_simulation:
            print("STARTING STATE")
            self.simulationBoard.displayBoard()
            print("CURRENT STATE - ROBOT MEMORY")
            self.board.displayBoard()
            input("TouchSensor - WAITING PRESS ENTER TO CONTINUE")
            return

        print("CURRENT STATE - ROBOT MEMORY")
        self.board.displayBoard()
        self.touch_sensor.wait_for_pressed()

    # Main logic for the Robot Movement - includes:
    # A*
    # Default Moves (first 4 moves)
    # Return to start with butter
    # Get Butter Strat
    # Get Mold to Toaster Start
    def makeMove(self):

        # A* algorithm from where the robot is to where ever we need
        # it returns nothing, it updates the path variable on robot scope
        def __aStarSearch(self, target = None):
            def __getNeighbors(self, position, restricted_positions):
                def is_wall_between(self, current, neighbor):
                    cx, cy = current
                    nx, ny = neighbor
                    current_cell = self.board.getCell(cx, cy)
                    if nx < cx:  # North
                        return current_cell.top_border.has_wall
                    elif nx > cx:  # South
                        return current_cell.bottom_border.has_wall
                    elif ny < cy:  # West
                        return current_cell.left_border.has_wall
                    elif ny > cy:  # East
                        return current_cell.right_border.has_wall
                    return True

                x, y = position
                neighbors = []
                directions = [
                    (x - 1, y),  # North
                    (x + 1, y),  # South
                    (x, y - 1),  # West
                    (x, y + 1)   # East
                ]

                for nx, ny in directions:
                    cell = self.board.getCell(nx, ny)
                    if cell and not is_wall_between(self, position, (nx, ny)) and (nx, ny) not in restricted_positions and not cell.is_mold_here:
                        neighbors.append((nx, ny))
                neighbors.reverse()
                return neighbors

            def __reconstructPath(self, came_from, current):
                self.path = []
                while current in came_from:
                    self.path.append(current)
                    current = came_from[current]

            def __predictMoldPositions(self):
                current_x, current_y = None, None
                for x in range(6):
                    for y in range(6):
                        if self.board.getCell(x, y).is_mold_here:
                            current_x, current_y = x, y
                            break
                    if current_x is not None:
                        break

                if current_x is None or current_y is None:
                    return []  # No mold found

                directions = [
                    (current_x - 1, current_y),  # North
                    (current_x + 1, current_y),  # South
                    (current_x, current_y - 1),  # West
                    (current_x, current_y + 1)   # East
                ]

                valid_positions = []
                for nx, ny in directions:
                    if 0 <= nx < 6 and 0 <= ny < 6:
                        cell = self.board.getCell(nx, ny)
                        if not cell.is_robot_here and not cell.is_butter_here:
                            valid_positions.append((nx, ny))

                return valid_positions

            def __calculateHeuristics(self, x, y):
                target_x_arr = []
                target_y_arr= []

                if self.hasButter: #Heuristics if Returning to (0,0)
                    return abs(x) + abs(y)
                if self.finding_toaster and not self.moving_right and not self.toaster_found: #Heuristics if finding toaster strat and going left
                    return abs(x- self.current_x) + abs(y- self.current_y-1)
                if self.moving_right and not self.toaster_found:    #Heuristics if finding toaster strat and going right
                    return abs(x- self.current_x) + abs(y- self.current_y+1)
                if self.toaster_found: #Heuristics if luring mold to toaster
                    toaster_x = -1
                    toaster_y = -1
                    for x in range(6):
                        for y in range(6):
                            if self.board.getCell(x,y).toaster_distance == 0:
                                toaster_x = x
                                toaster_y = y
                    return  abs(x-toaster_x) + abs(y-toaster_y)
                #Heuristics for going to butter/ average of where the robots thinks the butter is
                for x in range(6):
                    for y in range(6):
                        if self.board.getCell(x,y) in self.board.possible_cells:
                            target_x_arr.append(x)
                            target_y_arr.append(y)

                target_x = sum(target_x_arr) / len(target_x_arr)
                target_y = sum(target_y_arr) / len(target_y_arr)

                return abs(x - target_x) + abs(y - target_y)
            
            start = (self.current_x, self.current_y)
            possible_cells = self.board.possible_cells
            if target is None:
                possible_cells = self.board.possible_cells
            else:
                possible_cells = [target]

            possible_targets = []

            for cell in possible_cells:
                for x in range(6):
                    for y in range(6):
                        if cell == self.board.getCell(x,y):
                            possible_targets.append((x,y))



            if not possible_targets:
                print("No possible targets. A* search cannot proceed.")
                self.path = []
                return

            predicted_mold_positions = __predictMoldPositions(self)

            frontier = PriorityQueue()    
            frontier.put((0, start))

            came_from = {}
            g_score = {start: 0}  # Cost from start to current cell
            f_score = {start: __calculateHeuristics(self, self.current_x, self.current_y)}

            while not frontier.empty():
                _, current = frontier.get()

                # Check if we've reached any of the target cells
                if current in possible_targets:
                    __reconstructPath(self, came_from, current)
                    return

                for neighbor in __getNeighbors(self, current, predicted_mold_positions):
                    tentative_g_score = g_score[current] + 1
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + __calculateHeuristics(self,*neighbor)
                        frontier.put((f_score[neighbor], neighbor))
            
            print("A* search failed to find a path to any target.")
            self.path = []

        # Return to (0,0)
        def __returnToStartWithButter(self):
            print("Robot has grabbed the butter. Returning to start position with the butter.")
            while(True): 
                if not self.board.getCell(self.current_x, self.current_y).has_been_explored:
                    if self.is_in_simulation:
                        self.__updateCurrentCell()
                    else:
                        sensorReadings = self.crane.readCell()
                        self.__updateCurrentCell(sensorReadings)

                if self.board.getCell(self.current_x, self.current_y).toaster_distance == 0:
                    if self.toaster_stun:
                        self.toaster_stun = False
                        self.board.updateRobotPosition(self.current_x, self.current_y)
                        self.board.updateMoldPosition(self.current_x,self.current_y)
                        print("TOASTER STUN")
                        return

                    self.toaster_stun = True
                __aStarSearch(self, self.board.getCell(0,0))
                print("REVERSED PATH: "+ str(self.path))
            
                chosenDirection = self.path.pop()
                
                new_x, new_y = chosenDirection
                
                if new_x < self.current_x :
                    print("Moving North")
                    self.__moveOneCell(Direction.BACKWARD)
                    self.current_x -= 1
                elif new_y < self.current_y :
                    print("Moving West")
                    self.__moveOneCellToTheSide(Side.LEFT)
                    self.current_y -= 1
                elif new_x > self.current_x :
                    print("Moving South")
                    self.__moveOneCell(Direction.FORWARD)
                    self.current_x += 1
                elif new_y > self.current_y :
                    print("Moving East")
                    self.__moveOneCellToTheSide(Side.RIGHT)
                    self.current_y += 1
                else:
                    print("ERROR - Not A Valid Direction!!")
                self.board.updateRobotPosition(self.current_x,self.current_y)
                self.board.updateButterPosition(self.current_x,self.current_y)
                self.board.updateMoldPosition(self.current_x,self.current_y)
                self.__waitNewTurn()
                if self.current_x == 0 and self.current_y == 0:
                    print("Robot has returned to the starting position with the butter.")
                    print("WIN")
                    exit()

        # Execute the optimized first 4 moves
        # UNLESS the robot hits an exit condition
        def __defaultMoves(self):
            currentCell = self.board.getCell(self.current_x, self.current_y)
            tempX = self.current_x
            tempY = self.current_y

            if self.hasButter:
                objectInCell = self.board.getCell(self.current_x, self.current_y).getObjectInCell()
                if objectInCell == 0:  # Butter detected
                    print("Butter found! Picking it up.")
                    __returnToStartWithButter(self)  # Return to the start position with butter

            if self.current_x== 0 and not currentCell.bottom_border.has_wall:
                print("Moving South")
                self.__moveOneCell(Direction.FORWARD)
                self.current_x += 1
            else:
                print("Moving East")
                self.__moveOneCellToTheSide(Side.RIGHT)
                self.current_y += 1
            currentCell = self.board.getCell(self.current_x, self.current_y)


            self.board.updateRobotPosition(self.current_x,self.current_y)
            self.board.updateMoldPosition(self.current_x,self.current_y)

            self.__waitNewTurn()
            self.__updateCurrentCell(self.crane.readCell())
            
            self.turn_counter +=1

            #Stopping Conditions
            if self.board.getCell(self.current_x, self.current_y).butter_distance > self.board.getCell(tempX, tempY).butter_distance:
                self.turn_counter = 3
                print("EXIT CONDITION: Butter distance increased.") # robot now knows there the butter is
            elif self.current_x == 0 and currentCell.bottom_border.has_wall and currentCell.right_border.has_wall:
                self.turn_counter = 3
                print("EXIT CONDITION: Robot is at (0, 0) with walls on bottom and right.") # Can't reach (1,3) with only South/East moves
            elif self.current_x == 1 and currentCell.right_border.has_wall:
                self.turn_counter = 3
                print("EXIT CONDITION: Robot is at x=1 with a wall on the right.") # Can't reach (1,3) with only South/East moves
            elif currentCell.toaster_distance == 1:
                self.turn_counter = 3
                print("EXIT CONDITION: Toaster is within 1 cell distance.") # Let the robot actually think, toaster is near...

        def __moldToToasterStrat(self):
            def __lureMoldToToaster(self):
                toaster_x = -1
                toaster_y = -1
                for x in range(6):
                    for y in range(6):
                        if self.board.getCell(x, y).toaster_distance == 0:
                            toaster_x = x
                            toaster_y = y
                #On toaster, going left makes mold fall in toaster
                if self.current_x == toaster_x and self.current_y == toaster_y and self.current_y > 0 and not self.board.getCell(self.current_x,self.current_y).left_border.has_wall:
                    print("Moving West")
                    self.__moveOneCellToTheSide(Side.LEFT)
                    self.current_y -= 1
                    self.column_search_count = 0
                    return
                #On toaster, but cant go left - go up and then back down (win because if robot is in the toaster and mold moves there the robot wins)
                elif self.current_x == toaster_x and self.current_y == toaster_y and self.current_y == 0 and self.board.getCell(self.current_x, self.current_y).top_border.has_wall == False:
                    print("Moving North")
                    self.__moveOneCell(Direction.BACKWARD)
                    self.current_x -= 1
                    return
                #On toaster but cant go left nor north, same reason
                elif self.current_x == toaster_x and self.current_y == toaster_y and self.current_y == 0 and self.board.getCell(self.current_x, self.current_y).top_border.has_wall == False:
                    print("Moving South")
                    self.__moveOneCell(Direction.FORWARD)
                    self.current_x += 1
                    return

                #not on the toaster, move to toaster
                __aStarSearch(self, self.board.getCell(toaster_x, toaster_y))

                print("REVERSE PATH (1): "+ str(self.path))
                chosenDirection = self.path.pop()

                new_x, new_y = chosenDirection

                if new_x < self.current_x:
                    print("Moving North")
                    self.__moveOneCell(Direction.BACKWARD)
                    self.current_x -= 1
                elif new_y < self.current_y:
                    print("Moving West")
                    self.__moveOneCellToTheSide(Side.LEFT)
                    self.current_y -= 1
                    self.column_search_count = 0
                elif new_x > self.current_x:
                    print("Moving South")
                    self.__moveOneCell(Direction.FORWARD)
                    self.current_x += 1
                elif new_y > self.current_y:
                    print("Moving East")
                    self.__moveOneCellToTheSide(Side.RIGHT)
                    self.current_y += 1
                else:
                    print("Not A Valid Direction!!")
                pass

            if not self.toaster_found: #check if toaster is known
                for x in range(6):
                    for y in range(6):
                        if self.board.getCell(x,y).toaster_distance == 0:
                            self.toaster_found = True

            if self.toaster_found:
                __lureMoldToToaster(self)
                return

            if self.current_y + 2 < self.board.get_mold_y(): #Safe to move right
                self.column_search_count = 0
                #se R |  |  |M| | |
                #   0   ->2  3
                #     |R|M| | | | gud
                #MOVER COLUNA PARA A DIREITA
                if not self.board.getCell(self.current_x,self.current_y).right_border.has_wall:
                    self.__moveOneCellToTheSide(Side.RIGHT)
                    self.current_y += 1
                else:   #if theres a wall to the right, use A* to move to the right
                    self.moving_right = True
                    __aStarSearch(self, self.board.getCell(self.current_x, self.current_y + 1))
                    chosenDirection = self.path.pop()

                    new_x, new_y = chosenDirection

                    if new_x < self.current_x:
                        print("Moving North")
                        self.__moveOneCell(Direction.BACKWARD)
                        self.current_x -= 1
                    elif new_y < self.current_y:
                        print("Moving West")
                        self.__moveOneCellToTheSide(Side.LEFT)
                        self.current_y -= 1
                    elif new_x > self.current_x:
                        print("Moving South")
                        self.__moveOneCell(Direction.FORWARD)
                        self.current_x += 1
                    elif new_y > self.current_y:
                        print("Moving East")
                        self.__moveOneCellToTheSide(Side.RIGHT)
                        self.current_y += 1
                    else:
                        print("Not A Valid Direction!!")
                return
            else:
                #SEARCH COLUMN
                if self.board.getCell(self.current_x,self.current_y).toaster_distance == 1 and not self.board.getCell(self.current_x,self.current_y).left_border.has_wall and self.first_time_left_on_1_toaster:
                    print("Moving West")
                    self.first_time_left_on_1_toaster = False
                    self.__moveOneCellToTheSide(Side.LEFT)
                    self.current_y -= 1
                    self.column_search_count = 0
                    return
                for x in range(6):
                    if not self.board.getCell(x, self.current_y).has_been_explored and self.column_search_count <= 6:
                        if self.current_x >= x and not self.board.getCell(self.current_x,self.current_y).top_border.has_wall:
                            print("Moving North")
                            self.__moveOneCell(Direction.BACKWARD)
                            self.current_x -= 1
                            self.column_search_count += 1
                            return
                        elif self.current_x <= x and not self.board.getCell(self.current_x,self.current_y).bottom_border.has_wall:
                            print("Move South")
                            self.__moveOneCell(Direction.FORWARD)
                            self.current_x += 1
                            self.column_search_count += 1
                            return
                        self.board.getCell(x,self.current_y).has_been_explored = True
                #not worth searching this col
                self.column_search_count = 0
                self.finding_toaster = True
                __aStarSearch(self, self.board.getCell(self.current_x, self.current_y - 1))

                if not self.path :
                    print("There is no way to lure mold to toaster LOSS!!")
                    exit()
                chosenDirection = self.path.pop()

                new_x, new_y = chosenDirection

                if new_x < self.current_x:
                    print("Moving North")
                    self.__moveOneCell(Direction.BACKWARD)
                    self.current_x -= 1
                elif new_y < self.current_y:
                    print("Moving West")
                    self.__moveOneCellToTheSide(Side.LEFT)
                    self.current_y -= 1
                    self.column_search_count = 0
                elif new_x > self.current_x:
                    print("Moving South")
                    self.__moveOneCell(Direction.FORWARD)
                    self.current_x += 1
                elif new_y > self.current_y:
                    print("Moving East")
                    self.__moveOneCellToTheSide(Side.RIGHT)
                    self.current_y += 1
                else:
                    print("Not A Valid Direction!!")

        #READ NEW CELL if not explored
        if not self.board.getCell(self.current_x, self.current_y).has_been_explored:
            if self.is_in_simulation:
                self.__updateCurrentCell()
            else:
                sensorReadings = self.crane.readCell()
                self.__updateCurrentCell(sensorReadings)

        objectInCell = self.board.getCell(self.current_x,self.current_y).getObjectInCell()
        
        if not objectInCell == -1:
            if objectInCell == 1:
                print("LOSS")
            elif objectInCell == 2:
                print("OBJECT IN CELL: TOSTADEIRA")
                self.toaster_found = True
                if self.toaster_stun:
                    self.toaster_stun = False
                    self.board.updateRobotPosition(self.current_x, self.current_y)
                    self.board.updateMoldPosition(self.current_x,self.current_y)
                    print("TOASTER STUN")
                    self.__waitNewTurn()
                    return

                self.toaster_stun = True
            elif objectInCell == 0:
                self.hasButter = True
        
        if self.turn_counter < 3:
            __defaultMoves(self)
            return

        if self.hasButter:
            objectInCell = self.board.getCell(self.current_x, self.current_y).getObjectInCell()
            if objectInCell == 0:  # Butter detected
                print("Butter found! Picking it up.")
                __returnToStartWithButter(self)  # Return to the start position with butter
        elif not self.board.mold_to_toaster:

            __aStarSearch(self)
            if not self.path:
                self.board.mold_to_toaster = True
                __moldToToasterStrat(self)
                self.board.updateRobotPosition(self.current_x, self.current_y)
                self.board.updateMoldPosition(self.current_x, self.current_y)

                self.__waitNewTurn()
                return

            print("REVERSE PATH: (2): " + str(self.path))
         
            chosenDirection = self.path.pop()
            new_x, new_y = chosenDirection

            if new_x < self.current_x :
                print("Moving North")
                self.__moveOneCell(Direction.BACKWARD)
                self.current_x -= 1
            elif new_y < self.current_y :
                print("Moving West")
                self.__moveOneCellToTheSide(Side.LEFT)
                self.current_y -= 1
            elif new_x > self.current_x :
                print("Moving South")
                self.__moveOneCell(Direction.FORWARD)
                self.current_x += 1
            elif new_y > self.current_y :
                print("Moving East")
                self.__moveOneCellToTheSide(Side.RIGHT)
                self.current_y += 1
            else:
                print("Not A Valid Direction!!")
            
            self.board.updateRobotPosition(self.current_x,self.current_y)
            self.board.updateMoldPosition(self.current_x,self.current_y)
        else:
            __moldToToasterStrat(self)
            self.board.updateRobotPosition(self.current_x, self.current_y)
            self.board.updateMoldPosition(self.current_x, self.current_y)

        self.__waitNewTurn()
