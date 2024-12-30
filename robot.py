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
ROTATION360 = 53

ROTATIONS_TO_MOVE_ONE_CELL = 2.55*1.22
ROTATIONS_TO_ROTATE_90_DEGREES = 1.35

LEFT_MOTOR_CALIBRATION = 1.0
RIGHT_MOTOR_CALIBRATION = 1.04
CRANE_MOTOR_CALIBRATION = 1.09


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
COLOR_VALUES = [11, 15, 21, 30, 43, 50, 67, 86, 95, 95]
BOARD_VALUE = 69

class Crane():
    isInSimulationMode = False
    currentRotation = 0

    try:
        motor = MediumMotor(OUTPUT_C)
        colorSensor = ColorSensor(INPUT_3)
    except:
        print("CRANE: motor & sensor not connected -- SIMULATION MODE")
        isInSimulationMode = True

    def publicAdjust(self, rotations):
        self.motor.on_for_rotations(SpeedPercent(100), rotations)
    def __sensorValuesToMeaningfullValues(self, outputArray):
        meaningfullValues = [-1 for i in range(8)]

        for i  in range(4):
            if outputArray[i] < 50:
                meaningfullValues[i] = 1
            else:
                meaningfullValues[i] = 0

        for i in range(4):
            if (BOARD_VALUE + COLOR_VALUES[7])/2 > outputArray[i + 4] > (BOARD_VALUE + COLOR_VALUES[6])/2:
                print("NULL")
            else:
                for j in range (9):
                    if outputArray[i + 4] < (COLOR_VALUES[8 - j] + COLOR_VALUES[8 - j + 1])/2:
                        meaningfullValues[i+4] = 8-j
                if meaningfullValues[i + 4] == -1:
                    print("VALUE WAS STILL -1")
                    meaningfullValues[i+4] = 9



        print(meaningfullValues)
        return meaningfullValues
        pass

    def readCell(self):
        if self.isInSimulationMode:
            pass
        else:

            values_pass_1 = [-1 for i in range(8)]
            values_pass_2 = [-1 for i in range(8)]

            """
            index:
            0 parede atrás
            1 parede esquerda
            2parede frente
            3 parede direita
            4 quadrante 1
            5 quadrante 2
            6 quadrante 3
            7 quadrante 4
            """
            values_pass_1[0] = self.readSensor() # parede trás
            self.__rotate45Degrees(Side.LEFT)
            values_pass_1[6] = self.readSensor()  #quadrante 3
            self.__rotate45Degrees(Side.LEFT)
            values_pass_1[1] = self.readSensor() #wall esquerda
            self.__rotate45Degrees(Side.LEFT)
            values_pass_1[5] = self.readSensor() #quadrante 2
            self.__rotate45Degrees(Side.LEFT)
            values_pass_1[2] = self.readSensor() # wall frente
            #Return back
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_2[5] = self.readSensor() #quadrante 2
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_2[1] = self.readSensor()
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_2[6] = self.readSensor()
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_2[0] = self.readSensor()
            #Reset -- to the right (from reset)
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_1[7]=self.readSensor() #quadrante 4
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_1[3]=self.readSensor() #wall 4
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_1[4] = self.readSensor() #quadrante 1
            #+45º to get front wall the the second time
            self.__rotate45Degrees(Side.RIGHT)
            values_pass_2[2] = self.readSensor()
            #Return back
            self.__rotate45Degrees(Side.LEFT)
            values_pass_2[4] = self.readSensor()
            self.__rotate45Degrees(Side.LEFT)
            values_pass_2[3] = self.readSensor()
            self.__rotate45Degrees(Side.LEFT)
            values_pass_2[7] = self.readSensor()
            self.__rotate45Degrees(Side.LEFT)
            #reset

            print(values_pass_1)
            print(values_pass_2)

            values_average = [(values_pass_1[i]+values_pass_2[i])/2 for i in range(8)]

            return self.__sensorValuesToMeaningfullValues(values_average)

    def __rotate45Degrees(self, side):
        sign = 1
        if side == Side.LEFT:
            sign = -1

        self.motor.on_for_rotations(SpeedPercent(100), ROTATION360 / 8 * sign * CRANE_MOTOR_CALIBRATION)
        self.currentRotation += 45*sign
    def __resetRotation(self):
        while self.currentRotation != 0: 
            print(self.currentRotation)
            if self.currentRotation >= 45:
                self.__rotate45Degrees(Side.LEFT)
            else:
                self.__rotate45Degrees(Side.RIGHT)
    def readSensor(self):
        return self.colorSensor.reflected_light_intensity


class Robot:
    crane = Crane()
    isInSimulation = False

    turnCounter=0
    
    has_valid_path = False

    gx = 0
    prev_path = []

    path = []

    try:
        touchSensor = TouchSensor(INPUT_2)
   
        motor_left = LargeMotor(OUTPUT_A)
        motor_right = LargeMotor(OUTPUT_D)
    except:
        print("ROBOT: touchSensor and motors not connected!")
        isInSimulation = True
    previousDir = ""
    previousDist = -1
    
    def __init__(self, board, simulationBoard = None):
        self.board = board
        self.simulationBoard = simulationBoard
        self.current_x = 0
        self.current_y = 0 
        self.hasButter = False

    def moveOneCell(self, direction = Direction.FORWARD):
        sign = 1
        if direction == Direction.BACKWARD:
            sign = -1
        # Check if the robot can move forward or backward
        if direction == Direction.FORWARD:
            # Check if there is no wall in front
            #if self.board.getCell(self.current_x, self.current_y).getTopBorder().hasWall():
            #   print("Cannot move forward, there is a wall!001")
            #    return
            pass
        else:  # Direction.BACKWARD
            # Check if there is no wall behind
            #if self.board.getCell(self.current_x, self.current_y).getBottomBorder().hasWall():
            #    print("Cannot move backward, there is a wall!002")
            #    return 
            pass

        thread_move_left_motor = threading.Thread(target=self.__moveMotor, args=(Side.LEFT, ROTATIONS_TO_MOVE_ONE_CELL * sign))
        thread_move_right_motor = threading.Thread(target=self.__moveMotor, args=(Side.RIGHT, ROTATIONS_TO_MOVE_ONE_CELL * sign))

        thread_move_left_motor.start()
        thread_move_right_motor.start()

        thread_move_left_motor.join()
        thread_move_right_motor.join()

        # Update current position
        #if direction == Direction.FORWARD:
        #    self.current_y += 1
        #else:
        #    self.current_y -= 1
            
        #Talvez mexer menos de uma cell e dar inch forward testando com o sensor de vez em quando a ver se já tá no sitio certo???
    def moveOneCellToTheSide(self, side = Side.RIGHT):
        if self.isInSimulation:
            return

        if side == Side.RIGHT:
            # Check if there is no wall to the right
            if self.board.getCell(self.current_x, self.current_y).right_border.has_wall:
                print("Cannot move right, there is a wall!")
                return
            
            self.__rotate90Degrees(Side.RIGHT)
            self.moveOneCell()
            self.__rotate90Degrees(Side.LEFT)
            #self.current_x += 1
        else:
            # Check if there is no wall to the left
            if self.board.getCell(self.current_x, self.current_y).left_border.has_wall:
                print("Cannot move left, there is a wall!")
                return  
            
            self.__rotate90Degrees(Side.LEFT)
            self.moveOneCell()
            self.__rotate90Degrees(Side.RIGHT)
            #self.current_x -= 1

    def waitNewTurn(self):
        if self.isInSimulation:
            input("TouchSensor - WAITING PRESS ENTER TO CONTINUE")
            return
        self.touchSensor.wait_for_pressed()

    def __moveMotor(self, side, rotation):
        if self.isInSimulation:
            return
        if side == Side.LEFT:
            self.motor_left.on_for_rotations(SpeedPercent(MOVEMENT_SPEED_PERCENT), rotation * LEFT_MOTOR_CALIBRATION)
        else:
            self.motor_right.on_for_rotations(SpeedPercent(MOVEMENT_SPEED_PERCENT), rotation * RIGHT_MOTOR_CALIBRATION)
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
        
    def checkEdgesAndCorners(self):
        cell = self.board.getCell(self.current_x, self.current_y)
        
        # Check each border
        top_wall = cell.getTopBorder().has_wall
        bottom_wall = cell.getBottomBorder().has_wall
        left_wall = cell.getLeftBorder().has_wall
        right_wall = cell.getRightBorder().has_wall

        # Determine edge or corner status
        if top_wall and left_wall and not right_wall and not bottom_wall:
            print("Robot is in the top-left corner.")
        elif top_wall and right_wall and not left_wall and not bottom_wall:
            print("Robot is in the top-right corner.")
        elif bottom_wall and left_wall and not top_wall and not right_wall:
            print("Robot is in the bottom-left corner.")
        elif bottom_wall and right_wall and not top_wall and not left_wall:
            print("Robot is in the bottom-right corner.")
        elif top_wall and not bottom_wall:
            print("Robot is on the top edge.")
        elif bottom_wall and not top_wall:
            print("Robot is on the bottom edge.")
        elif left_wall and not right_wall:
            print("Robot is on the left edge.")
        elif right_wall and not left_wall:
            print("Robot is on the right edge.")
        else:
            print("Robot is not near any edge or corner.")
        
        # Return whether the robot can move forward (assuming it's facing top to bottom)
        return not top_wall

    def __chooseDirectionToMove(self):

        validDirectionsToMove = self.__getDirectionsThatAreValid()
        #print("valid directions: " + str(validDirectionsToMove))

        directionsWhereButterIsCloser = []
        directionsWhereButterIsFurther = []
        directionsWhereItsMissingInfo = []

        for direction in validDirectionsToMove:
            if self.__isButterCloserIn(direction) == 1: #butter is closer
                directionsWhereButterIsCloser.append(direction)
            elif self.__isButterCloserIn(direction) == 0: #butter is not closer
                directionsWhereButterIsFurther.append(direction)
            else: #Not enough info
                directionsWhereItsMissingInfo.append(direction)


        print("butter closer: " + str(directionsWhereButterIsCloser))
        print("butter is further: " + str(directionsWhereButterIsFurther))
        print("others: " + str(directionsWhereItsMissingInfo))
        if len(directionsWhereButterIsCloser) > 0:
            return directionsWhereButterIsCloser[0]
        elif len(directionsWhereItsMissingInfo) > 0:
            return directionsWhereItsMissingInfo[0]
        return directionsWhereButterIsFurther[0]


    

    def __isButterCloserIn(self, cardinalDirection):
        currentDistance = self.board.getCell(self.current_x, self.current_y).butter_distance

        if cardinalDirection == CardinalDir.NORTH:
            oppositeCell = self.__getCellTo(CardinalDir.SOUTH)
            nextCell = self.__getCellTo(CardinalDir.NORTH)
        elif cardinalDirection == CardinalDir.EAST:
            oppositeCell = self.__getCellTo(CardinalDir.WEST)
            nextCell = self.__getCellTo(CardinalDir.EAST)
        elif cardinalDirection == CardinalDir.SOUTH:
            oppositeCell = self.__getCellTo(CardinalDir.NORTH)
            nextCell = self.__getCellTo(CardinalDir.SOUTH)
        else :
            oppositeCell = self.__getCellTo(CardinalDir.EAST)
            nextCell = self.__getCellTo(CardinalDir.WEST)

        #print(cardinalDirection)
        #print(oppositeCell)
        #print(nextCell)
        if not nextCell.butter_distance is None and nextCell.butter_distance < currentDistance:
            return 1
        if (oppositeCell is None or oppositeCell.butter_distance is None) and (nextCell is None or nextCell.butter_distance is None):
            return -1 #Doesn't know/not valid
        elif (oppositeCell is None or (not oppositeCell.butter_distance is None and oppositeCell.butter_distance < currentDistance)) or (not nextCell.butter_distance is None and nextCell.butter_distance > currentDistance):
            return 0 #nope
        elif not nextCell.butter_distance is None and nextCell.butter_distance < currentDistance:
            return 1
        elif not oppositeCell is None and oppositeCell.butter_distance > self.board.getCell(self.current_x,self.current_y).butter_distance:
            #print("Butter is closer in: " + str(cardinalDirection))
            #print("Current distance: " + str(self.board.getCell(self.current_x, self.current_y).butter_distance))
            return 1 #yes
        else :
            return 0 #nope

    def __getCellTo(self, cardinalDirection):
        if cardinalDirection == CardinalDir.NORTH:
            return self.board.getCell(self.current_x - 1, self.current_y)
        elif cardinalDirection == CardinalDir.EAST:
            return self.board.getCell(self.current_x , self.current_y -1)
        elif cardinalDirection == CardinalDir.SOUTH:
            return self.board.getCell(self.current_x + 1, self.current_y)
        elif cardinalDirection == CardinalDir.WEST:
            return self.board.getCell(self.current_x, self.current_y+1)


    def __updateCurrentCell(self, sensorReadings = None):
        if self.isInSimulation:
            # Get the current cell on the main board
            currentCell = self.board.getCell(self.current_x, self.current_y)
            
            # Get the corresponding cell on the simulation board
            simCell = self.simulationBoard.getCell(self.current_x, self.current_y)
            if not simCell:
                print("Simulation Error: Cell ({self.current_x}, {self.current_y}) is out of bounds on the simulation board.")
                return
            
            # Update the main board's current cell with the simulation data
            currentCell.top_border.has_wall = simCell.top_border.has_wall
            currentCell.right_border.has_wall = simCell.right_border.has_wall
            currentCell.bottom_border.has_wall = simCell.bottom_border.has_wall
            currentCell.left_border.has_wall = simCell.left_border.has_wall
            currentCell.butter_distance = simCell.butter_distance
            currentCell.toaster_distance = simCell.toaster_distance

            print("Simulation Data Updated")

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

            try:
                currentCell.toaster_distance = int(sensorReadings[6])
            except:
                currentCell.toaster_distance = None
            
            if(sensorReadings[7] == 1):
                currentCell.is_butter_here = True
            
            elif(sensorReadings[7] == 3):
                currentCell.toaster_distance = 0
        
        currentCell.has_been_explored = True

        self.board.update_possible_butter_cells(self.current_x, self.current_y)
    
    def returnToStartWithButter(self):
        print("Robot has grabbed the butter. Returning to start position with the butter.")

        self.a_star_search(self.board.getCell(0,0))

        print(self.path)
        while(True):
            chosenDirection = self.path.pop()
            
            new_x, new_y = chosenDirection

            
            if new_x < self.current_x :
                print("Moving North")
                self.moveOneCell(Direction.BACKWARD)
                self.current_x -= 1
            elif new_y < self.current_y :
                print("Moving West")
                self.moveOneCellToTheSide(Side.LEFT)
                self.current_y -= 1
            elif new_x > self.current_x :
                print("Moving South")
                self.moveOneCell(Direction.FORWARD)
                self.current_x += 1
            elif new_y > self.current_y :
                print("Moving East")
                self.moveOneCellToTheSide(Side.RIGHT)
                self.current_y += 1
            else:
                print("Not A Valid Direction!!")
            

            #ESPERAR POR AJUSTE MANUAL
            print("ROBOT POSITION: ({self.current_x}, {self.current_y})")
            

            self.board.updateRobotPosition(self.current_x,self.current_y)
            self.board.updateMoldPosition(self.current_x,self.current_y)
            if self.current_x == 0 and self.current_y == 0:
                print("WIN MOFO")
                exit()
        
        

        print("\n\n")
        print("SIMULATION BOARD")
        self.simulationBoard.displayBoard()
        print("\n\n")
        print("robot board")
        self.board.displayBoard()
        
        self.waitNewTurn()




        def find_path_to_start():
            start = (self.current_x, self.current_y)
            goal = (0, 0)
            queue = [(start, [])]
            visited = set()
            
            while queue:
                (x, y), path = queue.pop(0)
                if (x, y) == goal:
                    return path
                
                if (x, y) in visited:
                    continue
                visited.add((x, y))
                
                for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Left, Right, Up, Down
                    next_x, next_y = x + dx, y + dy
                    if 0 <= next_x < 6 and 0 <= next_y < 6 and self.is_move_safe(next_x, next_y):
                        queue.append(((next_x, next_y), path + [(next_x, next_y)]))
            
            return None  # No path found

        path_to_start = find_path_to_start()
        
        if not path_to_start:
            print("No safe path to start position found. Game over.")
            return

        for next_x, next_y in path_to_start:
            if next_x < self.current_x:
                print("Moving North")
                self.moveOneCell(Direction.BACKWARD)
            elif next_y < self.current_y:
                print("Moving West")
                self.moveOneCellToTheSide(Side.LEFT)
            elif next_x > self.current_x:
                print("Moving South")
                self.moveOneCell(Direction.FORWARD)
            elif next_y > self.current_y:
                print("Moving East")
                self.moveOneCellToTheSide(Side.RIGHT)
            
            self.current_x, self.current_y = next_x, next_y
            
            # Update board state
            self.board.updateRobotPosition(self.current_x, self.current_y)
            self.board.updateMoldPosition(self.current_x, self.current_y)
            self.board.updateButterPosition(self.current_x, self.current_y)
            self.board.displayBoard()
            
            # Wait for next turn
            self.waitNewTurn()
        
        print("Robot has returned to the starting position with the butter.")
        exit()

    def is_move_safe(self, x, y):
        cell = self.board.getCell(x, y)
        
        if cell.is_mold_here:
            return False
        
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            adjacent_cell = self.board.getCell(x + dx, y + dy)
            if adjacent_cell and adjacent_cell.is_mold_here:
                return False
        
        if cell.toaster_distance == 0:
            return False
        
        return True


    
    def __getDirectionsThatAreValid(self):
        validDirections = []

        if not self.__getCellTo(CardinalDir.NORTH) is None and not self.board.getCell(self.current_x,self.current_y).top_border.has_wall:
            validDirections.append(CardinalDir.NORTH)
            #print("North is Valid")
        if not self.__getCellTo(CardinalDir.EAST) is None and not self.board.getCell(self.current_x,self.current_y).left_border.has_wall:
            validDirections.append(CardinalDir.WEST)
            #print("EAST is valid")
        if not self.__getCellTo(CardinalDir.SOUTH) is None and not self.board.getCell(self.current_x,self.current_y).bottom_border.has_wall:
            validDirections.append(CardinalDir.SOUTH)
            #print("south is Valid")
        if not self.__getCellTo(CardinalDir.WEST) is None and not self.board.getCell(self.current_x,self.current_y).right_border.has_wall:
            validDirections.append(CardinalDir.EAST)
            #print("West is valid")
        return validDirections
    toaster_stun = True
    def makeMove(self):
        print("TOASTER DISTANCE: "+ str(self.board.getCell(self.current_x,self.current_y).toaster_distance))
        
        #ler nova celula
        if not self.board.getCell(self.current_x, self.current_y).has_been_explored:
            if self.isInSimulation:
                print("READ FROM SIMULATION BOARD")
                self.__updateCurrentCell()
            else:
                sensorReadings = self.crane.readCell()
                self.__updateCurrentCell(sensorReadings)

        objectInCell = self.board.getCell(self.current_x,self.current_y).getObjectInCell()
        if not objectInCell == -1:
            print("current object in cell ", objectInCell)
            if objectInCell == 1:
                print("LOSS")
            elif objectInCell == 2:
                print("TOSTADEIRA")
                if self.toaster_stun:
                    self.toaster_stun = False
                    self.board.updateMoldPosition(self.current_x,self.current_y)
                    print("STUN FUCKING SHIT M8")
                    
                    return

                self.toaster_stun = True
            elif objectInCell == 0:
                self.hasButter = True
                print("MANTEIGA")
        
        if self.turnCounter < 3:
            self.defaultMoves()
            return

        print("HAS BUTTER: "+ str( self.hasButter))
        if self.hasButter:
            objectInCell = self.board.getCell(self.current_x, self.current_y).getObjectInCell()
            if objectInCell == 0:  # Butter detected
                print("Butter found! Picking it up.")
                self.returnToStartWithButter()  # Return to the start position with butter
        else:

            self.a_star_search()

            print(self.path)
            chosenDirection = self.path.pop()
            
            new_x, new_y = chosenDirection

            
            if new_x < self.current_x :
                print("Moving North")
                self.moveOneCell(Direction.BACKWARD)
                self.current_x -= 1
            elif new_y < self.current_y :
                print("Moving West")
                self.moveOneCellToTheSide(Side.LEFT)
                self.current_y -= 1
            elif new_x > self.current_x :
                print("Moving South")
                self.moveOneCell(Direction.FORWARD)
                self.current_x += 1
            elif new_y > self.current_y :
                print("Moving East")
                self.moveOneCellToTheSide(Side.RIGHT)
                self.current_y += 1
            else:
                print("Not A Valid Direction!!")
            

            #ESPERAR POR AJUSTE MANUAL
            print("ROBOT POSITION: ({self.current_x}, {self.current_y})")
            

            self.board.updateRobotPosition(self.current_x,self.current_y)
            self.board.updateMoldPosition(self.current_x,self.current_y)
            print("FINISHED MOVE FROM A STAR")

        print("\n\n")
        print("SIMULATION BOARD")
        self.simulationBoard.displayBoard()
        print("\n\n")
        print("robot board")
        self.board.displayBoard()
        
        self.waitNewTurn()


    def cell_diferent_from_expected(self):
        #TODO verificar se a celula que está corresponde com a que foi calculado na heuristica
        pass

    def calculate_heuristics(self, x, y):
        target_x_arr = []
        target_y_arr= []

        for x in range(6):
            for y in range(6):
                if self.board.getCell(x,y) in self.board.possible_cells:
                    target_x_arr.append(x)
                    target_y_arr.append(y)

        target_x = sum(target_x_arr) / len(target_x_arr)
        target_y = sum(target_y_arr) / len(target_y_arr)
        

        return abs(x - target_x) + abs(y - target_y)
    
    def a_star_search(self, target = None):
        start = (self.current_x, self.current_y)
        if target == None:
            possible_cells = self.board.possible_cells
        else:
            possible_cells = [self.board.getCell(0,0)]

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

        predicted_mold_positions = self.predict_mold_positions()

        frontier = PriorityQueue()    
        frontier.put((0, start))

        came_from = {}
        g_score = {start: 0}  # Cost from start to current cell
        f_score = {start: self.calculate_heuristics(self.current_x, self.current_y)}

        while not frontier.empty():
            _, current = frontier.get()
            

            # Check if we've reached any of the target cells
            if current in possible_targets:
                print("Target reached at {current}. Reconstructing path.")
                self.reconstruct_path(came_from, current)
                return
            
            
            for neighbor in self.get_neighbors(current, predicted_mold_positions):
                tentative_g_score = g_score[current] + 1  # Assuming uniform cost for moves
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.calculate_heuristics(*neighbor)
                    frontier.put((f_score[neighbor], neighbor))
        
        print("A* search failed to find a path to any target.")


    def predict_mold_positions(self):
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

    def reconstruct_path(self, came_from, current):
        self.path = []
        while current in came_from:
            self.path.append(current)
            current = came_from[current]
        #self.path.reverse()
        #self.execute_path(path)

    def get_neighbors(self, position, restricted_positions):
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
            if cell and not self.is_wall_between(position, (nx, ny)) and (nx, ny) not in restricted_positions:
                neighbors.append((nx, ny))
        neighbors.reverse()
        return neighbors


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

    def execute_path(self, path):
        print("path: ")
        print(path)

        for step in path:
            sx, sy = step
            if sx < self.current_x:
                self.moveOneCell(Direction.BACKWARD)
            elif sx > self.current_x:
                self.moveOneCell(Direction.FORWARD)
            elif sy < self.current_y:
                self.moveOneCellToTheSide(Side.RIGHT)
            elif sy > self.current_y:
                self.moveOneCellToTheSide(Side.LEFT)
            self.current_x, self.current_y = sx, sy

    def defaultMoves(self):
        currentCell = self.board.getCell(self.current_x, self.current_y)
        tempX = self.current_x
        tempY = self.current_y

        if self.hasButter:
            objectInCell = self.board.getCell(self.current_x, self.current_y).getObjectInCell()
            if objectInCell == 0:  # Butter detected
                print("Butter found! Picking it up.")
                self.returnToStartWithButter()  # Return to the start position with butter

        if self.current_x== 0 and not currentCell.bottom_border.has_wall:
            print("Moving South")
            self.moveOneCell(Direction.FORWARD)
            self.current_x += 1
        else:
            print("Moving East")
            self.moveOneCellToTheSide(Side.LEFT)
            self.current_y += 1
        currentCell = self.board.getCell(self.current_x, self.current_y)

        self.waitNewTurn()

        self.__updateCurrentCell(self.crane.readCell())
        self.board.updateRobotPosition(self.current_x,self.current_y)
        self.board.updateMoldPosition(self.current_x,self.current_y)
        
        self.gx += 1
        self.prev_path.append(self.board.getCell(self.current_x,self.current_y))
        self.turnCounter +=1

        #condições de paragem
        if self.board.getCell(self.current_x, self.current_y).butter_distance > self.board.getCell(tempX, tempY).butter_distance:
            self.turnCounter = 3
            print("EXIT CONDITION: Butter distance increased.")
        elif self.current_x == 0 and currentCell.bottom_border.has_wall and currentCell.right_border.has_wall:
            self.turnCounter = 3
            print("EXIT CONDITION: Robot is at (0, 0) with walls on bottom and right.")
        elif self.current_x == 1 and currentCell.right_border.has_wall:
            self.turnCounter = 3
            print("EXIT CONDITION: Robot is at x=1 with a wall on the right.")
        elif currentCell.toaster_distance == 1:
            self.turnCounter = 3
            print("EXIT CONDITION: Toaster is within 1 cell distance.")

        print("FINISHED MOVE FROM DEFAULT MOVES")


    def calibrate_light_sensor(self):
        global COLOR_VALUES
        global BOARD_VALUE
        calibration_done = False

        while not calibration_done:
            print("Calibration process started:")
            for i in range(10):
                print("Color " + str(i) + ": ")
                self.waitNewTurn()
                COLOR_VALUES[i] = self.crane.readSensor()
                time.sleep(1)
            print("Calibrate NULL (Board color):")
            self.waitNewTurn()
            BOARD_VALUE = self.crane.readSensor()
            time.sleep(1)
            print("VALUES:")
            print(COLOR_VALUES)
            print(BOARD_VALUE)
            input1 = input("CALIBRATE AGAIN?")
            if(input1 != "1"):
                break

    #funçao para ver o desempenho do robot e as cells ja exploradas
    def Cell_explored (self):
        explored_cells=sum(
            1 for row in self.board for cell in row if cell.has_been_explored )
        total_cells =len (self.board)*len(self.board[0])
        print("Células exploradas: {explored_cells}/{total_cells}")
        print("Porcentagem explorada: {explored_cells / total_cells * 100:.2f}%")#muito opcional


    