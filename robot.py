import threading
from asyncio import wait_for
from enum import Enum

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

ROTATIONS_TO_MOVE_ONE_CELL = 2.55
ROTATIONS_TO_ROTATE_90_DEGREES = 1.15

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
COLOR_VALUES = [4,11,19,25,37,47,52,70,75,81]
BOARD_VALUE = 64

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
            north_wall = int(input("Wall to the North? "))
            east_wall = int(input("Wall to the East? "))
            south_wall = int(input("Wall to the South? "))
            west_wall = int(input("Wall to the West? "))
            butter_distance = input("Butter distance? ")
            toaster_distance = input("Toaster distance? ")
            return [north_wall,east_wall,south_wall,west_wall,-1,butter_distance,toaster_distance,-1]

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

    try:
        touchSensor = TouchSensor(INPUT_2)
   
        motor_left = LargeMotor(OUTPUT_A)
        motor_right = LargeMotor(OUTPUT_D)
    except:
        print("ROBOT: touchSensor and motors not connected!")
        isInSimulation = True
    previousDir = ""
    previousDist = -1
    
    def __init__(self, board):
        self.board = board
        self.current_x = 0
        self.current_y = 0 

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
    def moveOneCellToTheSide(self, side):
        if self.isInSimulation:
            return

        if side == Side.RIGHT:
            # Check if there is no wall to the right
            if self.board.getCell(self.current_x, self.current_y).getRightBorder().has_wall:
                print("Cannot move right, there is a wall!")
                return
            
            self.__rotate90Degrees(Side.RIGHT)
            self.moveOneCell()
            self.__rotate90Degrees(Side.LEFT)
            #self.current_x += 1
        else:
            # Check if there is no wall to the left
            if self.board.getCell(self.current_x, self.current_y).getLeftBorder().has_wall:
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


    def __getDirectionsThatAreValid(self):
        validDirections = []

        if not self.__getCellTo(CardinalDir.NORTH) is None and not self.board.getCell(self.current_x,self.current_y).top_border.has_wall:
            validDirections.append(CardinalDir.NORTH)
            #print("North is Valid")
        if not self.__getCellTo(CardinalDir.EAST) is None and not self.board.getCell(self.current_x,self.current_y).right_border.has_wall:
            validDirections.append(CardinalDir.EAST)
            #print("EAST is valid")
        if not self.__getCellTo(CardinalDir.SOUTH) is None and not self.board.getCell(self.current_x,self.current_y).bottom_border.has_wall:
            validDirections.append(CardinalDir.SOUTH)
            #print("south is Valid")
        if not self.__getCellTo(CardinalDir.WEST) is None and not self.board.getCell(self.current_x,self.current_y).left_border.has_wall:
            validDirections.append(CardinalDir.WEST)
            #print("West is valid")
        return validDirections

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
            #print("North " + str(self.current_x - 1) + " " + str(self.current_y))
            #print( self.board.getCell(self.current_x - 1, self.current_y))
            return self.board.getCell(self.current_x - 1, self.current_y)
        elif cardinalDirection == CardinalDir.EAST:
            #print("East " + str(self.current_x) + " "+ str(self.current_y -1))
            #print( self.board.getCell(self.current_x , self.current_y -1))
            return self.board.getCell(self.current_x , self.current_y -1)
        elif cardinalDirection == CardinalDir.SOUTH:
            #print("South " + str(self.current_x +1) + " " + str(self.current_y ))
            #print(self.board.getCell(self.current_x + 1, self.current_y))
            return self.board.getCell(self.current_x + 1, self.current_y)
        elif cardinalDirection == CardinalDir.WEST:
            #print("West " + str(self.current_x) + " " + str(self.current_y+1))
            #print(self.board.getCell(self.current_x, self.current_y+1))
            return self.board.getCell(self.current_x, self.current_y+1)







    def __updateCurrentCell(self, sensorReadings):
        currentCell = self.board.getCell(self.current_x, self.current_y)
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
        print(sensorReadings)
        currentCell.has_been_explored = True

    def makeMove(self):
        chosenDirection = self.__chooseDirectionToMove()

        #mover nessa direção

        if chosenDirection == CardinalDir.NORTH:
            print("Moving North")
            self.moveOneCell(Direction.BACKWARD)
            self.current_x -= 1
        elif chosenDirection == CardinalDir.EAST:
            print("Moving West")
            self.moveOneCellToTheSide(Side.RIGHT)
            self.current_y -= 1
        elif chosenDirection == CardinalDir.SOUTH:
            print("Moving South")
            self.moveOneCell(Direction.FORWARD)
            self.current_x += 1
        elif chosenDirection == CardinalDir.WEST:
            print("Moving East")
            self.moveOneCellToTheSide(Side.LEFT)
            self.current_y += 1
        else:
            print("Not A Valid Direction!!")

        #ESPERAR POR AJUSTE MANUAL
        print("(" + str(self.current_x) + "," + str(self.current_y) + ")")
        self.waitNewTurn()

        #ler nova celula
        if not self.board.getCell(self.current_x, self.current_y).has_been_explored:
            sensorReadings = self.crane.readCell()
            self.__updateCurrentCell(sensorReadings)

        """sensorReadings=[]
        sensorReadings.append(0)
        sensorReadings.append(0)
        sensorReadings.append(0)
        sensorReadings.append(0)
        sensorReadings.append(0)
        sensorReadings.append(input("PM"))
        sensorReadings.append(0)
        sensorReadings.append(-1)"""

        objectInCell = self.board.getCell(self.current_x,self.current_y).getObjectInCell()
        if not objectInCell == -1:
            print("current object in cell ", objectInCell)
            if objectInCell == 1:
                print("WIN")
            elif objectInCell == 2:
                print("TOSTADEIRA")
            elif objectInCell == 0:
                print("LOSS")

        self.waitNewTurn()

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
        print(f"Células exploradas: {explored_cells}/{total_cells}")
        print(f"Porcentagem explorada: {explored_cells / total_cells * 100:.2f}%")#muito opcional
