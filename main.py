#!/usr/bin/env python3

import board
import robot

print("SIMULATION BOARD")
simulationBoard = board.Board()
print("EMPTY BOARD")
board = board.Board()



robot = robot.Robot(board, simulationBoard)

#robot.calibrate_light_sensor()

#while(True):
 #   print(robot.crane.readSensor())

#sensorReadings = robot.crane.readCell()
#print(sensorReadings)

#currentCell = robot.board.getCell(robot.current_x, robot.current_y)
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
"""if sensorReadings[0] == 1:
    currentCell.top_border.has_wall = True
if sensorReadings[1] == 1:
    currentCell.right_border.has_wall = True
if sensorReadings[2] == 1:
    currentCell.bottom_border.has_wall = True
if sensorReadings[3] == 1:
    currentCell.left_border.has_wall = True
currentCell.butter_distance = int(sensorReadings[5])
try:
    currentCell.toaster_distance = (int(sensorReadings[6]))
except:
    currentCell.toaster_distance = None"""

while True:
    simulationBoard.displayBoard()
    print("\n\n")
    board.displayBoard()
    robot.makeMove()

#board.getCell(0, 0).getTopBorder().setWall(True)
#board.getCell(0, 0).getLeftBorder().setWall(True)
#board.getCell(0, 0).getRightBorder().setWall(True)
#board.getCell(0, 0).getBottomBorder().setWall(True)

#board.getCell(3, 4).setObjectInCell(0)  # Butter
#board.getCell(5, 5).setObjectInCell(1)  # Mold
#board.getCell(2, 2).setObjectInCell(2)  # Toaster

#board.displayBoard()



#print(robot.crane.readCell())
#robot.makeMove()
