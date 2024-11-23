#!/usr/bin/env python3

import board
import robot
from robot import Direction, Side
from board import Cell

board = board.Board()
robot = robot.Robot(board)

#robot.calibrate_light_sensor()
#board.getCell(0,0).setButterDistance(4)
#board.getCell(0,0).getBottomBorder().setWall(True)

#while(True):
 #   print(robot.crane.readSensor())

sensorReadings = robot.crane.readCell()


currentCell = robot.board.getCell(robot.current_x, robot.current_y)
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
    currentCell.getTopBorder().setWall(True)
if sensorReadings[1] == 1:
    currentCell.getRightBorder().setWall(True)
if sensorReadings[2] == 1:
    currentCell.getBottomBorder().setWall(True)
if sensorReadings[3] == 1:
    currentCell.getLeftBorder().setWall(True)
currentCell.setMoldDistance(int(sensorReadings[4]))
currentCell.setButterDistance(int(sensorReadings[5]))
currentCell.setToasterDistance(int(sensorReadings[6]))
currentCell.setObjectInCell(int(sensorReadings[7]))


while True:
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
"""
#START TESTE DE MOVER EM FRENTE ATÉ NAO CONSEGUIR ANDAR MAIS
counter = 0

while True:
    cell = board.getCell(0,0)
    if(not cell.getTopBorder().hasWall()):
        print("no top wall")
        robot.moveOneCell()
        print(robot.crane.readCell())
    elif (not cell.getRightBorder().hasWall()):
        print("no right wall")
        robot.moveOneCellToTheSide(Side.RIGHT)
        print(robot.crane.readCell())
    elif (not cell.getBottomBorder().hasWall()):
        print("no bottom wall")
        robot.moveOneCell(Direction.BACKWARD)
        print(robot.crane.readCell())
    elif (not cell.getLeftBorder().hasWall()):
        print("no left wall")
        robot.moveOneCellToTheSide(Side.LEFT)
        print(robot.crane.readCell())
    else:
        print("cant move")
        
    if (counter == 0):
        cell.getTopBorder().setWall(False)
    elif ( counter == 1):
        cell.getTopBorder().setWall(True)
        cell.getRightBorder().setWall(False)
    elif ( counter == 2 ):
        cell.getRightBorder().setWall(True)
        cell.getBottomBorder().setWall(False)
    elif(counter == 3):
        cell.getBottomBorder().setWall(True)
        cell.getLeftBorder().setWall(False)
    elif(counter == 4):
        break
    counter = counter + 1
    robot.waitNewTurn()
  """  



"""while robot.checkEdgesAndCorners():
    print("Moving forward. Current Position: ({robot.current_x}, {robot.current_y})")
    robot.moveOneCell(Direction.FORWARD)
    robot.checkEdgesAndCorners() 

    robot.waitNewTurn()"""

#print("Robot cannot move forward, stopping.")
#END TESTE DE MOVER EM FRENTE ATÉ NAO CONSEGUIR ANDAR MAIS

#robot.moveOneCell()
"""
robot.moveOneCellToTheSide(Side.LEFT)
robot.moveOneCell(Direction.BACKWARD)
robot.moveOneCellToTheSide(Side.RIGHT)
robot.waitNewTurn()

robot.moveOneCell()
robot.moveOneCellToTheSide(Side.LEFT)
robot.waitNewTurn()

robot.moveOneCell(Direction.BACKWARD)
robot.moveOneCellToTheSide(Side.RIGHT)
robot.waitNewTurn()

robot.moveOneCell()
robot.waitNewTurn()

robot.moveOneCellToTheSide(Side.LEFT)
robot.waitNewTurn()

robot.moveOneCell(False)
robot.waitNewTurn()

robot.moveOneCellToTheSide(Side.RIGHT)
robot.waitNewTurn()
"""
#robot.crane.publicAdjust(0.5)
#robot.crane.public90Degree()
#robot.crane.public90Degree()


#robot.moveOneCell()
#robot.moveOneCell(Direction.BACKWARD)
#print(robot.crane.readCell())


