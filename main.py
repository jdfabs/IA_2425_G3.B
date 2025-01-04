#!/usr/bin/env python3

import board
import robot

print("SIMULATION BOARD")
simulationBoard = board.Board()
print("EMPTY BOARD")
board = board.Board()

robot = robot.Robot(board, simulationBoard)

robot.calibrate_light_sensor()
#while(True):
    #robot.crane.colorSensor.calibrate_white()
    #print(robot.crane.readSensor())
    #robot.calibrate_light_sensor()
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

simulationBoard.displayBoard()
print("\n\n")
board.displayBoard()

while True:
    robot.makeMove()
    
    print("\n\n")
    

# CODE TO CALIBRATE LIGHT SENSOR IF NEEDED

#robot.calibrate_light_sensor()

#while(True):
#   print(robot.crane.readSensor())

#sensorReadings = robot.crane.readCell()
#print(sensorReadings)