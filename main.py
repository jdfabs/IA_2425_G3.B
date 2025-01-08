#!/usr/bin/env python3

import board
import robot

simulationBoard = board.Board("random")
board = board.Board("empty")
robot = robot.Robot(board, simulationBoard)

#robot.calibrateLightSensor()

print("STARTING STATE")
simulationBoard.displayBoard()
print("CURRENT STATE - ROBOT MEMORY")
board.displayBoard()
input("###################### PRESS ENTER TO START ######################")

while True:
    robot.makeMove()
