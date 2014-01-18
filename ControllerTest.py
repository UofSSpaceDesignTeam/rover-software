#!/usr/bin/python

# A simple utility to test the Controller library.
# Added by Jordan Kubica

	# dependency list

import pygame
import Controller
import time


	# function definitions

def printAxes():    # print the status of all controller axes
	axes = Controller.getAxes()
	roundedAxes = [round(i, 2) for i in axes]
	print("  Lx   Ly   Rx   Ry    T")
	print(roundedAxes)
	print("\n")


def printButtons():	# print the state of all controller buttons
	buttons = Controller.getButtons()
	print(" A  B  X  Y  LB RB Bk St Ls Rs")
	print(buttons)
	print("\n")


def printDPad():	# print the state of the D Pad buttons
	dPad = Controller.getDPad()
	print("L/R U/D")
	print(dPad)
	print("\n")


	# main execution
	
pygame.init()
try:
	Controller.init()
except Exception as e:
	print(e)
	exit(1)
	
while(True):
	pygame.event.pump()
	printAxes()
	printButtons()
	printDPad()
	time.sleep(0.25)

