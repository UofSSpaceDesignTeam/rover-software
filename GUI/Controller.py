#!/usr/bin/python

# A library providing the interface to an XBox 360 Controller.
# Written by Jordan Kubica

	# dependency list

from pygame import joystick


	# global variables

leftJoystickXScale = 1.0
leftJoystickYScale = 1.0
rightJoystickXScale = 1.0
rightJoystickYScale = 1.0
triggerScale = 1.0

leftJoystickXDeadzone = 0.16
leftJoystickYDeadzone = 0.16
rightJoystickXDeadzone = 0.16
rightJoystickYDeadzone = 0.16
triggerDeadzone = 0.1


	# function definitions

def setup():    # connect to xbox controller if one exists
	joystick.init()
	
	try:
		global controller
		controller = joystick.Joystick(0)
		controller.init();
	except:
		raise Exception("Error: Controller not detected")


def getAxes():  # returns a tuple of all axis data (-1.0 to 1.0)
	leftJoystickX = controller.get_axis(0)
	leftJoystickY = -controller.get_axis(1)
	rightJoystickX = controller.get_axis(4)
	rightJoystickY = -controller.get_axis(3)
	trigger = -controller.get_axis(2)
	
	if(leftJoystickX < leftJoystickXDeadzone
	and leftJoystickX > -leftJoystickXDeadzone):
		leftJoystickX = 0
	if(leftJoystickY < leftJoystickYDeadzone
	and leftJoystickY > -leftJoystickYDeadzone):
		leftJoystickY = 0
	if(rightJoystickX < rightJoystickXDeadzone
	and rightJoystickX > -rightJoystickXDeadzone):
		rightJoystickX = 0
	if(rightJoystickY < rightJoystickYDeadzone
	and rightJoystickY > -rightJoystickYDeadzone):
		rightJoystickY = 0
	if(trigger < triggerDeadzone
	and trigger > -triggerDeadzone):
		trigger = 0
	
	leftJoystickX *= leftJoystickXScale
	leftJoystickY *= leftJoystickYScale
	rightJoystickX *= rightJoystickXScale
	rightJoystickY *= rightJoystickYScale
	trigger *= triggerScale
	
	return (leftJoystickX, leftJoystickY, rightJoystickX,
	rightJoystickY, trigger)


def getButtons():	# returns the boolean state of all buttons
	buttonA = controller.get_button(0)
	buttonB = controller.get_button(1)
	buttonX = controller.get_button(2)
	buttonY = controller.get_button(3)
	buttonLB = controller.get_button(4)
	buttonRB = controller.get_button(5)
	buttonBack = controller.get_button(6)
	buttonStart = controller.get_button(7)
	buttonLeftJoystick = controller.get_button(8)
	buttonRightJoystick = controller.get_button(9)
	
	return (buttonA, buttonB, buttonX, buttonY, buttonLB, buttonRB,
	buttonBack, buttonStart, buttonLeftJoystick, buttonRightJoystick)


def getDPad():	# returns the x and y states of the D Pad buttons
	return controller.get_hat(0)	# format is (x, y): -1, 0, or 1

