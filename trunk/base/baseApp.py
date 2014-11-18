#Import all of the thread modules
from communicationThread import communicationThread
from guiThread import guiThread
from inputThread import inputThread
from navigationThread import navigationThread
from panelThread import panelThread

import time

def startThreads():
	commThread.start()
	inputThread.start()
	navThread.start()
	panelThread.start()
	guiThread.start()

def stopThreads():
	commThread.stop()
	inputThread.stop()
	navThread.stop()
	panelThread.stop()
	guiThread.stop()


# make each top-level thread
commThread = communicationThread()
inputThread = inputThread()
navThread = navigationThread()
panelThread = panelThread()
guiThread = guiThread()

# configure threads
commThread.sendPort = 8001
commThread.receivePort = 8000
commThread.sendInterval = 0.25
commThread.inputThread = inputThread
commThread.navThread = navThread
commThread.panelThread = panelThread
commThread.guiThread = guiThread

inputThread.commThread = commThread

# test execution
print("starting")
startThreads()
print("running")
time.sleep(2)

# test code goes here

time.sleep(20)
print("stopping")
stopThreads()
print("done")

