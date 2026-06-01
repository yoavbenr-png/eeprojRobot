from xgolib import XGO
import time

dog = XGO(port='/dev/ttyAMA0', version='mini')
dog.action(2)
time.sleep(2)

dog.turn(25)
time.sleep(2)

dog.turn(0)