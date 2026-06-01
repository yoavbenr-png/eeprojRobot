from xgolib import XGO
import time
dog = XGO(port='/dev/ttyAMA0', version='mini')
#dog.action(2)
time.sleep(5)
dog.move_x(18)
time.sleep(5)
dog.move_x(0)