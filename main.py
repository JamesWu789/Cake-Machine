from logging_config import *
from hal import *
from pulse import *
from coordinates import *
from gmachine import GMachine, GMachineException

machine = GMachine()




_position=Coordinates(100,1,200,1)
_velocity=Coordinates(100,10,10,10)

res=machine.do_command('G0',_position,_velocity)