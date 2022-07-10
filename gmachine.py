from __future__ import division
import logging

from logging_config import *
import hal
from pulse import *
from coordinates import *

class GMachineException(Exception):
    """ Exceptions while processing gcode line.
    """
    pass

class GMachine(object):
    
    def __init__(self):
        """ Initialization.
        """
        self._position = Coordinates(0.0, 0.0, 0.0, 0.0)
        # init variables
        self._velocity = 2
        self._local = Coordinates(0.0, 0.0, 0.0, 0.0)
        self._convertCoordinates = 0   #單位換算
        self._absoluteCoordinates = 0
        self.reset()
        
         

    def reset(self):
        """ Reinitialize all program configurable thing.
        """
        
        self._local = Coordinates(0.0, 0.0, 0.0, 0.0)
        #self._convertCoordinates = 1.0
        self._absoluteCoordinates = True

    def __check_delta(self, delta):  ###限制要再修改
        pos = self._position + delta
        if not pos.is_in_aabb(Coordinates(0.0, 0.0, 0.0, 0.0), Coordinates(TABLE_SIZE_X_MM, TABLE_SIZE_Y_MM,TABLE_SIZE_Z_MM, 0)):
            raise GMachineException("out of effective area")
            
    def __check_velocity(self,max_velocity):
        if max_velocity.x > (1/(400 * MAX_VELOCITY_DELAYTIME_X + 0.036)*200*mm_per_step_x) \
                or max_velocity.y > (1/(400 * MAX_VELOCITY_DELAYTIME_Y + 0.036)*200*mm_per_step_y) \
                or max_velocity.z > (1/(400 * MAX_VELOCITY_DELAYTIME_Z + 0.036)*200*mm_per_step_z) \
                or max_velocity.e > (1/(400 * MAX_VELOCITY_DELAYTIME_E + 0.036)*200*mm_per_step_e):
            raise GMachineException("out of maximum speed")
        elif max_velocity.x < (1/(400 * delaytime_start_x + 0.036)*200*mm_per_step_x) \
                or max_velocity.y < (1/(400 * delaytime_start_y + 0.036)*200*mm_per_step_y) \
                or max_velocity.z < (1/(400 * delaytime_start_z + 0.036)*200*mm_per_step_z) \
                or max_velocity.e < (1/(400 * delaytime_start_e + 0.036)*200*mm_per_step_e):
            raise GMachineException("out of minimum speed")
            
            
    def _move_linear(self, delta, velocity):
        delta = delta.round(mm_per_step_x,\
                            mm_per_step_y,\
                            mm_per_step_z,\
                            mm_per_step_e)
        print('_move_linear')
        if delta.is_zero():
            print(delta.x,delta.y,delta.z,delta.e)
            print('delta.is_zero')
            return
        #self.__check_delta(delta)

        logging.info("Moving linearly {}".format(delta))
        gen = PulseGeneratorLinear(delta, velocity)
        #self.__check_velocity(gen.max_velocity())
        
        hal.add_task(gen)
        # save position
        self._local += delta
        print('self._local:',self._local)

    def safe_zero(self, x=True, y=True, z=True):
        """ Move head to zero position safely.
        :param x: boolean, move X axis to zero
        :param y: boolean, move Y axis to zero
        :param z: boolean, move Z axis to zero
        """
        if not x and z:
            self._move_linear(Coordinates(-self._position.x, 0, 0, 0), MAX_VELOCITY_MM_PER_MIN_X)
        elif not z and x:
            d = Coordinates(0, 0, -self._position.z, 0)
            self._move_linear(d, MAX_VELOCITY_MM_PER_MIN_Z)

        elif not x and not z:
            d = Coordinates(-self._position.x, 0, -self._position.z, 0)
            self._move_linear(d, min(MAX_VELOCITY_MM_PER_MIN_X, MAX_VELOCITY_MM_PER_MIN_Z))
        if not y: 
            self._move_linear(Coordinates(0, -self._position.y, 0, 0), MAX_VELOCITY_MM_PER_MIN_Y)

    def do_command(self, c, _position, _velocity):  ## 應該列入蛋糕尺寸、奶油種類等參數
        """ Perform action.
        :param gcode: GCode object which represent one gcode line
        :return String if any answer require, None otherwise.
        """
        print('do_command')
        logging.debug("got command " + str(c)) #logger依輕到嚴重分成5個等級DEBUG INFO WARNING ERROR CRITICAL，依照目前設定LOGGER等級，打印出程度以上的資訊，做為紀錄用。
        # read parameters
        delta = _position-self._local  #目標位置position -目前local
        self.__check_velocity(_velocity)
        
        print('delta',delta.x,delta.y,delta.z,delta.e)
        # select command and run it
        if c == 'G0':  # rapid move
            self._move_linear(delta, _velocity)
            print('G0')

        
        elif c == 'G28':  # home
            axises = self._local.is_zero()
            self.safe_zero(*axises)
            #hal.join()
            if not hal.calibrate(*axises):
                raise GMachineException("failed to calibrate")
        elif c =='Icing':
            pass
        elif c =='Scraching':
            pass
        elif c =='Filling':
            pass
            # 螺旋走法: x從0走到R，擠料速度Ve，線徑長(R*2)*pi，Vr應該跟當前r成正比，
        elif c == 'Decoration':
            pass
