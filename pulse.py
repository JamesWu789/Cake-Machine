from __future__ import division
import logging
import math

from logging_config import *
from coordinates import *

SECONDS_IN_MINUTE = 60.0

class PulseGenerator(object):
    

    def __init__(self, delta):
        """ Create object. Do not create directly this object, inherit this
            class and implement interpolation function and related methods.
            All child have to call this method ( super().__init__() ).
            :param delta: overall movement delta in mm, uses for debug purpose.
        """
        self._iteration_x = 0
        self._iteration_y = 0
        self._iteration_z = 0
        self._iteration_e = 0
        self._iteration_direction = None
        self._acceleration_time_s = 0.0
        self._linear_time_s = 0.0
        self._2Vmax_per_a = 0.0
        self._delta = delta
        

    def _adjust_velocity(self, velocity_mm_sec): #依比例降低每一軸速度，若有人超速
        """ Automatically decrease velocity to all axises proportionally if
        velocity for one or more axises is more then maximum velocity for axis.
        :param velocity_mm_sec: input velocity.
        :return: adjusted(decreased if needed) velocity.
        """
        if not self.AUTO_VELOCITY_ADJUSTMENT:
            return velocity_mm_sec
        k = 1.0
        if velocity_mm_sec.x * SECONDS_IN_MINUTE > MAX_VELOCITY_MM_PER_MIN_X:
            k = min(k, MAX_VELOCITY_MM_PER_MIN_X
                    / velocity_mm_sec.x / SECONDS_IN_MINUTE)
        if velocity_mm_sec.y * SECONDS_IN_MINUTE > MAX_VELOCITY_MM_PER_MIN_Y:
            k = min(k, MAX_VELOCITY_MM_PER_MIN_Y
                    / velocity_mm_sec.y / SECONDS_IN_MINUTE)
        if velocity_mm_sec.z * SECONDS_IN_MINUTE > MAX_VELOCITY_MM_PER_MIN_Z:
            k = min(k, MAX_VELOCITY_MM_PER_MIN_Z
                    / velocity_mm_sec.z / SECONDS_IN_MINUTE)
        if velocity_mm_sec.e * SECONDS_IN_MINUTE > MAX_VELOCITY_MM_PER_MIN_E:
            k = min(k, MAX_VELOCITY_MM_PER_MIN_E
                    / velocity_mm_sec.e / SECONDS_IN_MINUTE)
        if k != 1.0:
            logging.warning("Out of speed, multiply velocity by {}".format(k))
        return velocity_mm_sec * k

    def _get_movement_parameters(self):
        """ Get parameters for interpolation. This method have to be
            reimplemented in parent classes and should calculate 3 parameters.
            :return: Tuple of three values:
                acceleration_time_s: time for accelerating and breaking motors
                                 during movement
                linear_time_s: time for uniform movement, it is total movement
                                 time minus acceleration and braking time
                max_axis_velocity_mm_per_sec: maximum axis velocity of all
                                  axises during movement. 
Even if whole movement is accelerated, this value should be calculated as top velocity. """
        raise NotImplementedError

    def _interpolation_function(self, ix, iy, iz, ie):
        """ Get function for interpolation path. This function should returned
            values as it is uniform movement. There is only one trick, function
            must be expressed in terms of position, i.e. t = S / V for linear,
            where S - distance would be increment on motor minimum step.
        :param ix: number of pulse for X axis.
        :param iy: number of pulse for Y axis.
        :param iz: number of pulse for Z axis.
        :param ie: number of pulse for E axis.
        :return: Two tuples. First is tuple is directions for each axis,
                 positive means forward, negative means reverse. Second is
                 tuple of times for each axis in us or None if movement for
                 axis is finished.
        """
        raise NotImplementedError

    def __iter__(self):
        """ Get iterator.
        :return: iterable object.
        """
        (self._acceleration_time_s, self._linear_time_s,
         max_axis_velocity_mm_per_sec) = self._get_movement_parameters()
        # helper variable
        self._iteration_x = 0
        self._iteration_y = 0
        self._iteration_z = 0
        self._iteration_e = 0
        self._iteration_direction = None
        logging.debug(', '.join("%s: %s" % i for i in vars(self).items()))
        return self

    def _to_accelerated_time(self, pt_s):
        """ Internal function to translate uniform movement time to time for
            accelerated movement.
        :param pt_s: pseudo time of uniform movement.
        :return: time for each axis or None if movement for axis is finished.
        """
        # acceleration
        # S = Tpseudo * Vmax = a * t^2 / 2
        t = math.sqrt(pt_s * self._2Vmax_per_a)
        if t <= self._acceleration_time_s:
            return t

        # linear
        # pseudo acceleration time Tpseudo = t^2 / ACCELERATION_FACTOR_PER_SEC
        t = self._acceleration_time_s + pt_s - (self._acceleration_time_s ** 2
                                                / self._2Vmax_per_a)
        # pseudo breaking time
        bt = t - self._acceleration_time_s - self._linear_time_s
        if bt <= 0:
            return t

        # braking
        # Vmax * Tpseudo = Vlinear * t - a * t^2 / 2
        # V on start braking is Vlinear = Taccel * a = Tbreaking * a
        # Vmax * Tpseudo = Tbreaking * a * t - a * t^2 / 2
        d = self._acceleration_time_s ** 2 - self._2Vmax_per_a * bt
        if d > 0:
            d = math.sqrt(d)
        else:
            d = 0
        return 2.0 * self._acceleration_time_s + self._linear_time_s - d

    def __next__(self):
        # for python3
        return self.next()

    def next(self):
        """ Iterate pulses.
        :return: Tuple of five values:
                    - first is boolean value, if it is True, motors direction
                        should be changed and next pulse should performed in
                        this direction.
                    - values for all machine axises. For direction update,
                        positive values means forward movement, negative value
                        means reverse movement. For normal pulse, values are
                        represent time for the next pulse in microseconds.
                 This iteration strictly guarantees that next pulses time will
                 not be earlier in time then current. If there is no pulses
                 left StopIteration will be raised.
        """
        direction, (tx, ty, tz, te) = self._interpolation_function (self._iteration_x, self._iteration_y, self._iteration_z, self._iteration_e)
        # check if direction update:
        if direction != self._iteration_direction:
            self._iteration_direction = direction
            dir_x, dir_y, dir_z, dir_e = direction
            if STEPPER_INVERTED_X:
                dir_x = -dir_x
            if STEPPER_INVERTED_Y:
                dir_y = -dir_y
            if STEPPER_INVERTED_Z:
                dir_z = -dir_z
            if STEPPER_INVERTED_E:
                dir_e = -dir_e
            return True, dir_x, dir_y, dir_z, dir_e
        # check condition to stop
        if tx is None and ty is None and tz is None and te is None:
            raise StopIteration

        # convert to real time
        m = None
        for i in (tx, ty, tz, te):
            if i is not None and (m is None or i < m):
                m = i
        am = self._to_accelerated_time(m)
        # sort pulses in time
        if tx is not None:
            if tx > m:
                tx = None
            else:
                tx = am
                self._iteration_x += 1
        if ty is not None:
            if ty > m:
                ty = None
            else:
                ty = am
                self._iteration_y += 1
        if tz is not None:
            if tz > m:
                tz = None
            else:
                tz = am
                self._iteration_z += 1
        if te is not None:
            if te > m:
                te = None
            else:
                te = am
                self._iteration_e += 1

        return False, tx, ty, tz, te

    def total_time_s(self):
        """ Get total time for movement.
        :return: time in seconds.
        """
        acceleration_time_s, linear_time_s, _ = self._get_movement_parameters()
        return acceleration_time_s * 2.0 + linear_time_s

    def delta(self):
        """ Get overall movement distance.
        :return: Movement distance for each axis in millimeters.
        """
        return self._delta

    def max_velocity(self):
        """ Get max velocity for each axis.
        :return: Vector with max velocity(in mm per min) for each axis.
        """
        _, _, v = self._get_movement_parameters()
        return v * SECONDS_IN_MINUTE


class PulseGeneratorLinear(PulseGenerator): #在Gmachine中
    def __init__(self, delta_mm, velocity_mm_per_min):  #個軸獨立計算，step固定，計算速度
        print('PulseGeneratorLinear')
        super(PulseGeneratorLinear, self).__init__(delta_mm)
        distance_mm = abs(delta_mm)  #type: Coordinates
        _distance_mm = [distance_mm.x, distance_mm.y, distance_mm.z, distance_mm.e]
        
        # 計算目標delaytime => delaytime=[(MM/Step)/Velocity*200-0.036]/400
        self.max_velocity_delaytime = (Coordinates(mm_per_step_x, mm_per_step_y, mm_per_step_z, mm_per_step_e)/(velocity_mm_per_min)*Coordinates(200,200,200,200)-Coordinates(0.036,0.036,0.036,0.036))/Coordinates(400,400,400,400)  #前方mm_per_step_x要調整，為速度轉速放大比
        
        self.delaytime_start = Coordinates(delaytime_start_x, delaytime_start_y, delaytime_start_z, delaytime_start_e )
        
        
        stepdelay_add = Coordinates(stepdelay_add_x, stepdelay_add_y, stepdelay_add_z, stepdelay_add_e)
        # 計算加減速所需步數 =>  (最大速度delay-初速delay)/加速度delay_add
        self.acc_steps = math.ceil((self.delaytime_start-self.max_velocity_delaytime)/(stepdelay_add))
        
        #等速移動所需步數
        self.linear_steps = [0,0,0,0]
        
        mm_per_step = Coordinates(mm_per_step_x, mm_per_step_y, mm_per_step_z, mm_per_step_e)
        _mm_per_step = [mm_per_step_x, mm_per_step_y, mm_per_step_z, mm_per_step_e]
        
        '''print('剛開始acc_steps',self.acc_steps.x,',max_velocity_delaytime',self.max_velocity_delaytime.x)
        print('所需距離',self.acc_steps.x*mm_per_step.x*2,'總距離',distance_mm.x)
        print(self.acc_steps*mm_per_step*Coordinates(2,2,2,2),distance_mm)'''
        
        _max_velocity_mm_per_sec=[0,0,0,0]
        #########各軸獨立判斷
        # check if there is enough space to accelerate and brake, adjust velocity
        for i,j in enumerate(self.acc_steps*mm_per_step*Coordinates(2,2,2,2) > distance_mm):
            if j and i==0:  ###如果距離不夠加速到最快，一半距離用來加速，另一半減速
                print("x not enough space")
                self.acc_steps.x=int(distance_mm.x/mm_per_step_x/2)
                # 計算最大速度及其delaytime
                self.max_velocity_delaytime.x = self.delaytime_start.x - (self.acc_steps.x * stepdelay_add.x)
                _max_velocity_mm_per_sec[i] = 1 / (400 * self.max_velocity_delaytime.x + 0.036) * 200 * mm_per_step_x
            elif j and i==1:
                self.acc_steps.y=int(distance_mm.y/mm_per_step_y/2)
                self.max_velocity_delaytime.y = self.delaytime_start.y - (self.acc_steps.y * stepdelay_add.y)
                _max_velocity_mm_per_sec[i] = 1 / (400 * self.max_velocity_delaytime.y + 0.036) * 200 * mm_per_step_y
            elif j and i==2:
                self.acc_steps.z=int(distance_mm.z/mm_per_step_z/2)
                self.max_velocity_delaytime.z = self.delaytime_start.z - (self.acc_steps.z * stepdelay_add.z)
                _max_velocity_mm_per_sec[i] = 1 / (400 * self.max_velocity_delaytime.z + 0.036) * 200 * mm_per_step_z
            elif j and i==3:
                self.acc_steps.e=int(distance_mm.e/mm_per_step_e/2)
                self.max_velocity_delaytime.e = self.delaytime_start.e - (self.acc_steps.e * stepdelay_add.e)
                _max_velocity_mm_per_sec[i] = 1 / (400 * self.max_velocity_delaytime.e + 0.036) * 200 * mm_per_step_e
            else:
                # calculate linear time => 線性步數 = (全部步數-加速步數*2)
                self._acc_steps = [self.acc_steps.x, self.acc_steps.y, self.acc_steps.z, self.acc_steps.e]
                self.total_steps = math.floor(_distance_mm[i]/_mm_per_step[i])
                self.linear_steps[i] = self.total_steps-self._acc_steps[i]*2
        
        self._linear_steps=Coordinates(self.linear_steps[0], self.linear_steps[1], self.linear_steps[2], self.linear_steps[3])
            
        self._direction = (math.copysign(1, delta_mm.x),math.copysign(1, delta_mm.y),math.copysign(1, delta_mm.z),math.copysign(1, delta_mm.e))#方向 (後面的 +- 乘以前面的1)
        
    
        '''self.linear_time_s = (linear_distance_mm                     
                                  / self.max_velocity_mm_per_sec.length())
        self._total_pulses_x = round(distance_mm.x * STEPPER_PULSES_PER_MM_X)
        self._total_pulses_y = round(distance_mm.y * STEPPER_PULSES_PER_MM_Y)
        self._total_pulses_z = round(distance_mm.z * STEPPER_PULSES_PER_MM_Z)
        self._total_pulses_e = round(distance_mm.e * STEPPER_PULSES_PER_MM_E)'''

    def _get_movement_parameters(self):
        """ Return movement parameters, see super class for details.
        """
        return (self.acc_steps,
                self.delaytime_start,
                self.max_velocity_delaytime,
                self.linear_steps)