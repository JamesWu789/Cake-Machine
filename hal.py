import RPi.GPIO as GPIO
import threading
import sys
import time
from logging_config import *


def add_task(gen):    ##傳遞上需要傳 1.Target 2.Args(list) 3.是否要join (0,1)
    print('add_task')
    Motor_X = A4988Nema(direction, step, GPIO_pins, ENABLE, "A4988")
    Motor_Z = A4988Nema(direction_2, step_2, GPIO_pins_2, ENABLE_2, "A4988")
    
    missions=[] 
    for i,j in enumerate(gen.acc_steps==Coordinates(0,0,0,0)):
        if not i and i==0:
            missions.append(threading.Thread(target=Motor_X.motor_go, args=(False,"Full", gen.acc_steps.x, gen.delaytime_start.x, False, 0, stepdelay_add_x, gen.max_velocity_delaytime.x,)))
        if not i and i==1:
            pass
        if not i and i==2:
            missions.append(threading.Thread(target=Motor_Z.motor_go, args=(False,"Full", gen.acc_steps.z, gen.delaytime_start.z, False, 0, stepdelay_add_z, gen.max_velocity_delaytime.z,)))
        if not i and i==3:
            pass
    for i in range(len(missions)):
        missions[i].start()
    for i in range(len(missions)):  # 必須將start跟join分開，否則無法同時控制多個馬達
        missions[i].join()

class StopMotorInterrupt(Exception):
    """ Stop the motor """
    pass

class A4988Nema(object):
    
    def __init__(self, direction_pin, step_pin, mode_pins,ENABLE_pin, motor_type="A4988"):
        self.motor_type = motor_type
        self.direction_pin = direction_pin
        self.step_pin = step_pin
        self.ENABLE_pin =  ENABLE_pin
        self.mode_pins = mode_pins
        self.stop_motor = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENABLE,GPIO.OUT)
        GPIO.setwarnings(False)

    def motor_stop(self):
        """ Stop the motor """
        GPIO.output(self.ENABLE_pin, True)
        self.stop_motor = True

    def resolution_set(self, steptype):
        """ method to calculate step resolution
        based on motor type and steptype"""
        if self.motor_type == "A4988":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (1, 1, 1)}
        if steptype in resolution:
            pass
        else:
            print("Error invalid steptype: {}".format(steptype))
            quit()

    def motor_go(self, clockwise=False, steptype="Full", steps=200, stepdelay=.001, verbose=False, initdelay=0, stepdelay_add=0, target_stepdelay=.03):
        self.stop_motor = False
        # setup GPIO
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.ENABLE_pin,GPIO.OUT)
        GPIO.output(self.direction_pin, clockwise)
        GPIO.output(self.ENABLE_pin, False)
        GPIO.setup(self.mode_pins, GPIO.OUT)

        try:
            # dict resolution
            GPIO.output(self.ENABLE_pin, False)
            self.resolution_set(steptype)
            time.sleep(initdelay)
            print('target_stepdelay',target_stepdelay)
            print(steps)
            for i in range(steps):
                if self.stop_motor:
                    raise StopMotorInterrupt
                else:
                    print(stepdelay)
                    GPIO.output(self.step_pin, True)
                    time.sleep(stepdelay)
                    GPIO.output(self.step_pin, False)
                    time.sleep(stepdelay)
                    stepdelay -= stepdelay_add
                    if stepdelay<target_stepdelay:  
                        stepdelay= target_stepdelay
                    
                    if verbose:
                        print("Steps count {}".format(i+1), end="\r", flush=True)
            #GPIO.output(self.ENABLE_pin, 1)
            print('acc_steps:',steps,'target_stepdelay:',target_stepdelay,'stepdelay:',stepdelay)
            
        except KeyboardInterrupt:
            print("User Keyboard Interrupt : RpiMotorLib:")
        except StopMotorInterrupt:
            print("Stop Motor Interrupt : RpiMotorLib: ")
        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("RpiMotorLib  : Unexpected error:")
                
        finally:        
            # cleanup
            GPIO.output(self.step_pin, False)
            for pin in self.mode_pins:
                GPIO.output(pin, False)

        '''def degree_calc(steps, steptype):
            """ calculate and returns size of turn in degree, passed number of steps and steptype"""
            degree_value = {'Full': 1.8, 'Half': 0.9, '1/4': .45, '1/8': .225, '1/16': 0.1125, '1/32': 0.05625, '1/64': 0.028125, '1/128': 0.0140625}
            degree_value = (steps*degree_value[steptype])
            return degree_value'''
        
