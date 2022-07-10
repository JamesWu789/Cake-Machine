from coordinates import *


'''define GPIO pins'''
GPIO_pins = (14, 15, 18)
direction= 20
step = 21
ENABLE = 16

GPIO_pins_2 = (17, 27 ,22)
direction_2 = 23
step_2 = 24
ENABLE_2 = 12


'''速度加速度限制 & 馬達減速比設定'''
MAX_VELOCITY_DELAYTIME_X=0.00035
MAX_VELOCITY_DELAYTIME_Y=0.00035
MAX_VELOCITY_DELAYTIME_Z=0.00035
MAX_VELOCITY_DELAYTIME_E=0.00035

MAX_VELOCITY_MM_PER_MIN_X=0
MAX_VELOCITY_MM_PER_MIN_Y=0
MAX_VELOCITY_MM_PER_MIN_Z=0
MAX_VELOCITY_MM_PER_MIN_E=0

STEPPER_PULSES_PER_MM_X=1
STEPPER_PULSES_PER_MM_Y=1
STEPPER_PULSES_PER_MM_Z=1
STEPPER_PULSES_PER_MM_E=1

STEPPER_MAX_ACCELERATION_MM_PER_S2=1

multiply_x=1
multiply_y=1
multiply_z=1
multiply_e=1
#Coordinates(multiply_x, multiply_y, multiply_z, multiply_e)
######################################################### 被import的人，不能直接操作其他外部function(coordinates.Coordinates)，要在def內，(不確定，可修改)

## 需轉向?
STEPPER_INVERTED_X=False
STEPPER_INVERTED_Y=False
STEPPER_INVERTED_Z=False
STEPPER_INVERTED_E=False

## 初速度
delaytime_start_x=0.05
delaytime_start_y=0.05
delaytime_start_z=0.05
delaytime_start_e=0.05

#Coordinates(delaytime_start_x, delaytime_start_y, delaytime_start_z, delaytime_start_e)

##馬達速度極限
MAX_VELOCITY_STEPDELAY=0.00035

## 加減速最大允許值
stepdelay_add_x=0.003
stepdelay_add_y=0.003
stepdelay_add_z=0.003
stepdelay_add_e=0.003
#Coordinates(stepdelay_add_x, stepdelay_add_y, stepdelay_add_z, stepdelay_add_e)

## 每一軸step換算到距離 mm/step
mm_per_step_x=1
mm_per_step_y=1
mm_per_step_z=1
mm_per_step_e=1
#Coordinates(mm_per_step_x, mm_per_step_y, mm_per_step_z, mm_per_step_e)

''' 機台限制設定 '''
TABLE_SIZE_X_MM = 60
TABLE_SIZE_Y_MM = 60
TABLE_SIZE_Z_MM = 60
TABLE_SIZE_E_MM = 60