#!/usr/bin/python3
# test
import time
import math
from pymavlink import mavutil

# connect to SITL
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

# wait for a heartbeat
master.wait_heartbeat()

# inform user
print("Connected to system:", master.target_system, ", component:", master.target_component)


# mode Guided 
master.mav.command_long_send(
    master.target_system,
    master.target_component,
mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0, # confirmation
    1, # param1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
    4, # param2 (flightmode number)
    0, # param3
    0, # param4
    0, # param5
    0, # param6
    0) # param7

# Command a yaw
master.mav.command_long_send(
    master.target_system,
    master.target_component,
mavutil.mavlink.MAV_CMD_CONDITION_YAW,
    0,
    90, # deg
    20, # speed
    -1, # dir (1=CW)
    1, # abs (0) or rel (1)
    0, # param5
    0, # param6
    0) # param7
