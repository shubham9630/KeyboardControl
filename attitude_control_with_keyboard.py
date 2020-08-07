"""
Simple script for take off and control with arrow keys
"""

"""
Keys
Arm/Takeoff - t
Land -  l
RTL - r

Increasing Altitude - u
Decreasing Altitude - n

Roll Right - d
Roll Left - a
Pitch Forward - w
Pitch Backwards - z
Hold Altitude - s
Yaw Right - k
Yaw Left - j

Yaw Rate Right - o
Yaw Rate Left - i


"""


import time
import math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import readchar


#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551')  #--- Rpi

#-- Setup the commanded flying speed
gnd_speed = 1 # [m/s]

#Variables to check if the attituede rate has been set

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

 #  while not vehicle.is_armable:
 #    print("waiting to be armable")
 #     time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed:
       print(" Waiting for arming...")
       time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(" Altitude: %f  Desired: %f" %
            (v_alt, altitude))
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Bitmask to indicate which dimensions should be ignored by the vehicle
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that
    none of the setpoint dimensions should be ignored). Mapping:
    bit 1: x,  bit 2: y,  bit 3: z,
    bit 4: vx, bit 5: vy, bit 6: vz,
    bit 7: ax, bit 8: ay, bit 9:


    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def condition_yaw(heading, direction, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        direction,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,yaw_angle = None,
                        roll_rate=0.0, pitch_rate=0.0, yaw_rate = 0.0,
                        use_yaw_rate = False, thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        math.radians(roll_rate), # Body roll rate in radian
        math.radians(pitch_rate), # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)


def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None,
                 roll_rate=0.0, pitch_rate=0.0, yaw_rate = 0.0,
                 use_yaw_rate = False, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,yaw_angle,
                        roll_rate, pitch_rate, yaw_rate,
                        use_yaw_rate, thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,yaw_angle,
                            roll_rate, pitch_rate, yaw_rate,
                            use_yaw_rate, thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0, 0,
                         0, 0, 0,
                         True, thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

def set_roi(location):
    """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.

    For more information see:
    http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
    """
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)


#-- Key event function
def key():
  while True:
    key = readchar.readkey()
        # Taking off and Landing
        if event.keysym == 't':
            print("Taking off to 5 meters")
            arm_and_takeoff(7)
        elif event.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("RTL")
        elif event.keysym == 'l:
            print("l pressed >> Set the vehicle to Land")
            vehicle.mode = VehicleMode("LAND")

        #Increasing/Decreasing Altitude
        elif event.keysym == 'u':
            point1 = LocationGlobalRelative(0, 0, 3)
            vehicle.simple_goto(point1)
        elif event.keysym == 'n':
            point1 = LocationGlobalRelative(0, 0, -3)
            vehicle.simple_goto(point1)

        #Roll, Pitch, Yaw command
        elif event.keysym == 'd':
            set_attitude(roll_angle=10, duration = 3)
            print(" Attitude 10 deg Roll: %s", vehicle.attitude.roll)
        elif event.keysym == 'a':
            set_attitude(roll_angle=-10, duration = 3)
            print(" Attitude -10 deg Roll: %s", vehicle.attitude.roll)
        elif event.keysym == 'w':
            set_attitude(pitch_angle=10, duration = 3)
            print(" Attitude 10 deg pitch: %s", vehicle.attitude.pitch)
        elif event.keysym == 'z':
            set_attitude(pitch_angle=-10, duration = 3)
            print(" Attitude -pitch: %s", vehicle.attitude.pitch)
        elif event.keysym == 'k':
            condition_yaw(90,1, relative=True)
        elif event.keysym == 'j':
            condition_yaw(90,-1, relative=True)

        #Yaw rate command
        elif event.keysym == 'o':
            set_attitude(yaw_rate= 20, use_yaw_rate=True, duration = 1)
            set_velocity_body(vehicle, 0, 0, 0)
            print("YawRate - 20deg/s: ", vehicle.attitude.yaw)
        elif event.keysym == 'i':
            set_attitude(yaw_rate=-20, use_yaw_rate=True, duration = 1)
            print("YawRate - -20deg/s", vehicle.attitude.yaw)
            set_velocity_body(vehicle, 0, 0, 0)
         elif key =='\x1b[A':
             set_velocity_body(vehicle, gnd_speed, 0, 0)
             print("Key: ","Up arrow key pressed, Moving Forward")
        elif key =='\x1b[B':
             set_velocity_body(vehicle, -gnd_speed, 0,0)
             print("Key: ","Down arrow key pressed, Moving Backward")
         elif key=='\x1b[C':
             set_velocity_body(vehicle, 0, gnd_speed, 0)
             print("Key: ","Right arrow key pressed, Moving Right")
         elif key =='\x1b[D':
             set_velocity_body(vehicle, 0, -gnd_speed, 0)
             print("Key: ","Left arrow key pressed, Moving Left")
         elif key == 'r':
             print("r pressed >> Set the vehicle to RTL")
             vehicle.mode = VehicleMode("RTL")
         elif key == '\x03':
             print ("Ctrl+c pressed, will work only if disarmed")
             if vehicle.armed == True:
                print("Vehicle is armed, cannot exit")
             else:
                break
          else:
                 print("Key: ", key)






#---- MAIN FUNCTION
#- Takeoff
arm_and_takeoff(10)


if __name__=='__main__':
  print(">> Control the drone with the arrow keys. Press r for RTL mode")
  key()

