"""
Simple script for take off and control with arrow keys
"""


import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import readchar


#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:192.168.43.14:14550') #-- UDP of Raspberry Pi
print("Connected. Control with keyboard now")

    
#-- Setup the commanded flying speed
gnd_speed = 1 # [m/s]

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   # while not vehicle.is_armable:
   #    print("waiting to be armable")
   #    time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
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
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

    

#-- Key event function

def key():
        while True:
            #c = readchar.readchar()
            key = readchar.readkey()
            if key =='\x1b[A':
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
#arm_and_takeoff(10)


if __name__=='__main__':
    key()







