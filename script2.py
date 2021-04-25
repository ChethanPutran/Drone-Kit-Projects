##Dependencies

from dronekit import connect , VehicleMode , LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection = args.connect
    if not connection:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection = sitl.connection_string()

    vehicle = connect(connection, wait_ready=True)
    return vehicle

#Send a velocity command with +x being the heading of the drone
def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0, 0, 0,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
    0b0000111111000111,  #Bitmask
    0, 0, 0,#Position
    vx, vy, vz,#Velocity
    0, 0, 0,#Acceleration
    0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()  


#Send a velocity command with +x being true NORTH of Earth
def send_global_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0, 0, 0,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  #Bitmask
    0, 0, 0,#Position
    vx, vy, vz,#Velocity
    0, 0, 0,#Acceleration
    0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()    

def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable")
        time.sleep(1)
    print("Vehicle is now armable")

    #Changing the mode to GUIDED Flight mode
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")

    print("Vehicle now is in GUIDED flight mode")

    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed")
        time.sleep(1)
    print("Virtual props are spinning!!!")

    vehicle.simple_takeoff(targetHeight)

    while True:
        print("Current Attitude: %d" % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt > 0.95 * targetHeight:
            break
        time.sleep(1)
        print("Target altitude reached!!!")
        return None

vehicle = connectMyCopter()
print("About to takeoff...")

vehicle.mode = VehicleMode("GUIDED")
arm_and_takeoff(2)  #Flying 2m up
vehicle.mode = VehicleMode("LAND")  #Landing the drone once it move to 2m targetHeight
time.sleep(2)
vehicle.close()