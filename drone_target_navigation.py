import time
from pymavlink import mavutil
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
from call_from_yolo import main

def pix_to_meter(x,y):
    image_width = 1920
    image_height = 1080
    X_relative_pixel = x  
    Y_relative_pixel = y  
    Distance_to_object = vehicle.location.global_relative_frame.alt
    D_x = (2 * Distance_to_object * math.tan(math.radians(77.74) / 2)) / image_width
    D_y = (2 * Distance_to_object * math.tan(math.radians(62.27) / 2)) / image_height
    X_relative_meter = X_relative_pixel * D_x
    Y_relative_meter = Y_relative_pixel * D_y
    return X_relative_meter, Y_relative_meter

def go_to_location(drone_latitude, drone_longitude, relative_x, relative_y):
    global object_latitude, object_longitude
    latitude_to_meters = 111111
    longitude_to_meters = 111111
    object_latitude = drone_latitude + (relative_y / latitude_to_meters)
    object_longitude = drone_longitude + (relative_x / (longitude_to_meters * math.cos(math.radians(drone_latitude))))
    print("the detected objects latitude and longitude is: ({},{})".format(object_latitude, object_longitude))
    a=int(input("Would you like to proceed?? 1 for yes 0 for no: "))
    if a==1:
        guided_mode(vehicle, object_latitude, object_longitude, 28.956)
    else:
        print("doing nothing...")
        pass

def goto_position_target_local_ned(north, east, up=0):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b110111111000,  # type_mask (only positions enabled)
        north, east, up,
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def move_servo(channel_num, pwm_value):

    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # confirmation
        channel_num,  # servo number
        pwm_value,  # servo value
        0, 0, 0, 0, 0  # additional parameters (not used in this command)
    )
    vehicle.send_mavlink(msg)

def guided_mode(vehicle, lat, lon, alt):
    if vehicle.mode == "GUIDED":
        #print("Switching to GUIDED mode")
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
        #print("Navigating to target position")
        vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))
        time.sleep(5)
        #print("Reached target position")
    else:
        print("vehicle not in guided, skipping takeover")


def is_vehicle_armed2(vehicle):
    return vehicle.armed


def yolocheck():

    detected=0 #0 not 1 yes
    check_var=0

    human_coordinates, object_coordinates = main()

    while (check_var==0 and vehicle.mode == "GUIDED"):   
            
        # wait for yolo signal
        global vehlan,vehlon
        
        if len(object_coordinates) != 0:
            detected = 1
            vehlan=vehicle.location.global_frame.lat
            vehlon=vehicle.location.global_frame.lon
        
        if detected==1:
            
            #let x and y be the relative coor from yolo
            print(f"All Detected Objects are: {object_coordinates}.")
            selected_object = int(input(f"Select Object to proceed to: "))

            x=object_coordinates[selected_object][0]
            y=object_coordinates[selected_object][1]

            go_to_location(vehlan, vehlon, x, y)
            check_var=1
        
        elif vehicle.mode != "GUIDED":
            time.sleep(1)

        else:
            print("object not detected in the frame")
            check_var=int(input("do you want to wait for other frame? enter 0 to stay or 1 to exit and move to next midpoint: "))

            time.sleep(1)

def midpoint_nav(drops, dd):
    possss=[[38.3145062, -76.5450881],[38.3144589, -76.5448366], [38.3144157, -76.5445724], [38.3143642, -76.5442821], [38.3143363, -76.5440957]]

    while (drops>0 and vehicle.mode == "GUIDED"):
        
        #Option of to go different midpoint in between mission, if dont want it change dd  variable below to any random non conflicting variable
        a=int(input("would you like to proceed to midpoints? 1 for yes 0 for no: "))
        if a==1:
            dd=input("which midpoint to navigate?? range is form [0 to 4] current value to go to midpoint is {}: ".format(dd))
            latt=float(possss[int(dd)][0])
            lonn=float(possss[int(dd)][1])
            guided_mode(vehicle, latt, lonn, 27.432)
            check_var2=1
            while(check_var2==1 and vehicle.mode == "GUIDED"):
                if abs(vehicle.location.global_relative_frame.lat - latt) < 0.0001 and (abs(vehicle.location.global_relative_frame.lon - lonn) < 0.0001):
                    check_var2=0
                else:
                    pass 
            time.sleep(1)
            yolocheck()
            check_var2=1
            print(vehlan)
            print(vehlon)
            while(check_var2==1 and vehicle.mode == "GUIDED"):
                if abs(vehicle.location.global_relative_frame.lat - vehlan) < 0.0003 and (abs(vehicle.location.global_relative_frame.lon - vehlon) < 0.0003):
                    check_var2=0
                else:
                    pass 
            
            aaa=int(input("would you like to proceed for payload drop? 1 for yes 0 for no: "))
            if aaa==1:
                confirm_drop()
                drops-=1
            else:
                pass
        elif vehicle.mode != "GUIDED":
            time.sleep(1)

        else:
            aa=int(input(print("do you want to move to next midpoint or stay on this position? \n0 to stay and 1 to move ahead and 2 to end the mission: ")))
            if aa==1:
                pass
            if aa==2:
                drops=-1

            else:
                dd+=1
    
    if drops==0:
        bbh=int(input("mission ended, would you like to go to auto or rtl? 1 for auto 0 for rtl: "))
        if bbh == 1:
            vehicle.mode = "AUTO"
        else:
            vehicle.mode = "RTL"

def confirm_drop():
    #test for right pwm value for example 
    pwm=1900
    drop_channel=int(input("enter channel to set drop: "))
    aa=int(input("are you sure you want to drop? 1 for yes 0 for no: "))
    if aa =="1":
        move_servo(drop_channel, pwm)
        print("drop started...")
    else:
        pass 
        

if __name__== "__main__":

    watchpoints=[[]]

    vehicle = connect("tcp:192.168.100.18:14551", wait_ready=True)
    
    print("Connected to Cube")
    
    check_var3=1
    while (check_var3==1):
        if (vehicle.mode == "GUIDED"):
            print("In Search Area")
            drops=5
            dd=0
            midpoint_nav(drops,dd)
            check_var3=0
            bbh=int(input("mission ended, would you like to go to auto or rtl? 1 for auto 0 for rtl: "))
            if bbh == 1:
                vehicle.mode = "AUTO"
            else:
                vehicle.mode = "RTL"
            
        else:
            time.sleep(1)