from machine import Pin, I2C
import time
import math
import network
from simple import MQTTClient
from secret import WIFI_SSID, WIFI_PASSWORD, MQTT_SERVER, MQTT_PORT, USERNAME, PASSWORD
from l298n import L298N
from hc020k import HC020K
from hcsr04 import HCSR04
from qmc5883l import QMC5883L
from imu import MPU6050

# Constants
DISTANCE_PER_POINT = 5		# cm
MAX_RANGE = 300 			# cm
GYRO_WEIGHT = 0.85
MAG_WEIGHT = 0.15
ROTATION_TOLERANCE = 1.0 	# degrees
PULSES_PER_ROTATION = 20
WHEEL_DIAMETER = 6.5 		# cm

# Motor driver
motor = L298N(21, 20, 19, 18, 17, 16)

# Motor speed encoder
encoder = HC020K(2, PULSES_PER_ROTATION, WHEEL_DIAMETER)

# Ultrasonic sensors
ultra_front = HCSR04(trigger_pin=15, echo_pin=14)
ultra_right = HCSR04(trigger_pin=13, echo_pin=12)
ultra_left = HCSR04(trigger_pin=11, echo_pin=10)


# Compass and IMU
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)
compass = QMC5883L(i2c)
imu = MPU6050(i2c)	# both work on different addresses - can be together on one I2C

# Global variables
angle = 0.0			# degrees
position_x = 0.0	# cm
position_y = 0.0	# cm
motor_speed = 100 	# % of max speed
prev_right = None
running = False
manual = False

def process_distance(distance):
    """
    Function takes the measured distance and checks if it is bigger than MAX_RANGE.
    If true - don't use the data in data processing later (set as '#').
    
    Params:
        distance: distance measured by the sensor
    Returns:
        Values as string
    """
    return "#" if distance >= MAX_RANGE else f"{distance:.2f}"

def read_ultra():
    """
    Function for reading values from ultrasonic sensors.
    
    Returns:
        Distances measured from the center of the car
    """
    front = ultra_front.distance()
    right = ultra_right.distance()
    left = ultra_left.distance()
    
    front = process_distance(front + 10)
    right = process_distance(right + 5)
    left = process_distance(left + 5)
    
    return front, right, left

def read_compass():
    """
    Function for reading heading angle form compass.
    
    Returns:
        Angle the car is heading
    """
    x, y, _ = compass.read()
    return compass.heading(x, y)

def read_imu():
    """
    Function for reading current gyro rotation speed and accel value.
    
    Returns:
        Gyro and accel values
    """
    return imu.gyro, imu.accel

def update_position(distance):
    """
    Function updates the global position of the car.
    
    Params:
        distance: distance the car has traveled
    """
    global position_x, position_y, angle
    
    dx = distance * math.sin(math.radians(angle))
    dy = distance * math.cos(math.radians(angle))
    
    position_x += dx
    position_y += dy

def calculate_points(front, right, left):
    """
    Function calculates the points that are sent to the server.
    
    Params:
        front: distance measured at the front
        right: distance measured at the right
        left: distance measured at the left
        
    Returns:
        Points on the final map
    """
    global position_x, position_y, angle
    
    # calculate front point
    if front == "#":			# if the dsitance is # (>300), send point '#'
        front_point = "#"
    else:						# else measure the point
        front = float(front)
        front_dx = front * math.sin(math.radians(angle))
        front_dy = front * math.cos(math.radians(angle))
        front_point = f"{position_x + front_dx},{position_y + front_dy}"
    
    # calculate right point
    if right == "#":
        right_point = "#"
    else:
        right = float(right)
        right_dx = right * math.sin(math.radians(angle) + math.pi / 2)
        right_dy = right * math.cos(math.radians(angle) + math.pi / 2)
        right_point = f"{position_x + right_dx},{position_y + right_dy}"
    
    # calculate left point
    if left == "#":
        left_point = "#"
    else:
        left = float(left)
        left_dx = left * math.sin(math.radians(angle) - math.pi / 2)
        left_dy = left * math.cos(math.radians(angle) - math.pi / 2)
        left_point = f"{position_x + left_dx},{position_y + left_dy}"
    
    return front_point, right_point, left_point
    
def turn_by_angle(target_angle):
    """
    Function for turning the car by desired angle.
    
    Params:
        target_angle: angle the car is about to turn by
    """
    global angle
    
    compass.offset = read_compass() 	# set offset to start from 0
    motor.set_speed(80, 80)				# set the speed to lower for more accuracy
    
    error = 0.0
    prev_gyro_yaw = 0.0					# gyro yaw rotation measured last loop
    prev_time = time.ticks_us()			# time measured last time to count angle with gyro
    
    while not target_reached:
        gyro, _ = read_imu()			# read gyro from imu
        current_time = time.ticks_us()
        dt = time.ticks_diff(current_time, prev_time) / 1000000	# time diff
        
        gyro_z = -gyro.z - 2.035 		# calibration value
        
        gyro_yaw_change = gyro_z * dt	# angle turned by from last loop

        current_gyro_yaw = prev_gyro_yaw + gyro_yaw_change
        current_mag_yaw = read_compass()
        
        current_angle = MAG_WEIGHT * current_mag_yaw + GYRO_WEIGHT * current_gyro_yaw
        error = (target_angle - current_angle + 180) % 360 - 180
        
        if abs(error) < ROTATION_TOLERANCE:  		# Toleration is 1 degree
            break
        elif abs(error) < 45:
            rotation_speed = (0.4 * abs(error) + 60) % 100 	# minimum is 60 or it will not run
            motor.set_speed(rotation_speed, rotation_speed)
        
        if error > 0:
            motor.right()
        else:
            motor.left()

        prev_gyro_yaw = current_gyro_yaw		# update prev values
        prev_time = current_time
    
    motor.stop()
    motor.set_speed(motor_speed, motor_speed)	# set speed back to the set values
    compass.offset = 0 							# reset offset for another calculations
    
    # publish the status to the server
    client.publish("car/status", f"Rotation done! Rotated by: {str(target_angle + error)}")
    angle += (target_angle + error)				# update global angle

def drive_forward(target_distance):
    """
    Functions for driving the car by desired distance using wheel encoders.
    
    Params:
        target_distance: distance the car is about to drive
    """
    motor.set_speed(motor_speed, motor_speed)	# make sure the speed is set
    
    encoder.start()								# start measuring the pulses
    motor.forward()
    driven_distance = encoder.distance_cm()		# read the current driven distance
    
    while driven_distance < target_distance:
        driven_distance = encoder.distance_cm()	# read until target reached
        
    encoder.stop()	# stop measuring (will reset the pulses automaticaly)
    motor.stop()
    
    update_position(driven_distance)	# update global position
    
    # publish the status to the server
    client.publish("car/status", f"Went forward by: {driven_distance} cm")
    
def angle_correction(dist1, dist2):
    """
    Funtion for correction angle if not going with the wall in finding wall funtion.
    
    Params:
        dist1, dist2: two distances with which the angle is calculated
    """
    turn_angle = math.atan((float(dist2) - float(dist1))/10)
    turn_by_angle(turn_angle)	# turn by the correction angle

def adjust_right_distance(right):
    """
    Funtion for adjusting the right distance from the wall the car is following.
    
    Params:
        right: right distance from the car
    """
    global prev_right

    if float(right) < 10: 	# Car is too close to the wall
        turn_by_angle(-90)
        time.sleep(0.1)
        drive_forward(10)
        turn_by_angle(90)
        time.sleep(0.1)
        _, right, _ = read_ultra()
        prev_right = right
            
    elif float(right) > 40:	# Car is too far from the wall
        turn_by_angle(90)
        time.sleep(0.1)
        drive_forward(10)
        turn_by_angle(-90)
        time.sleep(0.1)
        _, right, _ = read_ultra()
        prev_right = right
    
    else:
        prev_right = right

def find_wall():
    """
    Function to find wall at the start of the whole algorithm.
    """
    global prev_right
    
    found_right_wall = False
    
    while True:
        front, right, _ = read_ultra()
        
        # check front distance
        if front != "#" and float(front) <= 25:	# if we are too close to the wall, turn left
            turn_by_angle(-90)
            time.sleep(0.1)
            _, right, _ = read_ultra()	# measure new right distance for global variable
            prev_right = right
            break
        
        # else trying right distance
        elif not found_right_wall and right != "#" and float(right) < 25:
            found_right_wall = True	# if we are close to the wall on right, we found one 
            prev_right = right
        
        elif found_right_wall:
            if right == "#" or (right != "#" and abs(float(prev_right) - float(right)) >= 10):
                found_right_wall = False	# right is further than previous, right wall not found
                
            elif right != "#" and float(right) < 25:
                angle_correction(prev_right, right)	# wall really found - correct the angle
                prev_right = right
                break
        
        drive_forward(DISTANCE_PER_POINT)
        
def find_corner():
    """
    Function for algorithm, that finds corner.
    """
    global prev_right
    
    front, _, _ = read_ultra()
    
    # check front distance
    if front == "#" or float(front) > 25:	# if there is enough space, go forward by 10 cm
        drive_forward(10)
        
    else:									# else don't bother going right and go left
        if float(front) > 15:
            drive_forward(float(front) - 15)
            
        turn_by_angle(-90)
        time.sleep(0.1)
        _, right, _ = read_ultra()
        
        prev_right = right
        return
    
    turn_by_angle(90)	# turn right
    time.sleep(0.1)
    
    front, _, _ = read_ultra()
    
    # check front distance if there is space
    if front == "#" or float(front) > float(prev_right) + 25:
        drive_forward(float(prev_right) + 10)	# go to reach right wall
        _, right, _ = read_ultra()
        
        if right != "#" and float(right) <= 40:
            prev_right = right
                
        else:
            find_corner()	# we did not find new wall, find new corner
    
    # try to go as much as possible forward
    elif float(front) > 15:
        drive_forward(float(front) - 15)
        _, right, _ = read_ultra()
        
        turn_by_angle(-90)	# turn left
        time.sleep(0.1)
        _, right, _ = read_ultra()
        
        prev_right = right
        
    else:
        turn_by_angle(-90)	# turn left
        time.sleep(0.1)
        
        _, right, _ = read_ultra()
        prev_right = right
    
def wall_following():
    """
    Function for wall following algorithm
    """
    global prev_right
        
    front, right, left = read_ultra()											# read distances
    
    front_point, right_point, left_point = calculate_points(front, right, left)	# process distances
        
    # publish the data to the server
    client.publish("car/data", f"{front_point};{right_point};{left_point}")
    
    # check front distance
    if front != "#" and float(front) < 30:	# if short, turn left
            turn_by_angle(-90)				
            time.sleep(0.1)
            _, _, right = read_ultra()
            prev_right = right 
        
    # right wall following logic
    elif right == "#" or abs(float(right) - float(prev_right)) > 5:
        find_corner()
        
    else:
        adjust_right_distance(right)
    
    drive_forward(DISTANCE_PER_POINT)	# drive by distance per point

def sub_cb(topic, msg):
    """
    Function for recieving messages from server.
    
    Params:
        topic: topic with which we determinate what msq could we get
        msq: message sent with the topic - determinates what to do
    """
    global running, manual
    
    print("TOPIC:", topic, "MESSAGE:", msg)
    
    # control - controls major car functions
    if topic == b"car/control":
        if msg == b"start":
            if not running:
                client.publish("car/status", "STARTED")
                time.sleep(1)
                client.publish("car/status", "Finding a wall...")
                find_wall()
                client.publish("car/status", "Wall founded!")
                time.sleep(0.5)
                client.publish("car/status", "Starting a algorithm.")
                time.sleep(0.5)
            else:
                client.publish("car/status", "Already started.")
                
            running = True
            
        elif msg == b"stop":
            motor.stop()
            running = False
            client.publish("car/status", "STOPPED")
        
        elif msg == b"manual_on":
            motor.stop()
            running = False
            client.publish("car/status", "STOPPED")
            manual = True
            client.publish("car/status", "Started Manual Control.")
        
        elif msg == b"manual_off":
            motor.stop()
            manual = False
            client.publish("car/status", "Ended Manual Control.")
    
    # manual_control - controls car manualy
    elif topic == b"car/manual_control" and manual:
        motor.set_speed(motor_speed, motor_speed)
        if msg == b"forward":
            motor.forward()
        elif msg == b"backward":
            motor.backward()
        elif msg == b"left":
            motor.left()
        elif msg == b"right":
            motor.right()
        elif msg == b"stop":
            motor.stop()
    
    # speed - changes motor_speed
    elif topic == b"car/speed":
        try:
            motor_speed = int(msg.decode()) * 0.4 + 60	# not less than 60 - would not run
            client.publish("car/status", f"Speed set to {motor_speed}")
        except Exception as e:
            client.publish("car/status", f"Failed to set speed: {str(e)}")

def initialize_network_connection():
    """
    Function to initialize network connection
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)	# variables from secret
    while wlan.isconnected() == False:
        print("Waiting for connection...")
        time.sleep(1)
    print("Connected to Wifi \n")

def setup_mqtt_client():
    """
    Function to initialize connection with MQTT server
    """
    # MQTT client
    client = MQTTClient(
        client_id="MAPPING_ROBOT_1",
        server=MQTT_SERVER,
        port=MQTT_PORT,
        keepalive=60
    )
    client.set_callback(sub_cb)

    while True:
        # try to connect and subscribe to topics
        try:
            print("Trying to connect to MQTT server...")
            client.connect()
            client.subscribe("car/control")
            client.subscribe("car/manual_control")
            client.subscribe("car/speed")
            print("Connected to MQTT \n")
            client.publish("car/status", f"Car is connected to the server!")
            return client
        except Exception as e:
            print("Failed to connect to MQTT:", e, "\n")
            time.sleep(1)

initialize_network_connection()
client = setup_mqtt_client()

while True:
    try:
        client.check_msg()
        if running:
            wall_following()
                
    except OSError as e:
        print("MQTT Error:", e)
        client.disconnect()
        time.sleep(1)
        client = setup_mqtt_client()