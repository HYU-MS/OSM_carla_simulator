import carla
import cv2
import threading
import numpy as np
import time
import math
import serial
import time
import random

ser = serial.Serial('COM13', 115200) # Replace 'COM3' with your Arduino's serial port name
time.sleep(3) # Wait for the serial connection to initialize

ser.write(b'7')
time.sleep(2)
ser.write(b'8')
time.sleep(2)


# Connect to the CARLA server
client = carla.Client('localhost', 2000)

world = client.get_world()
actors = world.get_actors().filter('vehicle.*')

traffic_manager = client.get_trafficmanager(8000)
traffic_manager.global_percentage_speed_difference(20.0)
traffic_manager.set_global_distance_to_leading_vehicle(2)

for actor in actors:
    actor.destroy()
# spawn location
spawn_points = world.get_map().get_spawn_points()
specific_spawn_point = spawn_points[0]
specific_spawn_point.location.x = -41.5
specific_spawn_point.location.y = 114
specific_spawn_point.rotation.yaw = -90.0

# set car
vehicle_bp = world.get_blueprint_library().find('vehicle.mini.cooper_s_2021')
vehicle = world.try_spawn_actor(vehicle_bp, specific_spawn_point)

# set traffic lights duration
lights = world.get_actors().filter('traffic.traffic_light')
for light in lights:
    light.set_state(carla.TrafficLightState.Green)
    light.set_green_time(10000.0)
#     # light.set_state(carla.TrafficLightState.Red)
#     light.set_red_time(7.0)
#     light.set_yellow_time(3.0)

# generate traffic
blueprint_library = world.get_blueprint_library()
cars = ['vehicle.tesla.model3', 'vehicle.audi.etron',  'vehicle.mercedes.coupe', 'vehicle.mercedes.coupe_2020',
        'vehicle.carlamotors.carlacola', 'vehicle.nissan.patrol_2021', 'vehicle.mercedes.coupe_2020', 'vehicle.mercedes.sprinter', 'vehicle.lincoln.mkz_2020', 'vehicle.nissan.patrol_2021', 'vehicle.carlamotors.carlacola',
        'vehicle.toyota.prius', 'vehicle.nissan.patrol_2021', 'vehicle.tesla.model3', 'vehicle.mercedes.coupe_2020', 
        'vehicle.mercedes.sprinter', 'vehicle.lincoln.mkz_2020', 'vehicle.nissan.patrol_2021', 'vehicle.mercedes.coupe',
        'vehicle.mercedes.sprinter', 'vehicle.mitsubishi.fusorosa', 'vehicle.audi.a2', 'vehicle.nissan.micra', 'vehicle.bmw.grandtourer',
        'vehicle.chevrolet.impala', 'vehicle.citroen.c3', 'vehicle.mercedes.sprinter',
        'vehicle.lincoln.mkz_2017', 'vehicle.toyota.prius',
        'vehicle.audi.etron', 'vehicle.audi.etron', 'vehicle.audi.etron',
        'vehicle.toyota.prius', 'vehicle.tesla.model3', 'vehicle.mercedes.coupe_2020', 'vehicle.mercedes.sprinter',
        'vehicle.lincoln.mkz_2020', 'vehicle.nissan.patrol_2021']

car_blueprints = [blueprint_library.find(car) for car in cars]
# Set the number of cars you want to spawn
num_of_cars = 22

# List to hold all spawned car actors
car_actors = []

#Spawn the cars
for i in range(num_of_cars):
    car_bp = car_blueprints[i]
    # Spawn the car
    car_actor = world.spawn_actor(car_bp, spawn_points[7*i+1])
    
    # Add the car actor to the list
    car_actors.append(car_actor)


# Camera mount offsets on the car - adjust as needed
LEFT_CAMERA_POS_X = 0.4  # 0.2m to the left (assuming the car's forward direction)
RIGHT_CAMERA_POS_X = 0.4  # 0.2m to the right (assuming the car's forward direction)
CAMERA_POS_Z = 1.25  # 1.4m up from the ground

# Set up the driver view camera
driver_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
driver_camera_bp.set_attribute('image_size_x', '640')  # Adjust the resolution as desired
driver_camera_bp.set_attribute('image_size_y', '360')

driver_camera_transform = carla.Transform(carla.Location(x=-0.05, y=-0.3, z=1.35))
driver_camera = world.spawn_actor(driver_camera_bp, driver_camera_transform, attach_to=vehicle)

# Set up the left side mirror camera
left_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
left_camera_bp.set_attribute('image_size_x', '640')  # Adjust the resolution as desired
left_camera_bp.set_attribute('image_size_y', '360')

left_camera_transform = carla.Transform(
    carla.Location(x=LEFT_CAMERA_POS_X, y=-1.02, z=CAMERA_POS_Z),
    carla.Rotation(yaw=180))
left_camera = world.spawn_actor(left_camera_bp, left_camera_transform, attach_to=vehicle)

# Set up the right side mirror camera
right_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
right_camera_bp.set_attribute('image_size_x', '640')  # Adjust the resolution as desired
right_camera_bp.set_attribute('image_size_y', '360')
right_camera_transform = carla.Transform(
    carla.Location(x=RIGHT_CAMERA_POS_X, y=1.02, z=CAMERA_POS_Z),
    carla.Rotation(yaw=180))
right_camera = world.spawn_actor(right_camera_bp, right_camera_transform, attach_to=vehicle)

# Camera data dictionaries
driver_camera_data = {'image': None}
left_camera_data = {'image': None}
right_camera_data = {'image': None}

# Callback functions for camera data
def driver_camera_callback(image):
    driver_camera_data['image'] = np.reshape(np.array(image.raw_data), (image.height, image.width, 4))

def left_camera_callback(image):
    left_camera_data['image'] = np.reshape(np.array(image.raw_data), (image.height, image.width, 4))

def right_camera_callback(image):
    right_camera_data['image'] = np.reshape(np.array(image.raw_data), (image.height, image.width, 4))

# Set up camera callbacks
driver_camera.listen(driver_camera_callback)
left_camera.listen(left_camera_callback)
right_camera.listen(right_camera_callback)

# Display function for each camera
def display_camera_stream(camera_data, window_name):
    while True:
        if camera_data['image'] is not None:
            # Display the camera view using imshow
            cv2.imshow(window_name, camera_data['image'])
        if cv2.waitKey(1) == ord('q'):
            break

# Create separate threads for displaying camera views
driver_display_thread = threading.Thread(target=display_camera_stream, args=(driver_camera_data, 'Driver View'))
left_display_thread = threading.Thread(target=display_camera_stream, args=(left_camera_data, 'Left Side View Mirror'))
right_display_thread = threading.Thread(target=display_camera_stream, args=(right_camera_data, 'Right Side View Mirror'))

# Start the display threads
driver_display_thread.start()
left_display_thread.start()
right_display_thread.start()

def update_side_mirror_angle():
    global detect_signal, turn_signal, comeback_signal, comeback_steers, steering_reference, comeback_authorized
    entered = False
    detect_signal = False
    comeback_steers = 0
    comeback_signal = False
    comeback_authorized = False
    turn_signal = False
    steering_reference = 0
    steer_previous = 0
    detect_off_count = 0
    ser.write(b'1')
    while True:
        steering_angle = round(vehicle.get_control().steer, 3)

        if detect_signal:
            if steering_angle != steer_previous:
                if abs(steering_angle) < 0.01:
                    detect_off_count += 1
                    if detect_off_count == 12:
                        ser.write(b'1')
                        print("Detect Algorithm Deactivated")
                        detect_signal = False
                        detect_off_count = 0
                        steer_previous = steering_angle
                        time.sleep(0.001)
                        continue

        steer_previous = steering_angle

        if turn_signal:
            rotate_algorithm()
            time.sleep(0.001)
            continue
        elif comeback_signal:
            velocity = vehicle.get_velocity()
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            if comeback_authorized:
                comeback_algorithm(steering_angle)
            elif speed >= 4:
                comeback_authorized = True
            time.sleep(0.001)
            continue
        if detect_signal and abs(steering_angle) >= 0.1:
            steering_reference = steering_angle
            turn_signal = True
            detect_off_count = 0
            detect_signal = False
            print("Rotation Algorithm Activated")
            continue

        location  = vehicle.get_location()
        x = location.x
        y = location.y

        #1
        if -87 <= x <= -77 and 11 <= y <= 14:
            entered = True
            # print('111111111')
        #2
        elif -87 <= x <= -77 and 15.5 <= y <= 18:
            entered = True
            # print('22222222')
        #3
        elif -53.5 <= x <= -51.8 and -13 <= y <= -3:
            entered = True
            # print('33333333')
        #4
        elif -49.8 <= x <= -48 and -13 <= y <= -3:
            entered = True
            # print('44444444')
        #5 
        elif -27 <= x <= -17 and 12 <= y <= 14:
            entered = True
            # print('555555555')
        #6
        elif -27 <= x <= -17 and 16 <= y <= 18:
            entered = True
            # print('666666666')
        #7
        elif -42.5 <= x <= -40 and 44 <= y <= 51:
            entered = True
            # print('777777777')
        #8
        elif -46 <= x <= -43.3 and 44 <= y <= 51:
            entered = True
            # print('888888888')
        #9
        elif -42.5 <= x <= -40 and -41 <= y <= -31:
            entered = True
            # print('999999999')
        #10
        elif -46 <= x <= -44.5 and -41 <= y <= -31:
            entered = True
            # print('10.10.10.10.10')
        #11
        elif -53 <= x <= -47.8 and 102 <= y <= 112:
            entered = True
            # print('11.11.11.11.11')
        #12
        elif -25 <= x <= -15 and 65 <= y <= 67:
            entered = True
            # print('12.12.12.12.12')
        #13
        elif 43 <= x <= 45 and 40 <= y <= 46:
            entered = True
            # print('13.13.13.13.13')
        #14
        elif 39 <= x <= 41 and 47 <= y <= 53:
            entered = True
            # print('14.14.14.14.14')
        #15
        elif 71 <= x <= 81 and 27.8 <= y <= 30:
            entered = True
            # print('15.15.15.15.15')
        #16
        elif 71 <= x <= 81 and 23.5 <= y <= 26:
            entered = True
            # print('16.16.16.16.16')
        #17
        elif 76 <= x <= 86 and 69 <= y <= 71:
            entered = True
            # print('17.17.17.17.17')
        #18
        elif -76 <= x <= -66 and 27 <= y <= 29:
            entered = True
            # print('18.18.18.18.18')
        #19
        elif -76 <= x <= -66 and 23 <= y <= 25.3:
            entered = True
            # print('19.19.19.19.19')           
        else:
            if entered:
                ser.write(b'2')
                print("Detect Algorithm Activated")
                detect_signal = True
                entered = False
                continue
            # print('NNNNNNNNNN')

        time.sleep(0.001)

def rotate_algorithm():
    global comeback_signal, steering_reference, turn_signal
    if steering_reference > 0:
        if round(left_camera_transform.rotation.yaw) != 205:
            left_camera_transform.rotation.yaw += 0.19
            ser.write(b'3')
        else:
            turn_signal = False
            comeback_signal = True
            left_camera_transform.rotation.yaw = round(left_camera_transform.rotation.yaw)
            left_camera.set_transform(left_camera_transform)
            return
        left_camera.set_transform(left_camera_transform)

    elif steering_reference < 0:
        if round(right_camera_transform.rotation.yaw) != 155:
            right_camera_transform.rotation.yaw -= 0.19
            ser.write(b'5')
        else:
            turn_signal = False
            comeback_signal = True
            right_camera_transform.rotation.yaw = round(right_camera_transform.rotation.yaw)
            right_camera.set_transform(right_camera_transform)
            return
        right_camera.set_transform(right_camera_transform)

def comeback_algorithm(steering_angle):
    global comeback_signal, comeback_steers, steering_reference, comeback_authorized

    if comeback_steers >= 3:
        if steering_reference > 0:
            if round(left_camera_transform.rotation.yaw) != 180:
                left_camera_transform.rotation.yaw -= 0.5
                ser.write(b'4')
            else:
                ser.write(b'1')
                comeback_signal = False
                comeback_authorized = False
                comeback_steers = 0
            left_camera.set_transform(left_camera_transform)

        elif steering_reference < 0:
            if round(right_camera_transform.rotation.yaw) != 180:
                right_camera_transform.rotation.yaw += 0.5
                ser.write(b'6')
            else:
                ser.write(b'1')
                comeback_signal = False
                comeback_authorized = False
                comeback_steers = 0
            right_camera.set_transform(right_camera_transform)   

    if abs(steering_angle) < 0.1:
        comeback_steers += 1
    else:
        comeback_steers = 0
    time.sleep(0.001) 


update_angle = threading.Thread(target=update_side_mirror_angle)

update_angle.start()


time.sleep(3)
for car in car_actors:
    car.set_autopilot(True)

# Enable autopilot for the vehicle
vehicle.set_autopilot(True)
