import glob
import os
import sys
import random
import time
import numpy as np
import cv2
import math
import threading

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

IM_WIDTH = 640
IM_HEIGHT = 480

def get_actor_display_name(actor):
    name = ' '.join(actor.type_d.replace('_', '.').title().split('.')[1:])
    return name

actor_list = []


def image(image):
    matrix_representational_data = np.array(image.raw_data)
    reshape_of_image = matrix_representational_data.reshape((IM_HEIGHT, IM_WIDTH, 4))
    live_feed_from_camera = reshape_of_image[:, :, 3]
    cv2.imshow("", live_feed_from_camera)
    cv2.waitKey(1)
    return

def camera(get_blueprint_of_world):

    camera_sensor = get_blueprint_of_world.find('sensor.camera.rgb')
    camera_sensor.set_attribute('image_size_x', f'{IM_WIDTH}')
    camera_sensor.set_attribute('image_size_y', f'{IM_HEIGHT}')
    camera_sensor.set_attribute('fov', '70')
    return camera_sensor

def car_control():

    dropped_vehicle.apply_control(carla.VehicleControl(throttle = 0.52, steer = -1, gear = 0))
    time.sleep(5)

    dropped_vehicle.apply_control(carla.VehicleControl(throttle = 0.5, gear = 0))
    time.sleep(6)

    dropped_vehicle.apply_control(carla.VehicleControl(throttle = 0.5, steer = -0.17, gear = 0))
    time.sleep(2)

    dropped_vehicle.apply_control(carla.VehicleControl(throttle = 0.5, steer = 0.14, gear = 0))
    time.sleep(9)

    dropped_vehicle.apply_control(carla.VehicleControl(throttle = 0.4, steer = -0.25, gear = 0))
    time.sleep(1)

    dropped_vehicle.apply_control(carla.VehicleControl(throttle = 0.8, gear = 0))
    time.sleep(4)

    dropped_vehicle.apply_control(carla.VehicleControl(hand_brake = True))
    time.sleep(5)

    location = dropped_vehicle.get_location()

    return location

data = []
actor_list = []

def number_of_vehicle():
    all_vehicles = world.get_actors().filter('vehicle.*')
    threading.Timer(0.1, number_of_vehicle).start()
    transform_location = dropped_vehicle.get_transform()
    print("Location Of Each Bot Vehicle : " , transform_location)

    if len(all_vehicles) > 1:
        distance = lambda data: math.sqrt((data.x - transform.location.location.x) ** 2 + (data.y - transform_location.location.y) ** 2 + (data.z - transform_location.location.z) ** 2)
        
        get_distance_of_bot_vehicles = []
        for each_car in all_vehicles:
            if each_car.id != world.id:
                get_distance_of_bot_vehicled.append((distance(each_car.get_location()), each_car))

        print("Distance Of Every Vehicle : ", get_distance_of_bot_vehicle)

        vehicle_data = {}
        final_vehicle_result = []

        sorted_vehicles = sorted(get_distance_of_bot-_vehicles)

        for distance_of_car, vehicle in sorted_vehicles:
            if ditance_of_car > 200.0:
                break

            vehicle_type = get_actor_display_name(vehicle)
            vehicle_data['vehicle_name'] = vehicle_type
            vehicle_data['distance'] = distance_of_car
            final_vehicle_result.append(vehicle_data)
            print("Final Vehicle Result : ", final_vehicle_result)

            for distance_in_meter in final_vehicle_result:
                if distance_in_meter['distance'] > 3 and distance_in_meter['distance'] < 6:
                    dropped_vehicle.apply_control(carla.VehicleControl(hand_brake = True))
                    time_sleep(5)
                    car_control()
                    
try:
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    get_blueprint_of_world = world.get_blueprint_library()
    car_model = get_blueprint_of_world.filter('model3')[0]
    spawn_point =random.choice(world.get_map().get_spawn_points())
    dropped_vehicle = world.spawn_actor(car_model, spawn_point)

    simulator_camera_location_rotation = carla.Transform(spawn_point.location, spawn_point.rotation)
    simulator_camera_location_rotation.location += spawn_point.get_forward_vector() * 30
    simulator_camera_location_rotation.rotation.yaw += 180
    simulator_camera_view = world.get_spectator()
    simulator_camera_view.set_transform(simulator_camera_location_rotation)
    location = dropped_vehicle.get_location()
    print("Car Present Location : ", location)
    
    camera_sensor = camera(get_blueprint_of_world)
    sensor_camera_spawn_point = carla.Transform(carla.Location(x = 2.5, z = 0.7))
    sensor = world.spawn_actor(camera_sensor, sensor_camera_spawn_point, attach_to = dropped_vehicle)
    actor_list.append(sensor)
    sensor.listen(lambda camera_data : image(camera_data))

    car_new_location = car_control()
    print("Car New Location : ", car_new_location)

    actor_list.append(dropped_vehicle) 

    collision_sensor = get_blueprint_of_world.find('sensor.other.collision')
    sensor_collision_spawn_point = carla.Transform(carla.Location(x = 2.5, z = 0.7))
    sensor = world.spawn_actor(collision_sensor, sensor_collision_spawn_point, attach_to = dropped_vehicle)

    sensor.listen(lambda data: _on_collision(data))

    actor_list.append(sensor)

    def _on_collision():
        print("**Caution Collision**")
        actor_type = get_actor_display_name(data.other_actor)
        print("Collision with", actor_type)
        Collision_event_record = data.normal_impulse
        intensity_of_collision = math.sqrt(Collision_event_record.x ** 2 + Collision_event_record.y ** 2 + Collision_event_record.z ** 2)
        print("Intensity Of Collision", intensity_of_collision)
        dropped_vehicle.apply_control(carla.VehicleControl(hand_brake = True))
        time.sleep(5)
        car_control()

    number_of_vehicle()
    car_control()
    time.sleep(1000)

finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')