from neural_net_lib import *
import paho.mqtt.client as mqtt
import cv2
import glob
import time
from change_name import *
from math import floor
from playsound import playsound
import threading

def play_cloud_processing():
    playsound('cloud.mp3')
    
while True:
    client = mqtt.Client("sally_ball_detect")

    files = glob.glob('./Images/frame.jpg')

    if len(files) > 0:
        time.sleep(3.0)
        start = time.time()
        client.connect('192.168.1.149', port=1883, keepalive=15)
        file = files[0]
        img = cv2.imread(file)
        change_name()
        balls = give_box_corners_of_ball(img)
        print(f'Number of balls {len(balls)}')
        name_corners = ['c_upp_l:', 'c_upp_r:', 'c_low_r:', 'c_low_l:']
        corner_arrange = [0, 3, 2, 1]
        message = ''
        for corners in balls:
            message = message + name_corners[0] + str(corners[0]) + '\n'
            message = message + name_corners[1] + str(corners[3]) + '\n'
            message = message + name_corners[2] + str(corners[2]) + '\n'
            message = message + name_corners[3] + str(corners[1]) + '\n'


        try:
            client.publish("sally/Corners", message, qos=1)
            print("Sending calculated corners")
        except:
            client.publish("sally/Corners", f'c_upp_l:({10}, {10})\n'
                                            f'c_upp_r:({10}, {10})\n'
                                            f'c_low_r:({10}, {10})\n'
                                            f'c_low_l:({10}, {10})\n', qos=1)
            print("Sending empty coorindates")

        print("New file     here")
        print(f'Time spent processing and sending: {time.time() - start} seconds')
        objects_to_detect = ['sports ball']
        img = object_detection_api(img, objects_to_detect, threshold=0.001)
        files = glob.glob('./Results/*.jpg')
        num = highest_num_file(files)
        cv2.imwrite(f'./Results/result_img_{num+1}.jpg', img)

