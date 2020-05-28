from neural_net_lib import *
import cv2
import time
import glob



images = glob.glob('./Images/*.jpg')
k = 0
for image in images:
    image = cv2.imread(image)
    objects_to_detect = ['sports ball']
    # cv2.imshow('original', image)
    start = time.time()
    img = object_detection_api(image, objects_to_detect, threshold=0.0)
    print(f'Time spent detecting {time.time()-start}')
    # cv2.imshow('target', img)
    cv2.imwrite(f'./Results/result_img_{k}.jpg', img)
    k += 1

