#2021.10.30 zyq

import os
import v4l2capture
from ctypes import *
import struct, array
from fcntl import ioctl
import cv2
import numpy as np
import time
from sys import argv
import getopt
import sys, select, termios, tty
import threading
import paddlemobile as pm
from paddlelite import *
import codecs
#import paddle
import multiprocessing
#import paddle.fluid as fluid
#from IPython.display import display
import math
import functools
from PIL import Image
from PIL import ImageFile
ImageFile.LOAD_TRUNCATED_IMAGES = True
from PIL import ImageFont
from PIL import ImageDraw
from collections import namedtuple
from datetime import datetime
from Q_user import user_cmd
from Q_user import user_detect

path = os.path.split(os.path.realpath(__file__))[0]+"/.."
opts,args = getopt.getopt(argv[1:],'-hH',['save_path=','vels=','camera='])

camera = "/dev/video2"
camera_2 = "/dev/video3"

save_path = 'model_infer'

crop_size = 128


def dataset(video):  # 图片处理
    lower_hsv = np.array([25, 75, 190])
    upper_hsv = np.array([40, 255, 255])

    select.select((video,), (), ())
    image_data = video.read_and_queue()
    frame = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    '''load  128*128'''
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)
    img = Image.fromarray(mask)  # 把array转化成image
    img = img.resize((128, 128), Image.ANTIALIAS)
    # img = cv2.resize(img, (128, 128))
    img = np.array(img).astype(np.float32)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img = img / 255.0
    img = np.expand_dims(img, axis=0)
    # print("image____shape:",img.shape)
    '''object   256*256'''
    Original_image = Image.fromarray(frame)
    # cv2.imwrite("test_up.jpg", frame)
    return frame , img


def dataset_2(video):
    select.select((video,), (), ())
    image_data = video.read_and_queue()
    Original_image  = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    # Original_image = Image.fromarray(frame)
    # cv2.imwrite("test_down.jpg", frame)
    return Original_image

def load_model():                                    #车道线
    valid_places = (
		Place(TargetType.kFPGA, PrecisionType.kFP16, DataLayoutType.kNHWC),
		Place(TargetType.kHost, PrecisionType.kFloat),
		Place(TargetType.kARM, PrecisionType.kFloat),
	);

    config = CxxConfig();

    model_dir = save_path;

    config.set_model_file(model_dir + "/model");
    config.set_param_file(model_dir + "/params");

    config.set_valid_places(valid_places);
    predictor = CreatePaddlePredictor(config);     #predictor是paddle创建的推理器

    return predictor;


def predict(predictor, image, z):

    img = image;

    i = predictor.get_input(0);
    i.resize((1, 3, 128, 128));
    # print("****************************img",img.shape)
    # print("****************************img",z.shape)
    z[ 0,0:img.shape[1], 0:img.shape[2] + 0, 0:img.shape[3]] = img
    z = z.reshape(1, 3, 128, 128);
    frame1 = cv2.imdecode(np.frombuffer(img, dtype=np.uint8), cv2.IMREAD_COLOR)

    i.set_data(z)

    predictor.run();
    out = predictor.get_output(0);
    score = out.data()[0][0];
    # print(out.data()[0])
    return score;


if __name__ == "__main__":

    cout = 0  # 定义计数器
    Time = 0  # 定义计时器
    save_path = path + "/model/" + save_path  # 设置模型读取路径

    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>初始化视频流<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    video = v4l2capture.Video_device(camera)
    video_2 = v4l2capture.Video_device(camera_2)

    video.set_format(424, 240, fourcc='MJPG')
    video.create_buffers(1)
    video.queue_all_buffers()
    video.start()

    video_2.set_format(424, 240, fourcc='MJPG')
    video_2.create_buffers(1)
    video_2.queue_all_buffers()
    video_2.start()

    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>加载模型，串口<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    predictor = load_model()
    lib_path = path + "/lib" + "/libart_driver.so"
    so = cdll.LoadLibrary
    lib = so(lib_path)
    car = "/dev/ttyUSB0"

    z = np.zeros((1, 128, 128, 3))

    if (lib.art_racecar_init(38400, car.encode("utf-8")) < 0):
        # raise
        pass

    mission_num = 1 #任务计数器

    template = cv2.imread("./template/template.jpg")

    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主循环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while 1:
        massion_count = 0

        start_time = time.time()

        label_img = dataset_2(video_2)  # label_img为底部摄像头返回的图片，用作识别标志物

        Original_image , img = dataset(video)  # img为上部摄像头返回图片，用作车道线检测

        # >>>>>>>>>>>>>>>>>>>>>>车道线检测，返回车道线偏差<<<<<<<<<<<<<<<<<<<<<<
        error = predict(predictor, img, z)

        ############################################PID控制####################################################

        angle = int(error * 600 + 1200)

        # # 转弯幅度
        # p_left = 4
        # p_right = 4
        #
        # min_limit = 1500
        # max_limit = 1500
        #
        # if (a >= min_limit and a <= max_limit):
        #     a = 1500
        #
        # if (a > max_limit):
        #     up = p_left * (a - max_limit)
        #     a = a + up
        # elif (a < min_limit):
        #     up = p_right * (min_limit - a)
        #     a = a - up
        #
        # if (a > 3000):
        #     a = 3000
        # elif (a < 0):
        #     a = 0

        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>交通标志<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        #mission_num = 1

        label , mission_count , limit_speed_time = user_detect(mission_num , label_img , Original_image  , Time)

        mission_num = mission_num + mission_count #任务计数器

        Time = limit_speed_time

        # label = user_detect(label_img, Original_image)

        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>进入速度控制 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        # vel = 1540
        # speed = int(vel) # 速度 1570
        # a = a
        user_cmd(label , angle)

        print(cout)  # 计数器
        cout = cout + 1

        print(">>>>>>>>>>>>>>>>>>>>>>>> angle and speed <<<<<<<<<<<<<<<<<<<<<<<")
        # print("a: %d , speed: %d  " % (label , angle))
