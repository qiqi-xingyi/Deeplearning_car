import os
import time
from ctypes import *

path = os.path.split(os.path.realpath(__file__))[0]+"/.."
lib_path = path + "/lib" + "/libart_driver.so"
so = cdll.LoadLibrary
lib = so(lib_path)
car = "/dev/ttyUSB0"

if (lib.art_racecar_init(38400, car.encode("utf-8")) < 0):
    raise
    pass

label_ten=0
label_xx=0
number = 0
detect =True
nowtime11 = time.time()
straight_a = 1500



#user_cmd函数用作控制模块，负责控制所有的电机动作
def user_cmd(state,labels,scores,center_x,center_y,pre_vel,pre_angle):

    global label_ten
    global label_xx
    global number
    global detect
    global nowtime11
    global crossing

    stop_sign = False  # 停车标志
    limit_speed = 1528
    crossing_sign = False
    limit_sign = False #限速标志


    if state==True:

        '''
            0	crossing
            1	ramp
            2	limit_10
            3	cancel_10

        '''
        print('labels' , labels)
        #labels存储所有检测到的目标标签

        print('scores:' , scores)
        #scores存储检测到目标的置信度


        if(detect == True):

            # print('scores:', scores[0])
            # print('scores:', scores[1])
            if (limit_sign == True):
                lib.send_cmd(limit_speed, pre_angle)

            for i in range(len(labels)):

######################################################################

                if(labels[i] == 0):   #停车标记被识别
                    if (crossing_sign == True):
                        lib.send_cmd(pre_vel, pre_angle)

                    else:
                        if(scores[i]>0.6):

                            if (number < 5):
                                number+=1 #检测目标次数
                                lib.send_cmd(pre_vel, pre_angle)


                            else:
                                lib.send_cmd(pre_vel, pre_angle)
                                number = 1
                                nowtime11 = time.time()
                                nowetime2 = time.time()
                                crossing_sign = True
                                while (time.time() - nowetime2 < 2):
                                    print("crossing stop")

                                    lib.send_cmd(1500 , 1500)

                        else:
                            lib.send_cmd(pre_vel, pre_angle)

                else:
                    lib.send_cmd(pre_vel, pre_angle)

######################################################################

                if (labels[i] == 1):  # 限速标记被识别
                    if (scores[i] > 0.6):

                        if (number < 15) :
                            number += 1  # 检测目标次数

                            lib.send_cmd(pre_vel, pre_angle)

                        else:
                            number = 1
                            limit_sign = True
                            lib.send_cmd(limit_speed , pre_angle)

                    else:
                        lib.send_cmd(pre_vel, pre_angle)

                else:
                    lib.send_cmd(pre_vel, pre_angle)

######################################################################

                if (labels[i] == 2):  # 取消限速被识别
                    if (scores[i] > 0.6):

                        if (number < 10):
                            number += 1  # 检测目标次数
                            lib.send_cmd(pre_vel, pre_angle)
                            # print("******************************number:",number)

                        else:
                            lib.send_cmd(pre_vel, pre_angle)
                            number = 1
                            limit_sign = False

                    else:
                        lib.send_cmd(pre_vel, pre_angle)

                else:
                    lib.send_cmd(pre_vel, pre_angle)

######################################################################

                if (labels[i] == 1):  # 坡道标记被识别

                    if (scores[i] > 0.6):
                        time_1 = time.time()
                        if(stop_sign == True):
                            lib.send_cmd(pre_vel, pre_angle)

                        else:
                            if (number < 75):
                                number += 1  # 检测目标次数
                                lib.send_cmd(pre_vel, pre_angle)
                                # print("******************************number:",number)


                            else:
                                time_2 = time.time()
                                if (time_2 - time_1 >= 9):
                                    lib.send_cmd(pre_vel, pre_angle)
                                    number = 1
                                    nowtime11 = time.time()
                                    nowetime2 = time.time()
                                    while (time.time() - nowetime2 < 1):
                                        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>ramp stop>>>>>>>>>>>>>>>>>>>>>>>>")
                                        stop_sign = True
                                        lib.send_cmd(1500, 1500)
                                else:
                                    lib.send_cmd(pre_vel, pre_angle)

                    else:
                        lib.send_cmd(pre_vel, pre_angle)

                else:
                    lib.send_cmd(pre_vel, pre_angle)


######################################################################


        elif(detect == False):
            if (time.time()-nowtime11 > 20.0):
                detect = True
            else:

                lib.send_cmd(pre_vel, pre_angle)
            lib.send_cmd(pre_vel, pre_angle)


        if (limit_sign == True):
            lib.send_cmd(limit_speed, pre_angle)


    else:

        lib.send_cmd(pre_vel, pre_angle)

        

