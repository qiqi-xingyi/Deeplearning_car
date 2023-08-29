import os
import time
from ctypes import *
import cv2 as cv
import numpy as np

path = os.path.split(os.path.realpath(__file__))[0]+"/.."
lib_path = path + "/lib" + "/libart_driver.so"
so = cdll.LoadLibrary
lib = so(lib_path)
car = "/dev/ttyUSB0"

if (lib.art_racecar_init(38400, car.encode("utf-8")) < 0):
    # raise
    pass

#任务标志位 0x01 , 0x02 , 0x03 , 0x04 , 0x05 , 0x06
#路段标志  0x0A , 0x0B , 0x0C , 0x0D , 0x0E , 0x0F


def user_detect(mission , label_img , Original_image , Time):

    label = '0x00'
    #通常情况进0x0A

    mission_count = 0
    return_time = 0

    #任务轴
    # #按任务段分区
    if int(mission) == 1 :
        #进入任务一
        #需要在人行道前停1秒

        lower_hsv = np.array([0, 10, 200])
        upper_hsv = np.array([180, 30, 255])
        # 二值化处理
        hsv = cv.cvtColor(Original_image , cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)

        kernel_1 = np.ones((2, 2), np.uint8)  # 定义卷积核

        erosion = cv.erode(mask, kernel_1, iterations=3)

        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>膨胀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        kernel_2 = np.ones((2, 2), np.uint8)

        dilation = cv.dilate(erosion, kernel_2, iterations=4)

        # 图片剪裁
        cropped = mask[220:235, 30:50]  # 裁剪坐标为[y0:y1, x0:x1]
        detect_flag = mask[150, 80]     #取一个像素点，判断二值化之后的值 白线[160,40]
        # detect_flag为255则说明检测到白线

        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>新方案 利用人行道标志读取绿色区域<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        # img = label_img
        # cropped = img[40:110, 220:300]
        #
        # lower_hsv = np.array([35, 43, 46])
        # upper_hsv = np.array([77, 255, 255])  # 绿色阈值
        #
        # hsv = cv.cvtColor(cropped, cv.COLOR_BGR2HSV)
        # mask = cv.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)
        #
        # #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>适当腐蚀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        #
        # kernel_1 = np.ones((2, 2), np.uint8)  # 定义卷积核
        #
        # erosion = cv.erode(mask, kernel_1, iterations=3)
        #
        # cv.imwrite("erode.jpg", erosion)
        #
        # #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>过度膨胀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        #
        # kernel_2 = np.ones((4, 4), np.uint8)
        #
        # dilation = cv.dilate(erosion, kernel_2, iterations=5)
        #
        # detect_flag = dilation[35 , 40]

        label = '0x0A'
        mission_count = 0

        if detect_flag == 255:
            cv.imwrite("crossing.jpg" , Original_image)
            #任务标志位加一，执行后跳出任务1
            mission_count = 1
            label = '0x01'


    elif int(mission) == 2:
        # 进入任务二
        # 需要在坡道前加速
        # 第一次检测道红框不用停车，正常行驶，适当加速
        lower_hsv_1 = np.array([156, 60, 100])
        upper_hsv_1 = np.array([180, 255, 255])

        lower_hsv_2 = np.array([0, 60, 100])
        upper_hsv_2 = np.array([10, 255, 255])
        # 二值化处理

        hsv = cv.cvtColor(Original_image, cv.COLOR_BGR2HSV)

        mask_up = cv.inRange(hsv, lowerb=lower_hsv_1, upperb=upper_hsv_1)
        mask_low = cv.inRange(hsv, lowerb=lower_hsv_2, upperb=upper_hsv_2)

        mask = mask_up + mask_low

        detect_flag = mask[210 , 30]#红线上坡前[210,30] [y,x]

        label = '0x02'
        mission_count = 0

        if detect_flag == 255:
            # 任务标志位加一，执行后跳出任务2
            mission_count = 1
            # label = '0x02'


    elif int(mission) == 3:
        #任务二
        #第二次扫到红框，停车
        lower_hsv_1 = np.array([156, 60, 100])
        upper_hsv_1 = np.array([180, 255, 255])

        lower_hsv_2 = np.array([0, 60, 100])
        upper_hsv_2 = np.array([10, 255, 255])
        # 二值化处理

        hsv = cv.cvtColor(Original_image, cv.COLOR_BGR2HSV)

        mask_up = cv.inRange(hsv, lowerb=lower_hsv_1, upperb=upper_hsv_1)
        mask_low = cv.inRange(hsv, lowerb=lower_hsv_2, upperb=upper_hsv_2)

        mask = mask_up + mask_low

        detect_flag = mask[150 , 60]#红线上坡后[175,60]

        label = '0x0C'

        if detect_flag == 255:
            # 任务标志位加一，执行后跳出任务3
            mission_count = 1
            label = '0x03'

    elif int(mission) == 4:
        # 任务三
        # 限速路段行驶
        # 检测限速路段标志物，图像读标识位置
        # 先腐蚀，后膨胀 去噪点
        # 过度膨胀，突出目标
        img = label_img  #label_img是前置摄像头采集的图片

        cropped = img[0:100, 240:320]  # 裁剪坐标为[y0:y1, x0:x1]

        # 2.阈值分割，二值化处理
        lower_hsv_1 = np.array([156, 60, 100])
        upper_hsv_1 = np.array([180, 255, 255])

        lower_hsv_2 = np.array([0, 60, 100])
        upper_hsv_2 = np.array([10, 255, 255])

        # 二值化处理
        hsv = cv.cvtColor(cropped, cv.COLOR_BGR2HSV)

        mask_up = cv.inRange(hsv, lowerb=lower_hsv_1, upperb=upper_hsv_1)
        mask_low = cv.inRange(hsv, lowerb=lower_hsv_2, upperb=upper_hsv_2)

        mask = mask_up + mask_low

        # 3.先腐蚀，后膨胀去除噪声，然后过度膨胀，放大目标。
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>腐蚀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        kernel_1 = np.ones((3, 3), np.uint8)  # 定义卷积核

        erosion = cv.erode(mask, kernel_1, iterations=2)

        # cv.imwrite("erode.jpg", erosion)

        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>膨胀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        kernel_2 = np.ones((8, 8), np.uint8)

        dilation = cv.dilate(erosion, kernel_2, iterations=6)

        # cv.imwrite("dilation.jpg", dilation)

        # 4.检测目标特征点
        detect_flag = dilation[40, 60]

        label = '0x0D'
        mission_count = 0

        if detect_flag == 255:

            # label = '0x04'
            mission_count = 1
            return_time = time.time()


    elif int(mission) == 5:
        #任务三结束

        #Time = return_time
        if(time.time() - Time < 8):
            print("time < 8")

            label = '0x0E'#限速路段行驶标签
            return_time = Time
            mission_count = 0

            # return label, mission_count, return_time

        elif(time.time() - Time > 8):
            print("time > 8")

            label = '0x0F'
            mission_count = 1
            return_time = Time

            # return label, mission_count, return_time

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> S弯进环岛 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    elif int(mission) == 6:
        # 任务四
        # 环岛标志1
        lower_hsv = np.array([25, 75, 190])
        upper_hsv = np.array([40, 255, 255]) #黄色
        # 二值化处理
        hsv = cv.cvtColor(Original_image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)

        detect_point_1 = mask[225 , 30]
        # detect_flag为255则说明检测到左侧的

        detect_point_2 = mask[225 , 295]
        # detect_flag为255则说明检测到右侧的

        label = '0x0F' #S弯行驶
        mission_count = 0
        return_time = Time

        if detect_point_2 == 255 :

        #第一次检测到环岛标志
        # if detect_point_2 == 255 :
            # 进入环岛
            # 任务标志位加一，执行后跳出任务
            mission_count = 1

            cv.imwrite("huandao_1.jpg" , Original_image)


    elif int(mission) == 7:
        # 任务四
        # 环岛标志2
        lower_hsv = np.array([25, 75, 190])
        upper_hsv = np.array([40, 255, 255])  # 黄色
        # 二值化处理
        hsv = cv.cvtColor(Original_image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)

        detect_point_1 = mask[220 , 40]
        # detect_flag为255则说明检测到左侧的

        detect_point_2 = mask[200, 270]
        # detect_flag为255则说明检测到右侧的

        label = '0xA1' #横向十字路口
        mission_count = 0

        #第二次检测到环岛标志
        # if detect_point_1 == 255 and detect_point_2 == 255:
        if detect_point_1 == 255 :
            # 任务标志位加一，执行后跳出任务
            mission_count = 1
            cv.imwrite("huandao_2.jpg", Original_image)

            return_time = time.time()

    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>加入防误读延时<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    elif int(mission) == 8:
        #环岛延时
        #Time = return_time
        if(time.time() - Time < 2):

            label = '0xA2'
            return_time = Time
            mission_count = 0

        elif(time.time() - Time >= 2):
            label = '0xA2'
            mission_count = 1
            return_time = Time


    elif int(mission) == 9:
        # 任务四
        # 环岛标志3
        lower_hsv = np.array([25, 75, 190])
        upper_hsv = np.array([40, 255, 255])  # 黄色
        # 二值化处理
        hsv = cv.cvtColor(Original_image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)

        detect_point_1 = mask[190 , 30]
        # detect_flag为255则说明检测到左侧的


        detect_point_2 = mask[220, 290]
        # detect_flag为255则说明检测到右侧的

        label = '0xA2'  # 环岛行驶
        mission_count = 0

        #第三次检测道环岛标志
        # if detect_point_1 == 255 and detect_point_2 == 255:
        if detect_point_2 == 255:
            # 任务标志位加一，执行后跳出任务
            # 出环岛，准备停车
            cv.imwrite("huandao_3.jpg", Original_image)
            mission_count = 1

    # elif int(mission) == 10:
    #     #任务五
    #     #红灯停车
    #     #利用cv模板匹配或者k210双目测距
    #     #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>模板法<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #     # result = cv.matchTemplate(label_img , template, cv.TM_SQDIFF_NORMED)
    #     # theight, twidth = template.shape[:2]
    #     # # 归一化处理
    #     # # cv.normalize( result, result, 0, 1, cv.NORM_MINMAX, -1 )
    #     # # 寻找矩阵（一维数组当做向量，用Mat定义）中的最大值和最小值的匹配结果及其位置
    #     # min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
    #     # # print("min_val:", min_val)
    #     # # print("max_val ", min_val)
    #     # # print("min_loc", min_loc)
    #     # # print("max_loc", max_loc)
    #     # label = '0xA1' #环岛行驶
    #     # mission_count = 0
    #     # if min_val < 0.5:
    #     #
    #     #     cv.rectangle(label_img , min_loc, (min_loc[0] + twidth, min_loc[1] + theight), (0, 0, 225), 2)
    #     #     # #保存绘图结果
    #     #     cv.imwrite("./template/result.jpg", label_img)
    #     #
    #     #     label = '0x06'
    #     #     mission_count = 1
    #
    #     # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>标记法<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #     #最新：利用转向蓝色标识，判断进入识别状态，然后识别红绿灯，看车道线定距离
    #     #第一次看到转向蓝色标签时车前边沿应该刚越过停车框第一条线，此时智能车需要适当减速，为接下来的操作争取时间
    #     #看到蓝色标签后先判断红绿灯状态，此时智能车处于减速状态，有足够的时间判断红绿灯状态，对红绿灯区域进行裁窗，
    #     #之后二值化处理，适当腐蚀后过度膨胀，突出特征颜色，然后取区域检测取值，判断红绿灯状态属于哪一种。
    #     #除红灯外其余状态都加速通过，红灯停靠期间持续判断颜色，一旦红灯消失，立刻加速然后转向。
    #
    #     img = label_img
    #
    #     # 色彩空间转换，二值化分割
    #     lower_hsv = np.array([100, 43, 46])
    #     upper_hsv = np.array([124, 255, 255])
    #
    #     hsv = cv.cvtColor(img , cv.COLOR_BGR2HSV)  # 色彩空间转换
    #
    #     cropped = img[75: 120, 30:100]  # 裁剪坐标为[y0:y1, x0:x1]
    #
    #     # cv.imwrite("result2.jpg", cropped)
    #
    #     mask = cv.inRange(cropped, lowerb=lower_hsv, upperb=upper_hsv)  # 二值化
    #
    #     # cv.imwrite("hsv.jpg", mask)
    #
    #     # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>腐蚀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #
    #     kernel_1 = np.ones((2, 2), np.uint8)  # 定义卷积核
    #
    #     erosion = cv.erode(mask, kernel_1, iterations=3)
    #
    #     # cv.imwrite("erode.jpg", erosion)
    #
    #     # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>膨胀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #
    #     kernel_2 = np.ones((6, 6), np.uint8)
    #
    #     dilation = cv.dilate(erosion, kernel_2, iterations=5)
    #     # cv.imwrite("erosion.jpg", dilation)
    #
    #     detect_point = dilation[20, 40]
    #
    #     mission_count = 0
    #     label = '0xA3'
    #
    #     if detect_point == 255:
    #         #看到蓝色左转标识进下一个状态
    #         #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>检测红绿灯<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #         mission_count = 1


    elif int(mission) == 10:
        #读红绿灯状态
        img = label_img

        # 色彩空间转换，二值化分割
        # lower_hsv_1 = np.array([156, 43, 46])
        # upper_hsv_1 = np.array([180, 255, 255])
        #
        # lower_hsv_2 = np.array([0, 43, 46])
        # upper_hsv_2 = np.array([10, 255, 255])
        # # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        #
        # cropped = img[100: 160, 140: 200]  # 裁剪坐标为[y0:y1, x0:x1]
        #
        # hsv = cv.cvtColor(cropped, cv.COLOR_BGR2HSV)  # 色彩空间转换
        # # cv.imwrite("result2.jpg", cropped)
        # mask1 = cv.inRange(hsv, lowerb=lower_hsv_1, upperb=upper_hsv_1)  # 二值化
        # mask2 = cv.inRange(hsv, lowerb=lower_hsv_2, upperb=upper_hsv_2)
        #
        # mask = mask1 + mask2

        # cv.imwrite("hsv.jpg", mask)
        #
        # #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>滤波<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        #
        # # Gaussian = cv.GaussianBlur(mask , (3, 3), 1.3) #高斯滤波
        #
        # # cv.imwrite("gaussian.jpg" , Gaussian)
        #
        # #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>腐蚀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        #
        # kernel_1 = np.ones((2, 2), np.uint8)  # 定义卷积核
        #
        # erosion = cv.erode(mask, kernel_1, iterations=3)
        #
        # # cv.imwrite("erode.jpg", erosion)
        #
        # # #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>膨胀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        #
        # kernel_2 = np.ones((8, 8), np.uint8)
        #
        # dilation = cv.dilate(erosion, kernel_2, iterations=6)
        #
        # # cv.imwrite("erosion.jpg", dilation)
        #
        # detect_traffic_light = dilation[40, 20]
        lower_hsv = np.array([0, 30, 220])
        upper_hsv = np.array([10, 155, 255])

        # # 二值化处理
        # lower_hsv = np.array([25, 75, 190])
        # upper_hsv = np.array([40, 255, 255]) #黄色

        cropped = img[100: 160, 140: 200]  # 裁剪坐标为[y0:y1, x0:x1]

        hsv = cv.cvtColor(cropped, cv.COLOR_BGR2HSV)  # 色彩空间转换

        # hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)
        # mask_2 = cv.inRange(hsv, lowerb=lower_hsv_2 , upperb=upper_hsv_2)

        # mask = mask_1 + mask_2

        # cv.imwrite("hsv2.jpg", mask)

        kernel_1 = np.ones((2, 2), np.uint8)  # 定义卷积核

        erosion = cv.erode(mask, kernel_1, iterations=1)

        # cv.imwrite("erosion.jpg", erosion)

        kernel_2 = np.ones((5, 5), np.uint8)

        dilation = cv.dilate(erosion, kernel_2, iterations=10)

        # cv.imwrite("dilation .jpg", dilation)

        # 图片剪裁
        # cropped = mask[210:235, 30:50]  # 裁剪坐标为[y0:y1, x0:x1]
        detect_traffic_light = dilation[30, 30]  # 取一个像素点，判断二值化之后的值 白线[160,40]
        detect_traffic_light = 255

        label = '0xA3'

        if detect_traffic_light == 255:
            cv.imwrite("traffic_light.jpg", cropped)
            # 此时已经检测到红灯
            #跳进下一个任务停车
            mission_count = 1

    #具体定点停车
    elif int(mission) == 11:
        #看车位置停车
        lower_hsv = np.array([25, 75, 190])
        upper_hsv = np.array([40, 255, 255])  # 黄色
        # 二值化处理
        hsv = cv.cvtColor(Original_image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)

        kernel_1 = np.ones((3, 3), np.uint8)  # 定义卷积核

        erosion = cv.erode(mask, kernel_1, iterations=4)

        kernel_2 = np.ones((4 , 4), np.uint8)

        dilation = cv.dilate(erosion, kernel_2, iterations=5)

        detect_point_1 = dilation[155 , 48]
        # detect_flag为255则说明检测到左侧的
        #
        detect_point_2 = mask[200, 270]
        # # detect_flag为255则说明检测到右侧的

        label = '0xA3'  #减速定位停车
        mission_count = 0

        # 第三次检测道环岛标志
        # if detect_point_1 == 255 and detect_point_2 == 255:
        if detect_point_1 == 255:
            #检测到停车位置
            #停车
            label = '0x06'
            cv.imwrite("huandao_4.jpg", Original_image)
            mission_count = 1

    #反复读取红绿灯状态，红灯变绿灯就走

    elif int(mission) == 12:
        #直角弯路段
        #读取终点标记
        lower_hsv = np.array([0, 0, 221])
        upper_hsv = np.array([180, 30, 255])  # 白色
        # 二值化处理
        hsv = cv.cvtColor(Original_image ,  cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)

        # cv.imwrite("hsv_result.jpg", mask)

        # #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>腐蚀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        kernel_1 = np.ones((3, 3), np.uint8)  # 定义卷积核

        erosion = cv.erode(mask, kernel_1, iterations=4)

        # cv.imwrite("erode.jpg", erosion)

        # #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>膨胀<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        kernel_2 = np.ones((9, 9), np.uint8)

        dilation = cv.dilate(erosion, kernel_2, iterations=9)

        # cv.imwrite("erosion.jpg", dilation)

        # cv.circle(dilation, (120, 200), 2, (0, 0, 255), 0)

        # cv.imwrite("result.jpg", dilation)

        label = '0xA0'#直角弯

        mission_count = 0

        end_flag = dilation[200,110]

        if end_flag == 255:
            # print("Arrive the end.")

            mission_count = 1 #进入延时环节

            return_time = time.time()

    elif int(mission) == 13:
        # 终点延时
        # Time = return_time
        if (time.time() - Time < 1):

            label = '0x0B'  #
            return_time = Time
            mission_count = 0

        elif (time.time() - Time >= 1):
            label = '0x0B'
            mission_count = 1
            return_time = Time

    elif int(mission) == 14:
        label = '0x07'
        mission_count = 0

    return label , mission_count , return_time


#分情况控制电机动作，任务或路段
def user_cmd(label , angle):
    #根据目标控制电机转速
    speed = 1540

    #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>任务动作<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    if label == '0x01':

        #0x01 任务一
        #人行横道前停车1s
        nowetime1 = time.time()

        # lib.send_cmd(1300, 1500)

        while (time.time() - nowetime1 < 1):
            print(">>>>>>>>>>>>>>>>>>>>>>>>>crossing stop<<<<<<<<<<<<<<<<<<<<<<<<<")

            lib.send_cmd(1490, 1500)

    if label == '0x02':
        #第一次扫到红色框 行驶正常
        # nowetime2 = time.time()
        # while (time.time() - nowetime2 < 1):

        p_left = 4
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up

        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1565 , angle)
        print("**************************frist_red_line*****************************")
        # while (time.time() - nowetime2 < 1):
        #     lib.send_cmd(1545 , 1500)


    if label == '0x03':
        #第二次扫到红色框，停车
        nowetime3 = time.time()

        while (time.time() - nowetime3 < 1):
            print("**************************red_box_stop*****************************")
            lib.send_cmd(1500, 1500)

        # while (time.time() - nowetime3 < 1):
        #     print("**************************red_box_stop*****************************")
        #     lib.send_cmd(1550, 1500)

    if label == '0x04':
        #限速路段行驶
        #限制行驶速度，适当降低
        p_left = 4
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1535, angle)


    if label == '0x05':
        #进入环岛
        print("**************************huandao out*****************************")
        p_left = 4
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1540, angle)


    if label == '0x06':
        #红绿灯停车
        nowetime6 = time.time()

        while (time.time() - nowetime6 < 1):
            print("**************************traffic_light_stop*****************************")
            lib.send_cmd(1487 , 1500)

    if label == '0x07':
        #终点停车
        while (True):
            print("************************** Arrive the end. *****************************")
            lib.send_cmd(1500, 1500)


    #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>特定路段<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    if label == '0x00':
        # 路段一 人行道停车之前
        # 通用路段

        # 转弯幅度
        p_left = 4
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1550, angle)


    if label == '0x0A':
        #路段一 人行道停车之前
        #通用路段


        # 转弯幅度
        p_left = 6
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1575 , angle)


    if label == '0x0B':
        #  直角弯

        # 转弯幅度
        p_left = 4
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1543 , angle)

    if label == '0x0C':
        # 路段三 坡道前---->停车线前

        # 转弯幅度
        p_left = 3
        p_right = 3

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1550, angle)


    if label == "0x0D":
        # 路段四 坡道停车---->限速路段

        # 转弯幅度
        p_left = 5
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1560 , angle)


    if label == '0x0E':
        #限速路段行驶

        # 转弯幅度
        p_left = 4
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        print(">>>>>>>>>>>   limit 10 speed run  <<<<<<<<<")

        lib.send_cmd(1535 , angle)

    if label == '0x0F':
        # 路段四 限速路段---->进入环岛
        # s弯区域
        # 转弯幅度

        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>S S S<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")

        p_left = 5
        p_right = 6

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1552 , angle)

    if label == '0xA1':
        # 路段五 环岛区域前横向十字路口

        # 转弯幅度
        p_left = 4
        p_right = 5

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>col crossing<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")

        lib.send_cmd(1555 , angle)


    if label == '0xA2':
        # 路段五 环岛区域

        # 转弯幅度
        p_left = 4
        p_right = 6

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>huandao...<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")

        lib.send_cmd(1555 , angle)


    if label == '0xA3':
        # 路段五 红路灯停车框前
        # 减速行驶

        # 转弯幅度
        p_left = 4
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        print(">>>>>>>>>>>speed lower<<<<<<<<<<")

        lib.send_cmd(1545 , angle)


    if label == '0xA4':
        # 路段五 红路灯停车框前 读红绿灯后定线
        # 减速行驶

        # 转弯幅度
        p_left = 4
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1535, angle)

    if label == '0xA0':
        # 转弯幅度
        p_left = 5
        p_right = 4

        min_limit = 1500
        max_limit = 1500

        if (angle >= min_limit and angle <= max_limit):
            angle = 1500

        if (angle > max_limit):
            up = p_left * (angle - max_limit)
            angle = angle + up
        elif (angle < min_limit):
            up = p_right * (min_limit - angle)
            angle = angle - up

        if (angle > 3000):
            angle = 3000
        elif (angle < 0):
            angle = 0

        lib.send_cmd(1555 , angle)