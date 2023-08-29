import cv2 as cv
import numpy as np
import os


path = "./turn_left"

if __name__ == '__main__':
    folders = os.listdir(path)

    for img_name in folders:

        img_path = path + '/' + img_name
        print(img_path)

        img = cv.imread(img_path)


        B, G, R = cv.split(img)

        B1 = np.clip(cv.subtract(1 * B, 70), 0, 255)
        G1 = np.clip(cv.subtract(1 * G, 70), 0, 255)
        R1 = np.clip(cv.subtract(1 * R, 70), 0, 255)

        img_result = np.uint8(cv.merge((B1, G1, R1)))
        # img_Result = cv.cvtColor(img_result, cv.COLOR_HSV2RGB)

        cv.imwrite("./E/" + img_name , img_result)

        print('已处理 :' , "./E/" + img_name)


