import cv2 as cv
import os

path = "./E"

if __name__ == '__main__':
    folders = os.listdir(path)

    for img_name in folders:

        img_path = path + '/' + img_name
        print(img_path)

        img = cv.imread(img_path)

        dst = img[150 : 400 , 0 :640]

        cv.imwrite("./turn_left/" + img_name , dst)
        print('已处理 :' , "./E/" + img_name)
