#166 131 96 Red min
#176 255 228 Red max

#39 89 30 gREEN min
#84 216 129 green max

import cv2
import RPi.GPIO as GPIO
import numpy as np


GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

PINREADY = 3
PINRED = 5
PINGREEN = 7

GPIO.setup(PINREADY, GPIO.OUT)
GPIO.setup(PINRED, GPIO.OUT)
GPIO.setup(PINGREEN, GPIO.OUT)

GPIO.output(PINRED, GPIO.LOW)
GPIO.output(PINGREEN, GPIO.LOW)
GPIO.output(PINREADY, GPIO.LOW)

cv2.namedWindow("result")

cap = cv2.VideoCapture(0)
cap.set(3, 320)# set video widht
cap.set(4, 240)
# HSV фильтр для зеленых объектов из прошлого урока
hsv_minR = np.array((170, 76, 59), np.uint8)
hsv_maxR = np.array((177, 255, 218), np.uint8)
hsv_minG = np.array((39, 89, 30), np.uint8)
hsv_maxG = np.array((84, 216, 129), np.uint8)


while True:
    flag, img = cap.read()
    # преобразуем RGB картинку в HSV модель
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # применяем цветовой фильтр
    threshR = cv2.inRange(hsv, hsv_minR, hsv_maxR)
    threshG = cv2.inRange(hsv, hsv_minG, hsv_maxG)
    # вычисляем моменты изображения
    momentsR = cv2.moments(threshR, 1)
    dM01R = momentsR['m01']
    dM10R = momentsR['m10']
    dAreaR = momentsR['m00']
    momentsG = cv2.moments(threshG, 1)
    dM01G = momentsG['m01']
    dM10G = momentsG['m10']
    dAreaG = momentsG['m00']
    # будем реагировать только на те моменты,
    # которые содержать больше 100 пикселей
    if dAreaR > dAreaG and dAreaR > 3200:
        GPIO.output(PINRED, GPIO.HIGH)
        GPIO.output(PINGREEN, GPIO.LOW)
        x = int(dM10R / dAreaR)
        y = int(dM01R / dAreaR)
        print("x->",x ,"y->",y , "red", "pixels=>", dAreaR)
        cv2.circle(img, (x, y), 10, (0, 0, 255), -1)
    elif dAreaG > dAreaR and dAreaG > 3200:
        GPIO.output(PINRED, GPIO.LOW)
        GPIO.output(PINGREEN, GPIO.HIGH)
        x = int(dM10G / dAreaG)
        y = int(dM01G / dAreaG)
        print("x->",x ,"y->",y , "green", "pixels=>", dAreaG)
        cv2.circle(img, (x, y), 10, (0, 255, 0), -1)
    else:
        GPIO.output(PINRED, GPIO.LOW)
        GPIO.output(PINGREEN, GPIO.LOW)
    cv2.imshow('result', img)
    ch = cv2.waitKey(5)
    if ch == 27:
        break

cap.release()
cv2.destroyAllWindows()
