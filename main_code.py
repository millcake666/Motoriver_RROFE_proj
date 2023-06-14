import pyb
from pyb import Timer, Servo
from machine import Pin, I2C
from vl53l0x import setup_tofl_device, TBOOT
import utime
import sensor
import image
import time
import math

# Настройка пинов для управления моторами хз как этрабтает
timer = Timer(2, freq=100)
motor_pin1 = timer.channel(3, Timer.PWM, pin=Pin('P4'))
motor_pin2 = timer.channel(4, Timer.PWM, pin=Pin('P5'))
motor_pin1.pulse_width_percent(0)
motor_pin2.pulse_width_percent(0)

# Кнопка
button = Pin('P1', Pin.OUT)

# настройка лазеров
device_1_xshut = Pin('P3', Pin.OUT)
i2c_1 = I2C(scl='P8', sda='P7')
# Set this low to disable device 1
# print("Setting up device 0")
device_1_xshut.value(0)
tofl0 = setup_tofl_device(i2c_1, 20001, 10, 6)
tofl0.set_address(0x31)  # Right
# print("Now setting up device 1")
# Re-enable device 1 - on the same bus
device_1_xshut.value(1)
utime.sleep_us(TBOOT)
tofl1 = setup_tofl_device(i2c_1, 20001, 10, 6)  # Left

# константы для управление мотором
FORWARD = True
BACKWARD = False
FOLLOW = False
BREAK = True
# Настройка сервопривода
servo = Servo(3)
CORP_ANGLE = [-17, 5]
CORP_ANGLET = [-37, 25]
# CORP_ANGLE = [-5, 25]
KOEFF = 1
TO_DEG = 180 / 3.14159265
start = 0

# настройка режимов камеры
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
ROI = (0, 120, 320, 240)
trasholdBlue = ((10, 47, -59, 32, -51, -4))
trasholdOrange = ((26, 64, 8, 54, 5, 61))

# Функции для движения робота
# Direction - bool , spped - int (0 - 100)


a = []


def filtL(value):
    a.append(value)
    if len(a) != 3:
        return 0

    if (a[1] <= a[0] and a[0] <= a[2]) or (a[2] <= a[0] and a[0] <= a[1]):
        ret = a.pop(0)
        return ret
    if (a[0] <= a[1] and a[1] <= a[2]) or (a[2] <= a[1] and a[1] <= a[0]):
        buf = a[1]
        ret = a.pop(0)
        return buf
    if (a[0] <= a[2] and a[2] <= a[1]) or (a[1] <= a[2] and a[2] <= a[0]):
        buf = a[2]
        ret = a.pop(0)
        return buf


b = []


def filtR(value):
    b.append(value)
    if len(b) != 3:
        return 0

    if (b[1] <= b[0] and b[0] <= b[2]) or (b[2] <= b[0] and b[0] <= b[1]):
        ret = b.pop(0)
        return ret
    if (b[0] <= b[1] and b[1] <= b[2]) or (b[2] <= b[1] and b[1] <= b[0]):
        buf = b[1]
        ret = b.pop(0)
        return buf
    if (b[0] <= b[2] and b[2] <= b[1]) or (b[1] <= b[2] and b[2] <= b[0]):
        buf = b[2]
        ret = b.pop(0)
        return buf


# Вперёд назад
def go(direction, speed):
    if direction:
        motor_pin1.pulse_width_percent(0)
        motor_pin2.pulse_width_percent(speed)
    else:
        motor_pin2.pulse_width_percent(0)
        motor_pin1.pulse_width_percent(speed)


# Стоп
def stop(var):
    if var:
        motor_pin1.pulse_width_percent(100)
        motor_pin2.pulse_width_percent(100)
    else:
        motor_pin1.pulse_width_percent(0)
        motor_pin2.pulse_width_percent(0)


# Серва поворот
def set_angle(angle):
    # Неогр значений
    if angle > 90:
        angle = 90
    if angle < -90:
        angle = -90
    norm_angle = (angle - (-90)) * (CORP_ANGLE[1] - CORP_ANGLE[0]) / (90 - (-90)) + CORP_ANGLE[0]

    servo.angle(norm_angle)


def set_angleT(angle):
    # Неогр значений
    if angle > 90:
        angle = 90
    if angle < -90:
        angle = -90
    norm_angle = (angle - (-90)) * (CORP_ANGLET[1] - CORP_ANGLET[0]) / (90 - (-90)) + CORP_ANGLET[0]

    servo.angle(norm_angle)


def ride_center():
    go(FORWARD, 22)  # запуск робота
    corr_value = -((300 - tofl1.ping()) * KOEFF)  # "Ошибка" для регулятора
    set_angle(corr_value)


def ride_wallL(speed, dist):
    go(FORWARD, speed)
    tof_dist = tofl0.ping()
    # print(tof_dist)
    corr_angle = -((tof_dist - dist) * KOEFF)
    set_angleT(corr_angle)


def ride_wallR(speed, dist):
    go(FORWARD, speed)
    tof_dist = tofl1.ping()
    # print(tof_dist)
    corr_angle = (tof_dist - dist) * KOEFF
    set_angleT(corr_angle)


#################################################################################################
while button.value() == 0:
    motor_pin1.pulse_width_percent(0)
    motor_pin2.pulse_width_percent(0)
while button.value() == 1:
    motor_pin1.pulse_width_percent(0)
    motor_pin2.pulse_width_percent(0)
position = 0
left_l = []
right_l = []
for i in range(10):
    left_l.append(tofl0.ping())
    right_l.append(tofl1.ping())
turn = 0
cflag = False
set_angleT(-7)

while start == 0:
    go(FORWARD, 22)

    img = sensor.snapshot()
    blobsB = img.find_blobs([trasholdBlue], area_threshold=1500, merge=True, roi=ROI)
    blobsO = img.find_blobs([trasholdOrange], area_threshold=1500, merge=True, roi=ROI)

    max_sizeB = 0
    max_sizeO = 0
    max_blobB = []
    max_blobO = []

    for blob in blobsB:
        if blob.pixels() > max_sizeB:
            max_sizeB = blob.pixels()
            max_blobB = blob
    for blob in blobsO:
        if blob.pixels() > max_sizeO:
            max_sizeO = blob.pixels()
            max_blobO = blob

    max_size = [max_sizeB, max_sizeO]
    # if max_blobB and max_blobO and max(max_size) == max_sizeB:
    if max_blobB != [] and max_blobO != [] and max_blobB.pixels() > max_blobO.pixels() and max_blobB.pixels() > 1000:
        img.draw_rectangle(max_blobB.rect(), color=(0, 0, 255))
        img.draw_cross(max_blobB.cx(), max_blobB.cy(), color=(0, 0, 255))
        img.draw_rectangle(max_blobO.rect(), color=(255, 0, 0))
        img.draw_cross(max_blobO.cx(), max_blobO.cy(), color=(255, 0, 0))
        print('Blue Pixels =>', max_blobB.pixels(), 'Orange Pixels =>', max_blobO.pixels())
        cflag = True
        if min(left_l) < 350:
            position = 1
        elif min(right_l) < 350:
            position = 2
        else:
            position = 2
        start = 1
    elif max_blobO != [] and max_blobB != [] and max_blobO.pixels() > max_blobB.pixels() and max_blobO.pixels() > 1000:
        cflag = False
        start = 1
        if min(right_l) < 350:
            position = 1
        elif min(left_l) < 350:
            position = 2
        else:
            position = 2
    elif max_blobB != [] and max_blobB.pixels() > 800:
        cflag = True
        img.draw_rectangle(max_blobB.rect(), color=(0, 0, 255))
        img.draw_cross(max_blobB.cx(), max_blobB.cy(), color=(0, 0, 255))
        if min(left_l) < 350:
            position = 1
        elif min(right_l) < 350:
            position = 2
        else:
            position = 2
        start = 1
    elif max_blobO != [] and max_blobO.pixels() > 800:
        cflag = False
        img.draw_rectangle(max_blobO.rect(), color=(255, 0, 0))
        img.draw_cross(max_blobO.cx(), max_blobO.cy(), color=(255, 0, 0))
        start = 1
        if min(right_l) < 350:
            position = 1
        elif min(left_l) < 350:
            position = 2
        else:
            position = 2

stop(BREAK)
print(1)
print(cflag)
turn = 0
start_time = 0
state = True
if cflag:
    print('L')
    if position == 1:
        go(FORWARD, 25)
        time.sleep_ms(1250)
    elif position == 0:
        stop(BREAK)
        time.sleep_ms(10000)
    elif position == 2:
        go(FORWARD, 25)
        time.sleep_ms(1000)
        stop(BREAK)
        time.sleep_ms(500)
        set_angleT(-90)
        go(FORWARD, 25)
        time.sleep_ms(2500)
        set_angleT(-7)
        stop(BREAK)
        go(FORWARD, 25)
        time.sleep_ms(1000)
        turn += 1
    while turn < 13:
        img = sensor.snapshot()
        blobs = img.find_blobs([trasholdBlue, trasholdOrange], area_threshold=1500, merge=True, roi=ROI)
        max_blob = []
        max_size = 0

        for blob in blobs:
            if blob.pixels() > max_size:
                max_size = blob.pixels()
                max_blob = blob

        ride_wallL(25, 200)
        if max_blob != [] and (time.ticks_ms() - start_time) > 1850 and max_size > 1500:
            turn += 1
            start_time = time.ticks_ms()
else:
    print('R')
    if position == 1:
        go(FORWARD, 25)
        time.sleep_ms(1250)
    elif position == 0:
        stop(BREAK)
        time.sleep_ms(10000)
    elif position == 2:
        go(FORWARD, 25)
        time.sleep_ms(500)
        stop(BREAK)
        time.sleep_ms(500)
        set_angleT(90)
        go(FORWARD, 25)
        time.sleep_ms(2650)
        set_angleT(-7)
        stop(BREAK)
        go(FORWARD, 25)
        time.sleep_ms(1000)
        turn += 1
    KOEFF = 1
    while turn < 13:
        img = sensor.snapshot()
        blobs = img.find_blobs([trasholdBlue, trasholdOrange], area_threshold=1500, merge=True, roi=ROI)
        max_blob = []
        max_size = 0
        for blob in blobs:
            if blob.pixels() > max_size:
                max_size = blob.pixels()
                max_blob = blob

        ride_wallR(25, 250)
        if max_blob != [] and (time.ticks_ms() - start_time) > 1850 and max_size > 1500:
            turn += 1
            start_time = time.ticks_ms()

print('end')
stop(BREAK)
time.sleep_ms(500)
