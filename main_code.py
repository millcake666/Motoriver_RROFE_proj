import pyb
from pyb import Timer, Servo
from machine import Pin, I2C
from vl53l0x import setup_tofl_device, TBOOT
import utime
import sensor
import image
import time

#Настройка пинов для управления моторами хз как этрабтает
timer = Timer(2, freq=100)
motor_pin1 = timer.channel(3,Timer.PWM, pin = Pin('P4'))
motor_pin2 = timer.channel(4,Timer.PWM, pin = Pin('P5'))
motor_pin1.pulse_width_percent(0)
motor_pin2.pulse_width_percent(0)

pin = pyb.Pin('P6', pyb.Pin.IN)
adc = pyb.ADC(pin)

# Закупка батона
button = Pin('P1' , Pin.OUT)

# настройка лазеров
device_1_xshut = Pin('P3', Pin.OUT)
i2c_1 = I2C(scl='P8', sda='P7')
# Set this low to disable device 1
#print("Setting up device 0")
device_1_xshut.value(0)
tofl0 = setup_tofl_device(i2c_1, 20001, 10, 6)
tofl0.set_address(0x31) # Right
#print("Now setting up device 1")
# Re-enable device 1 - on the same bus
device_1_xshut.value(1)
utime.sleep_us(TBOOT)
tofl1 = setup_tofl_device(i2c_1, 20001, 10, 6) # Left

# константы для управление мотором
FORWARD = True
BACKWARD = False
FOLLOW = False
BREAK = True
# Настройка сервопривода
servo = Servo(3)
CORP_ANGLE = [-13, 24]
CORP_ANGLET = [-25, 38]
#CORP_ANGLE = [-5, 25]
KOEFF = 0.7
start = 0
turn = 0

# настройка режимов камеры
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

trasholdBlue = ((21, 63, -24, 39, -64, -2))
trasholdOrange = ((38, 61, -3, 42, 13, 38))

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
    if direction :
        motor_pin1.pulse_width_percent(0)
        motor_pin2.pulse_width_percent(speed)
    else:
        motor_pin2.pulse_width_percent(0)
        motor_pin1.pulse_width_percent(speed)


# Стоп
def stop(var):
    if var :
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
    go(FORWARD, 22) # запуск робота
    corr_value = -((300 - tofl1.ping()) * KOEFF) # "Ошибка" для регулятора
    set_angle(corr_value)

def ride_wallL(speed, dist):
    go(FORWARD, speed)
    tof_dist = tofl0.ping()
    print(tof_dist)
    corr_angle = -((tof_dist - dist ) * KOEFF)
    set_angleT(corr_angle)


def ride_wallR(speed, dist):
    go(FORWARD, speed)
    tof_dist = tofl1.ping()
    print(tof_dist)
    corr_angle = (tof_dist - dist ) * KOEFF
    set_angleT(corr_angle)


#################################################################################################
while button.value() == 0:
    motor_pin1.pulse_width_percent(0)
    motor_pin2.pulse_width_percent(0)
while button.value() == 1:
    motor_pin1.pulse_width_percent(0)
    motor_pin2.pulse_width_percent(0)

while start == 0:
    go(FORWARD, 20)



    img = sensor.snapshot()
    blobsB = img.find_blobs([trasholdBlue] , area_threshold = 2500 , merge = True)
    blobsO = img.find_blobs([trasholdOrange] , area_threshold = 2500 , merge = True)


    max_sizeB = 0
    max_sizeO = 0
    max_blobB = []
    max_blobO = []

    for blob in blobsB :
        if blob.pixels() > max_sizeB:
            max_sizeB = blob.pixels()
            max_blobB = blob
    for blob in blobsO :
        if blob.pixels() > max_sizeO:
            max_sizeO = blob.pixels()
            max_blobO = blob
    max_size = [max_sizeB , max_sizeO]
    cflag = bool()
    blob_cord = []
    if max_blobB and max(max_size) == max_sizeB :
        img.draw_rectangle(max_blobB.rect(), color = (0,0,255))
        img.draw_cross(max_blobB.cx() , max_blobB.cy() , color = (0,0,255))
        blob_cord = [max_blobB.cx() , max_blobB.cy()]
        start = 1
        turn += 1
        cflag = True
    elif max_blobO and max(max_size) == max_sizeO :
        img.draw_rectangle(max_blobO.rect(), color = (255,0,0))
        img.draw_cross(max_blobO.cx() , max_blobO.cy() , color = (255,0,0))
        blob_cord = [max_blobO.cx() , max_blobO.cy()]
        start = 1
        turn += 1
        cflag = False
stop(BREAK)
time.sleep_ms(3000)
if cflag:
    while turn != 12:
        img = sensor.snapshot()
        blobsB = img.find_blobs([trasholdBlue] , area_threshold = 2500 , merge = True)
        blobsO = img.find_blobs([trasholdOrange] , area_threshold = 2500 , merge = True)


        max_sizeB = 0
        max_sizeO = 0
        max_blobB = []
        max_blobO = []

        for blob in blobsB :
            if blob.pixels() > max_sizeB:
                max_sizeB = blob.pixels()
                max_blobB = blob
        for blob in blobsO :
            if blob.pixels() > max_sizeO:
                max_sizeO = blob.pixels()
                max_blobO = blob
        max_size = [max_sizeB , max_sizeO]
        blob_cord = []
        if max_blobB and max(max_size) == max_sizeB :
            img.draw_rectangle(max_blobB.rect(), color = (0,0,255))
            img.draw_cross(max_blobB.cx() , max_blobB.cy() , color = (0,0,255))
            blob_cord = [max_blobB.cx() , max_blobB.cy()]
            start = 1
            turn += 1
        elif max_blobO and max(max_size) == max_sizeO :
            img.draw_rectangle(max_blobO.rect(), color = (255,0,0))
            img.draw_cross(max_blobO.cx() , max_blobO.cy() , color = (255,0,0))
            blob_cord = [max_blobO.cx() , max_blobO.cy()]
            start = 1
        #ride_wallR(22, 250)
        ride_wallL(22, 250)
else:
    while turn != 12:
        img = sensor.snapshot()
        blobsB = img.find_blobs([trasholdBlue] , area_threshold = 2500 , merge = True)
        blobsO = img.find_blobs([trasholdOrange] , area_threshold = 2500 , merge = True)


        max_sizeB = 0
        max_sizeO = 0
        max_blobB = []
        max_blobO = []

        for blob in blobsB :
            if blob.pixels() > max_sizeB:
                max_sizeB = blob.pixels()
                max_blobB = blob
        for blob in blobsO :
            if blob.pixels() > max_sizeO:
                max_sizeO = blob.pixels()
                max_blobO = blob
        max_size = [max_sizeB , max_sizeO]
        blob_cord = []
        if max_blobB and max(max_size) == max_sizeB :
            img.draw_rectangle(max_blobB.rect(), color = (0,0,255))
            img.draw_cross(max_blobB.cx() , max_blobB.cy() , color = (0,0,255))
            blob_cord = [max_blobB.cx() , max_blobB.cy()]
            start = 1
            turn += 1
        elif max_blobO and max(max_size) == max_sizeO :
            img.draw_rectangle(max_blobO.rect(), color = (255,0,0))
            img.draw_cross(max_blobO.cx() , max_blobO.cy() , color = (255,0,0))
            blob_cord = [max_blobO.cx() , max_blobO.cy()]
            start = 1
        Ride_wallR(22, 250)
        #ride_wallL(22, 250)
