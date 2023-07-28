import sensor, image, time

from pid import PID
from pyb import Servo

pan_servo=Servo(1)#p7
tilt_servo=Servo(2)#P8

pan_servo.calibration(500,2000,1500)
tilt_servo.calibration(750,2000,1500)#竖直方向不能低于750

red_threshold  = (30, 80, 11, 61, 32, 82)

pan_pid = PID(p=0.25, i=0, imax=90,d=0.06)
tilt_pid = PID(p=0.16, i=0, imax=90,d=0.04)


sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_vflip(True)#镜像翻转
sensor.set_hmirror(False)
sensor.set_auto_gain(False) # 颜色跟踪必须关闭自动增益
sensor.set_auto_whitebal(False) # 颜色跟踪必须关闭白平衡
sensor.set_auto_exposure(8300) #设置一个固定曝光时间
clock = time.clock() # Tracks FPS.

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob


while(True):
    clock.tick()
    img = sensor.snapshot().binary([red_threshold]).erode(0)# Take a picture and return the image.

    #blobs = img.find_blobs([red_threshold])
    blobs = img.find_blobs([(100, 100)])
    if blobs:
        max_blob = find_max(blobs)
        pan_error = max_blob.cx()-img.width()/2
        tilt_error = max_blob.cy()-img.height()/2

        print("pan_error: ", pan_error)

        img.draw_rectangle(max_blob.rect(),color=(255,0,0)) # rect
        img.draw_cross(max_blob.cx(), max_blob.cy(),color=(255,0,0)) # cx, cy

        pan_output=pan_pid.get_pid(pan_error,1)/2
        tilt_output=tilt_pid.get_pid(tilt_error,1)
        print("pan_output",pan_output)
        pan_servo.angle(pan_servo.angle()+pan_output)
        tilt_servo.angle(tilt_servo.angle()-tilt_output)
