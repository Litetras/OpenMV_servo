import sensor, image, time
from pid import PID
from pyb import Servo, UART
from pyb import millis
from math import pi, isnan

class PID:
    _kp = _ki = _kd = _integrator = _imax = 0
    _last_error = _last_derivative = _last_t = 0
    _RC = 1/(2 * pi * 20)
    def __init__(self, p=0, i=0, d=0, imax=0):
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self._last_derivative = float('nan')

    def get_pid(self, error, scaler):
        tnow = millis()
        dt = tnow - self._last_t
        output = 0
        if self._last_t == 0 or dt > 1000:
            dt = 0
            self.reset_I()
        self._last_t = tnow
        delta_time = float(dt) / float(1000)
        output += error * self._kp
        if abs(self._kd) > 0 and dt > 0:
            if isnan(self._last_derivative):
                derivative = 0
                self._last_derivative = 0
            else:
                derivative = (error - self._last_error) / delta_time
            derivative = self._last_derivative + \
                                     ((delta_time / (self._RC + delta_time)) * \
                                        (derivative - self._last_derivative))
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative
        output *= scaler
        if abs(self._ki) > 0 and dt > 0:
            self._integrator += (error * self._ki) * scaler * delta_time
            if self._integrator < -self._imax: self._integrator = -self._imax
            elif self._integrator > self._imax: self._integrator = self._imax
            output += self._integrator
        return output

    def reset_I(self):
        self._integrator = 0
        self._last_derivative = float('nan')


# 初始化串口
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# 黑色阈值设置（HSV）+
black_threshold = (0, 57, -57, 21, 17, -30)


# 初始化PID控制器
pan_pid = PID(p=0.07, i=0, imax=90)
tilt_pid = PID(p=0.05, i=0, imax=90)

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 使用RGB格式
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=5000)
sensor.set_vflip(True)             # 垂直翻转（True/False，根据安装方向调整）
clock = time.clock()
sensor.set_auto_whitebal(True)    # 关闭自动白平衡，保持颜色稳定性
sensor.set_contrast(500)             # 提高对比度，增强黑色边框识别
def find_centroid(rect):
    """计算矩形中心点（根据四个角点）"""
    corners = rect.corners()
    cx = (corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4
    cy = (corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4
    return int(cx), int(cy)

def find_max_rectangle(rects):
    """寻找面积最大的矩形"""
    max_rect = None
    max_area = 0
    for rect in rects:
        corners = rect.corners()
        width = ((corners[0][0] - corners[1][0])**2 + (corners[0][1] - corners[1][1])** 2)**0.5
        height = ((corners[1][0] - corners[2][0])** 2 + (corners[1][1] - corners[2][1])**2)** 0.5
        area = width * height
        if area > max_area:
            max_rect = rect
            max_area = area
    return max_rect


# 主循环
while(True):
    clock.tick()
    img = sensor.snapshot()

    # 初始化变量
    cx, cy = 0, 0
    retangle_find = 0
    should_send = True  # 默认发送数据
    laser_on = 0  # 新增：激光状态变量，默认关闭
    
    # 设置画面中点
    screen_center_x = img.width() // 2
    screen_center_y = img.height() // 2
    
    # 在画面中点绘制十字标记
    img.draw_cross(screen_center_x, screen_center_y, color=(0, 0, 255), size=3)
    
    # 初始化位置关系变量
    left_of_center = 0
    right_of_center = 0
    above_center = 0
    below_center = 0

    # 寻找矩形
    rects = img.find_rects(threshold=15000)
    if rects:
        max_rect = find_max_rectangle(rects)
        if max_rect:
            cx, cy = find_centroid(max_rect)
            retangle_find = 1
            
            # 计算与画面中点的差值
            dx = abs(cx - screen_center_x)
            dy = abs(cy - screen_center_y)
            
            # 判断是否在允许范围内（误差范围改为±10像素）
            if dx <= 5 and dy <= 5:
                laser_on = 1  # 在误差范围内，开启激光
                # 在图像上显示中心对齐状态
                img.draw_string(screen_center_x + 10, screen_center_y - 10, "CENTER ALIGNED", color=(0, 255, 0))
                # 绘制绿色圆圈表示激光开启
                img.draw_circle(screen_center_x, screen_center_y, 15, color=(0, 255, 0), thickness=2)
            else:
                laser_on = 0  # 超出误差范围，关闭激光
                # 判断矩形中点相对于画面中点的位置
                if cx < screen_center_x:
                    left_of_center = 1
                elif cx > screen_center_x:
                    right_of_center = 1
                    
                if cy < screen_center_y:
                    above_center = 1
                elif cy > screen_center_y:
                    below_center = 1
                
                # 在图像上显示位置关系
                position_text = ""
                if left_of_center:
                    position_text += "Left "
                if right_of_center:
                    position_text += "Right "
                if above_center:
                    position_text += "Above "
                if below_center:
                    position_text += "Below"
                    
                if not position_text:
                    position_text = "Center"
                    
                img.draw_string(cx + 10, cy - 10, position_text, color=(255, 255, 0))
            
            # 绘制矩形
            corners = max_rect.corners()
            for i in range(4):
                x0, y0 = corners[i]
                x1, y1 = corners[(i+1)%4]
                img.draw_line(x0, y0, x1, y1, color=(255, 0, 0), thickness=2)
            img.draw_cross(cx, cy, color=(0, 255, 0), size=2)
        else:
            print("未找到有效矩形")
    else:
        print("未检测到矩形")

    # 发送串口数据，包含激光状态变量
    if should_send:
        # 发送常规数据（包含位置关系和激光状态）
        coord_str = "${},{},{},{},{},{}#".format(
             retangle_find,
            left_of_center, right_of_center,
            above_center, below_center,
            laser_on  # 新增激光状态
        )
        print(coord_str)
        uart.write(coord_str)
    else:
        # 当中心对齐时，发送激光状态数据
        aligned_str = "$1,0,0,0,0,{}#".format(laser_on)
        print(aligned_str)
        uart.write(aligned_str)

    # 显示帧率
    img.draw_string(0, 0, "FPS:%.1f" % clock.fps(), color=(255, 0, 0))
    # 显示画面中点坐标
    img.draw_string(0, 10, "Center:({},{})".format(screen_center_x, screen_center_y), color=(0, 0, 255))
    # 显示当前矩形中心坐标
    img.draw_string(0, 20, "Rect:({},{})".format(cx, cy), color=(255, 0, 0))
    # 显示发送状态
    img.draw_string(0, 30, "Send:{}".format("Normal" if should_send else "Aligned"), color=(0, 255, 0))
    # 显示激光状态
    img.draw_string(0, 40, "Laser:{}".format("ON" if laser_on == 1 else "OFF"), color=(0, 255, 0) if laser_on == 1 else (255, 0, 0))