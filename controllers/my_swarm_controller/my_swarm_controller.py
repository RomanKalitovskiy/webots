from controller import Robot, Camera, Emitter
import cv2
import numpy as np
import math
# Initialize the robot and camera
robot = Robot()

base_timestep = int(robot.getBasicTimeStep())
timestep = 64
camera = Camera('camera')
camera.enable(64) # Enable the camera with a refresh rate of 10ms
emitter = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')
red_led = robot.getDevice("led0")
green_led = robot.getDevice("led8")
red_led.set(255)
green_led.set(0)

# set the LED to red (full red, no green, no blue)
# led.set(1)
EMITTER_RANGE = 0.5

emitter.setRange(EMITTER_RANGE) # Set the range to 20 cm
receiver.enable(10) 
robot_name = robot.getName()

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.getPositionSensor().enable(timestep)
right_motor.getPositionSensor().enable(timestep)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

def cut_speed(speed):
    if speed > 7.536:
        speed = 7.536
    elif speed < -7.536:
        speed = -7.536
    return speed

def set_speed(left, right):

    left_motor.setVelocity(cut_speed(left))
    right_motor.setVelocity(cut_speed(right))

set_speed(0, 0)




def find_shape(img, color):
    # there are 2 colors of shapes: red and green
    img = np.frombuffer(img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if color == "red":
        lower_color = np.array([0, 50, 50])
        upper_color = np.array([10, 255, 255])
    elif color == "green":
        lower_color = np.array([50, 50, 50])
        upper_color = np.array([70, 255, 255])
    else:
        return None, None
    mask_color = cv2.inRange(hsv, lower_color, upper_color)
    contours_color, _ = cv2.findContours(mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_color:
        area = cv2.contourArea(cnt)
        if area > 500:
            # Find the moments of the contour to get the centroid
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            # Calculate the radius and size of the sphere
            radius = int(np.sqrt(area/np.pi))
            size = radius*2
            x_pos = (cx - 320)/320
            return x_pos, size
    
    # If no sphere is found, return None
    return None, None


score = 0
tumbler = "red"
distance = -1.
def toggle_tumbler():
    global tumbler, score
    score += 1
    print("Score: {}".format(score))
    if tumbler == "red":
        tumbler = "green"
        red_led.set(0)
        green_led.set(1)
    else:
        tumbler = "red"
        red_led.set(1)
        green_led.set(0)

search_speed = 5  # angular velocity for searching
max_search_time = 5  # maximum time to spend searching before giving up
search_start_time = -max_search_time  # time when search started


def rotate_degree(angle):
    speed = 2
    if angle < 0:
        speed = -speed
        angle = -angle
    angle = int(angle / 360 * 140)
    set_speed(speed, -speed)
    for _ in range(angle):
        robot.step(32)
    set_speed(0, 0)

def rotate_radian(angle):
    angle = math.degrees(angle)
    rotate_degree(angle)



# a_dict = {'color': 'blue', 'fruit': 'apple', 'pet': 'dog'}
# for key, value in a_dict.items():
#   print(key, '->', value)

dict = {}
mode = "scanning"
# Main control loop
while robot.step(64) != -1:
    # print(robot_name, mode, dict)
    image = camera.getImage()
    position, size = find_shape(image, tumbler)
    if position is not None:
        mode = "scanning"
        if size > 250:
            toggle_tumbler()
        distance = 0 + EMITTER_RANGE
        emitter.send("{},{},{}".format(robot_name, tumbler, distance))
        set_speed(position*3+5, -position*3+5)
        
    else:
        if receiver.getQueueLength() > 0:
            message = receiver.getString()
            message = message.split(",")
            sender_name = message[0]
            sender_color = message[1]
            sender_distance = float(message[2])
            emitter_direction = receiver.getEmitterDirection()
            sender_angle = math.atan2(emitter_direction[0], emitter_direction[1])-math.pi/2
            dict[sender_name] = (sender_color, sender_distance, sender_angle)
        
        receiver.nextPacket()
        distance = -1
        emitter.send("{},{},{}".format(robot_name, tumbler, distance))
        min = 1000000
        angle = None
        for key, value in dict.items():
            if value[0] == tumbler:
                if value[1] < min:
                    min = value[1]
                    angle = value[2]
        if min < 1000000 and min > 0:
            emitter.send("{},{},{}".format(robot_name, tumbler, min+EMITTER_RANGE))
            set_speed(angle+5, -angle+5)
        else:
          if robot.getTime() - search_start_time > max_search_time:
              search_start_time = robot.getTime()
              search_direction = np.random.uniform(low=-180, high=180)
              rotate_degree(search_direction)
              set_speed(5, 5)
    
    
    pass