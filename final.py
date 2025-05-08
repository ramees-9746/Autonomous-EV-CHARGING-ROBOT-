import argparse
import sys
import time
import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import numpy as np
import utils
import pigpio
import math
from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

#IR PINS
IR_LEFT_FRONT = 21
IR_RIGHT_FRONT = 16
IR_LEFT_BACK= 26
IR_RIGHT_BACK = 19

# GPIO pins for motor control
MOTOR1_PIN1 = 23
MOTOR1_PIN2 = 22
MOTOR2_PIN1 = 27
MOTOR2_PIN2 = 17

# GPIO Pins for servos
SHOULDER_SERVO = 18  
ELBOW_SERVO = 12     
WRIST_SERVO = 13 
 
TRIG = 2  # GPIO pin for Trig
ECHO = 3  # GPIO pin for Echo

# Initialize pigpio
pi = pigpio.pi()
pi.set_mode(IR_LEFT_FRONT, pigpio.INPUT)
pi.set_mode(IR_RIGHT_FRONT, pigpio.INPUT)
pi.set_mode(IR_LEFT_BACK, pigpio.INPUT)
pi.set_mode(IR_RIGHT_BACK, pigpio.INPUT)

# Set Motor Pins as Output
pi.set_mode(MOTOR1_PIN1, pigpio.OUTPUT)
pi.set_mode(MOTOR1_PIN2, pigpio.OUTPUT)
pi.set_mode(MOTOR2_PIN1, pigpio.OUTPUT)
pi.set_mode(MOTOR2_PIN2, pigpio.OUTPUT)

pi.set_mode(TRIG, pigpio.OUTPUT)
pi.set_mode(ECHO, pigpio.INPUT)

# Servo pulse width range (0° = 500us, 180° = 2500us)
MIN_PULSE = 500
MAX_PULSE = 2500
# Constants
PORT_WIDTH_CM = 3  # cm
PORT_HEIGHT_CM = 2  # cm
FOCAL_LENGTH_PX = 800  # px
# Arm segment lengths (cm) 
L1 = 12  # Shoulder to Elbow
L2 = 12  # Elbow to Wrist

# camera height 
h = 12.5
width_difference_cm =0
initial_shoulder = 0
initial_elbow = 0
initial_wrist = 0
# robot's current position
current_position = "slot1" 

sensor_value = "No data received yet"

if not pi.connected:
    print("Error: Unable to connect to pigpio daemon.")
    sys.exit(1)


SAFE_DISTANCE = 8   # Robot stops at 10 cm
alignment_threshold = 18  # Pixels tolerance for alignment
# Function to move forward
def move_forward():
    pi.write(MOTOR1_PIN1, 1)
    pi.write(MOTOR1_PIN2, 0)
    pi.write(MOTOR2_PIN1, 1)
    pi.write(MOTOR2_PIN2, 0)

# Function to move backward
def move_backward():
    pi.write(MOTOR1_PIN1, 0)
    pi.write(MOTOR1_PIN2, 1)
    pi.write(MOTOR2_PIN1, 0)
    pi.write(MOTOR2_PIN2, 1)
# Function to turn left
def turn_left():
    pi.write(MOTOR1_PIN1, 1)
    pi.write(MOTOR1_PIN2, 0)
    pi.write(MOTOR2_PIN1, 0)
    pi.write(MOTOR2_PIN2, 1)

# Function to turn right
def turn_right():
    pi.write(MOTOR1_PIN1, 0)
    pi.write(MOTOR1_PIN2, 1)
    pi.write(MOTOR2_PIN1, 1)
    pi.write(MOTOR2_PIN2, 0)


# Function to stop
def stop():
    pi.write(MOTOR1_PIN1, 0)
    pi.write(MOTOR1_PIN2, 0)
    pi.write(MOTOR2_PIN1, 0)
    pi.write(MOTOR2_PIN2, 0)


def measure_distance():
    SPEED_OF_SOUND = 34300 
    # Send a 10 µs trigger pulse
    pi.gpio_trigger(TRIG, 10, 1)

    # Wait for the echo pulse to start
    pulse_start = time.time()
    timeout = pulse_start + 0.02  # 20ms timeout

    while pi.read(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            print("Echo signal not received (timeout)")
            return 0  # Avoid infinite loop

    # Wait for the echo pulse to end
    pulse_end = time.time()
    timeout = pulse_end + 0.02  # 20ms timeout

    while pi.read(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            print("Echo signal too long (timeout)")
            return 0  # Avoid infinite loop

    # Calculate pulse duration
    pulse_duration = pulse_end - pulse_start

    # Calculate distance (distance = speed × time / 2)
    distance = (SPEED_OF_SOUND * pulse_duration) / 2

    return distance

    
  
def is_distance_stable(target_distance, tolerance=1, checks=6):
    stable_count = 0

    for _ in range(checks):
        time.sleep(0.1)  # Add delay before measuring
        distance = measure_distance()
        
        if distance is None:  # Handle timeout case
            print("Skipping unstable reading due to timeout.")
            continue
        
        if abs(distance - target_distance) <= tolerance:
            stable_count += 1
    
    return stable_count >= (checks // 2)  # At least half the readings must be stable
         
def move_to_car():
    
    while True:
        left_sensor = pi.read(IR_LEFT_FRONT)  # 1 = White, 0 = Black
        right_sensor = pi.read(IR_RIGHT_FRONT)
        distance = measure_distance()
        if distance < 10:  # Stop if an obstacle is within 10 cm
            stop()
            print("Obstacle detected! Waiting...")
            time.sleep(0.1) 
            continue
        if left_sensor == 0 and right_sensor == 0:
            move_forward()
        elif left_sensor == 0 and right_sensor == 1:
            turn_left()
        elif left_sensor == 1 and right_sensor == 0:
            turn_right()
        elif left_sensor == 1 and right_sensor == 1:
            stop()
            break
        time.sleep(0.05)
        
def back_to_charger():
    
    while True:
        left_sensor = pi.read(IR_LEFT_BACK)  # 1 = White, 0 = Black
        right_sensor = pi.read(IR_RIGHT_BACK)

        if left_sensor == 0 and right_sensor == 0:
            move_backward()
        elif left_sensor == 0 and right_sensor == 1:
            turn_left()
        elif left_sensor == 1 and right_sensor == 0:
            turn_right()
        elif left_sensor == 1 and right_sensor == 1:
            stop()
            break
        time.sleep(0.05)
        

def angle_to_pulse(angle):
    return int(MIN_PULSE + (angle / 180.0) * (MAX_PULSE - MIN_PULSE))
    
def find_time(distance, u=13.33, a=-1.52):
    # Calculate the time required to travel a given distance with initial velocity u and acceleration a.
    if a == 0:
        return distance / u if u != 0 else None  # Avoid division by zero
    # Solving quadratic equation: t = (-u + sqrt(u^2 + 2as)) / a
    discriminant = u**2 + 2 * a * distance
    if discriminant < 0:
        return None  # No real solution
    t = (-u + math.sqrt(discriminant)) / a
    return max(t, 0)  # Return the positive root  
    
def calculate_width_difference_cm(port_bbox, frame_width):
    if port_bbox:
        port_center_x = port_bbox.origin_x + (port_bbox.width / 2)
        frame_center_x = frame_width / 2
        width_difference_px = frame_center_x - port_center_x

        # Convert pixel difference to cm using the known width
        if port_bbox.width > 0:
            width_difference_cm = (width_difference_px * PORT_WIDTH_CM) / port_bbox.width
            return width_difference_cm
    return None
    
def align_right(width_difference_cm):
    t=find_time(abs(width_difference_cm)*2)
    print("time",t)
    turn_right()
    time.sleep(.45)# .5 sec for 30 degree rotation
    stop()
    time.sleep(.5)
    move_forward()
    time.sleep(t)
    stop()
    time.sleep(.5)
    turn_left()
    time.sleep(.45)# .5 sec for 30 degree rotation
    stop()
    time.sleep(.5)
    
def align_right_back(width_difference_cm):
    t=find_time(abs(width_difference_cm)*2)
    print("time",t)
    turn_right()
    time.sleep(.45)# .5 sec for 30 degree rotation
    stop()
    time.sleep(.5)
    move_backward()
    time.sleep(t)
    stop()
    time.sleep(.5)
    turn_left()
    time.sleep(.45)# .5 sec for 30 degree rotation
    stop()
    time.sleep(.5)
        
def align_left(width_difference_cm):
    t=find_time(abs(width_difference_cm)*2)
    print("time",t)
    turn_left()
    time.sleep(.45)# .5 sec for 30 degree rotation
    stop()
    time.sleep(.5)
    move_forward()
    time.sleep(t)
    stop()
    time.sleep(.5)
    turn_right()
    time.sleep(.45)# .5 sec for 30 degree rotation
    stop()
    time.sleep(.5)
    
def align_left_back(width_difference_cm):
    t=find_time(abs(width_difference_cm)*2)
    print("time",t)
    turn_left()
    time.sleep(.45)# .5 sec for 30 degree rotation
    stop()
    time.sleep(.5)
    move_backward()
    time.sleep(t)
    stop()
    time.sleep(.5)
    turn_right()
    time.sleep(.45)# .5 sec for 30 degree rotation
    stop()
    time.sleep(.5)  
      
def move_servos(shoulder_angle, elbow_angle, wrist_angle):
    #Move servos to specified angles with micro-adjustments for the wrist
    pi.set_servo_pulsewidth(SHOULDER_SERVO, angle_to_pulse(shoulder_angle))
    pi.set_servo_pulsewidth(ELBOW_SERVO, angle_to_pulse(elbow_angle))
    pi.set_servo_pulsewidth(WRIST_SERVO, angle_to_pulse(wrist_angle))
    
def move_servos_slow(shoulder_angle, elbow_angle, wrist_angle):
    # Get current positions
    current_shoulder = pulse_to_angle(pi.get_servo_pulsewidth(SHOULDER_SERVO))
    current_elbow = pulse_to_angle(pi.get_servo_pulsewidth(ELBOW_SERVO))
    current_wrist = pulse_to_angle(pi.get_servo_pulsewidth(WRIST_SERVO))

    # Move each servo in small steps
    for i in range(0, 101, 3):  # Speed controls smoothness (lower is smoother)
        new_shoulder = current_shoulder + (shoulder_angle - current_shoulder) * i / 100.0
        new_elbow = current_elbow + (elbow_angle - current_elbow) * i / 100.0
        new_wrist = current_wrist + (wrist_angle - current_wrist) * i / 100.0

        pi.set_servo_pulsewidth(SHOULDER_SERVO, angle_to_pulse(new_shoulder))
        pi.set_servo_pulsewidth(ELBOW_SERVO, angle_to_pulse(new_elbow))
        pi.set_servo_pulsewidth(WRIST_SERVO, angle_to_pulse(new_wrist))

        time.sleep(0.02)  # Adjust sleep time for smoother motion

def pulse_to_angle(pulse):
    return (pulse - 500) * 180.0 / 2000
    
def move_parallel(initial_shoulder, initial_elbow, initial_wrist):
    """Move servos while making real-time wrist adjustments to maintain parallel behavior."""
    steps = 100  
    delay = 4 / steps
    move_servos_slow(initial_shoulder,initial_elbow,initial_wrist)
    time.sleep(1)   

    for i in range(steps):
        new_shoulder_angle = initial_shoulder - i * 0.5
        new_elbow_angle = initial_elbow - i * 0.25
        new_wrist_angle =initial_wrist + i * 0.25

        move_servos(new_shoulder_angle, new_elbow_angle, new_wrist_angle)
        time.sleep(delay)
        
def move_back_arm(initial_shoulder, initial_elbow, initial_wrist):
    steps = 100
    delay = 4/ steps  
    for i in range(steps):
        new_shoulder_angle = initial_shoulder - (steps - i - 1) * 0.5
        new_elbow_angle = initial_elbow - (steps - i - 1) * 0.25
        new_wrist_angle =initial_wrist + (steps - i - 1) * 0.25
        move_servos(new_shoulder_angle, new_elbow_angle, new_wrist_angle)
        time.sleep(delay)
    time.sleep(2)
    move_servos_slow(150,175,0)  

def calculate_height_difference_cm(port_bbox, frame_height):
    if port_bbox:
        port_center_y = port_bbox.origin_y + (port_bbox.height / 2)
        frame_center_y = frame_height / 2
        height_difference_px =  frame_center_y - port_center_y
        
        # Convert pixel difference to cm using the known height
        if port_bbox.height > 0:
            height_difference_cm = (height_difference_px * PORT_HEIGHT_CM) / port_bbox.height
            return height_difference_cm
    return None 
    
def find_angles(h):
    solutions = []

    for theta2 in range(0, 181):  # Try all possible elbow angles
        for theta1 in range(0, 181):  # Try all possible shoulder angles
            calc_h = L1 * math.sin(math.radians(theta1)) + L2 * math.sin(math.radians(theta1 + theta2))
            if abs(calc_h - h) < 0.02:  # Small tolerance for precision
                solutions.append((theta1, theta2))

    if len(solutions) >= 2:
        return solutions[0], solutions[-1]  # First = Elbow-Up, Last = Elbow-Down
    return None, None
    
def conneting_charger(port_bbox,height):
    global initial_shoulder
    global initial_elbow
    global initial_wrist

    if port_bbox:
        height_difference_cm = calculate_height_difference_cm(port_bbox, height)
        print(f"Height Difference: {height_difference_cm:.2f} cm")
        h_new=h+height_difference_cm
        (elbow_up, elbow_down) = find_angles(h_new)
        if elbow_up and elbow_down and elbow_down[1]<135:
            print(f"New Angles for h = {h_new} cm:")
            print(f"Elbow-Down Position: θ₁ = {elbow_down[0]}°, θ₂ = {elbow_down[1]}°")
            initial_shoulder = float(200-elbow_down[0])
            initial_elbow = float(elbow_down[1]+45)
            initial_wrist = float((90-((180-elbow_down[1])-elbow_down[0]))-23)
            move_parallel(initial_shoulder, initial_elbow, initial_wrist)
        else:
            print("No valid angles found for the given height.")
    
def run(model: str, camera_id: int, width: int, height: int,flag: bool, num_threads: int,
        enable_edgetpu: bool) -> None:
    """Continuously run inference on images acquired from a USB webcam."""
    global width_difference_cm
    global alignment_threshold
    move_to_car()

    # Variables to calculate FPS
    counter, fps = 0, 0
    start_time = time.time()

    # Initialize the webcam
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Unable to access the camera (ID {camera_id}).")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Visualization parameters
    row_size = 20  # pixels
    left_margin = 24  # pixels
    text_color = (0, 0, 255)  # red
    font_size = 1
    font_thickness = 1
    fps_avg_frame_count = 20

    # Initialize the object detection model
    base_options = core.BaseOptions(
        file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
    detection_options = processor.DetectionOptions(
        max_results=1, score_threshold=0.5)
    options = vision.ObjectDetectorOptions(
        base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)

    # Variables for alignment

    frame_center_x = width / 2
    frame_center_y = height / 2
    x_center = None
    distance=0
    speed_adjust=0
    
    flag2=True
    alignment_stable_count = 0

    # Continuously capture frames from the webcam and run inference
    while True:
        # Capture frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to capture a frame from the camera.")
            break

        # Convert the frame from BGR (OpenCV format) to RGB (required by TensorFlow Lite)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Create a TensorImage object from the captured frame
        input_tensor = vision.TensorImage.create_from_array(rgb_frame)

        counter += 1

        # Run object detection inference
        detection_result = detector.detect(input_tensor)
        
        port_bbox = None
        
        # Extract bounding box information and calculate the center of each box
        for detection in detection_result.detections:
            bbox = detection.bounding_box
            x_center = bbox.origin_x + bbox.width / 2
            y_center = bbox.origin_y + bbox.height / 2
            perceived_width = bbox.width
            
            # Calculate the distance to the object
            distance = measure_distance()
            print(f"Distance to object: {distance:.2f} cm")
            if distance<10:
                alignment_threshold=6
                speed_adjust=0.01
            else:
                alignment_threshold=15
                speed_adjust=0
      
            if not port_bbox or bbox.height > port_bbox.height:
                port_bbox = bbox         
            
            if port_bbox and flag2==True:
                width_difference_cm = calculate_width_difference_cm(port_bbox, width)
                print("width diff = ",width_difference_cm)
                if width_difference_cm <0:
                    align_right(width_difference_cm)
                    flag2=False
                    break
                if width_difference_cm >0:
                    align_left(width_difference_cm)
                    flag2=False
                    break
                       
            if x_center < frame_center_x - alignment_threshold:
                print("Turning left to align...")
                turn_left()
                time.sleep(0.025-speed_adjust)
                stop()
                alignment_stable_count = 0  # Reset stability count

            elif x_center > frame_center_x + alignment_threshold:
                print("Turning right to align...")
                turn_right()
                time.sleep(0.025-speed_adjust)
                stop()
                alignment_stable_count = 0  # Reset stability count
             
            else:
                print("Aligned! Moving forward...")
                alignment_stable_count += 1
                if alignment_stable_count >= 3:  # Ensure alignment is stable for at least 5 frames
                    print("Alignment stable. Moving forward...")
                    if distance > SAFE_DISTANCE:
                        move_forward()
                        time.sleep(0.03-speed_adjust)  # Adjust delay as needed
                        stop()
                    elif distance< SAFE_DISTANCE-1:

                        move_backward()
                        time.sleep(0.03-speed_adjust)
                        stop()

                    else:
                        print("Safe distance reached. Stopping...")
                        stop()
                        if is_distance_stable(SAFE_DISTANCE):
                            flag=True
        if port_bbox and flag==True:
            time.sleep(2)
            conneting_charger(port_bbox, height)
            break
                    
        # Visualize the detections on the frame
        annotated_frame = utils.visualize(frame, detection_result)

        # Draw vertical threshold lines at the screen center ± threshold
        threshold_left = int(frame_center_x - alignment_threshold)
        threshold_right = int(frame_center_x + alignment_threshold)

        cv2.line(annotated_frame, (threshold_left, 0), (threshold_left, height), (0, 255, 0), 2)  # Green line
        cv2.line(annotated_frame, (threshold_right, 0), (threshold_right, height), (0, 255, 0), 2)  # Green line
        cv2.line(annotated_frame, (0,int(frame_center_y)), (width,(int(frame_center_y))), (0, 255, 0), 2)  # Green line
        if x_center is not None:
            cv2.circle(annotated_frame, (int(x_center), height // 2), 5, (0, 0, 255), -1)
        

        # Display alignment guide text
        cv2.putText(annotated_frame, "Align to Center", (threshold_left - 40, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Calculate the FPS
        if counter % fps_avg_frame_count == 0:
            end_time = time.time()
            fps = fps_avg_frame_count / (end_time - start_time)
            start_time = time.time()

        # Display the FPS on the frame
        fps_text = 'FPS = {:.1f}'.format(fps)
        text_location = (left_margin, row_size)
        cv2.putText(annotated_frame, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN, font_size, text_color, font_thickness)

        # Display the annotated frame
        cv2.imshow('Object Detector', annotated_frame)

        # Stop the program if the ESC key is pressed
        if cv2.waitKey(1) == 27:
            break

    # Cleanup
    cap.release()
    stop()


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--model',
        help='Path of the object detection model.',
        required=False,
        default='best.tflite')
    parser.add_argument(
        '--cameraId', help='ID of the webcam.', required=False, type=int, default=0)
    parser.add_argument(
        '--frameWidth',
        help='Width of frame to capture from webcam.',
        required=False,
        type=int,
        default=640)
    parser.add_argument(
        '--frameHeight',
        help='Height of frame to capture from webcam.',
        required=False,
        type=int,
        default=480)
    parser.add_argument(
        '--flag',
        help='flag',
        required=False,
        type=bool,
        default=False)
    parser.add_argument(
        '--numThreads',
        help='Number of CPU threads to run the model.',
        required=False,
        type=int,
        default=4)
    parser.add_argument(
        '--enableEdgeTPU',
        help='Whether to run the model on EdgeTPU.',
        action='store_true',
        required=False,
        default=False)
    args = parser.parse_args()

    run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,bool(args.flag),
        int(args.numThreads), bool(args.enableEdgeTPU))
        
@app.route('/')
def home():
    return render_template("index.html", sensor_value=sensor_value)

@app.route('/action', methods=['POST'])

def action():
    global current_position
    global width_difference_cm
    global initial_shoulder
    global initial_elbow
    global initial_wrist
    command = request.form.get('command')

    if command == "start":
        main()
   
    elif command == "stop":
        move_back_arm(initial_shoulder, initial_elbow, initial_wrist)
        if width_difference_cm > 0:
            align_left_back(width_difference_cm)  
        elif width_difference_cm < 0:
            align_right_back(width_difference_cm)  
        time.sleep(2)
        back_to_charger()    
    elif command == "slot1":
        if current_position == "slot1":
            return "Already at Slot 1"
        else:
            turn_right()
            time.sleep(3.1)
            stop()

        current_position = "slot1"

    elif command == "slot2":
        if current_position == "slot2":
            return "Already at Slot 2"
        else:
            turn_left()
            time.sleep(3.1)
            stop()

        current_position = "slot2"

    return "OK"

@app.route('/data', methods=['POST', 'GET'])
def receive_data():
    global sensor_value
    if request.method == 'POST':
        sensor_value = request.form.get('sensor_value', "No data")
        print(f"Received data: {sensor_value}")
    return jsonify({"status": "success", "sensor_value": sensor_value})
    
if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=True)
    finally:
        cv2.destroyAllWindows()
        pi.stop()
        print("Robot stopped.")

    

