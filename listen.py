import socket
import struct
import io
import threading
import time
from time import monotonic
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# Network Configuration
HOST = '0.0.0.0'
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002

# Pins
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB = 25
LEFT_MOTOR_IN3 = 23
LEFT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16

# PID Constants (default values, will be overridden by client)
use_PID = 0
# Linear movement PID constants
LINEAR_KP, LINEAR_KI, LINEAR_KD = 0, 0, 0
# Rotation movement PID constants
ROTATION_KP, ROTATION_KI, ROTATION_KD = 0, 0, 0
MAX_CORRECTION = 30  # Maximum PWM correction value

# Motor calibration factors
LEFT_MOTOR_FACTOR = 1.0   # Adjust if left motor is slower/faster
RIGHT_MOTOR_FACTOR = 1.0  # Example: if right motor is 5% faster

# Global variables
running = True
left_pwm, right_pwm = 0, 0
left_count, right_count = 0, 0
prev_left_state, prev_right_state = None, None
use_ramping = True
RAMP_RATE = 250  # PWM units per second (adjust this value to tune ramp speed)
MIN_RAMP_THRESHOLD = 15  # Only ramp if change is greater than this
MIN_PWM_THRESHOLD = 15
current_movement, prev_movement = 'stop', 'stop'
movement_threshold = 50  # PWM difference threshold to distinguish turn from straight

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Motor
    GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENB, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN3, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN4, GPIO.OUT)
    
    # This prevents slight motor jerk when connection is established
    GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_ENB, GPIO.LOW)
    
    # Encoder setup and interrupt (both activated and deactivated)
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)
    
    # Initialize PWM (frequency: 100Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)
    
    # Check for actual state change. Without this, false positive happens due to electrical noise
    # After testing, debouncing not needed
    if (prev_left_state is not None and current_state != prev_left_state):       
        left_count += 1
        prev_left_state = current_state
    
    elif prev_left_state is None:
        # First reading
        prev_left_state = current_state

def right_encoder_callback(channel):
    global right_count, prev_right_state, prev_right_time
    current_state = GPIO.input(RIGHT_ENCODER)
    
    if (prev_right_state is not None and current_state != prev_right_state): 
        right_count += 1
        prev_right_state = current_state
        
    elif prev_right_state is None:
        prev_right_state = current_state
    
def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0

def set_motors(left, right):
    global prev_movement, current_movement
    
    # Pre-Start Kick (Motor Priming), to reduce initial jerk and slight orientation change
    if prev_movement == 'stop' and current_movement in ['forward', 'backward']:
        if current_movement  == 'forward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        elif current_movement == 'backward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        time.sleep(0.05)

    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right * RIGHT_MOTOR_FACTOR, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right) * RIGHT_MOTOR_FACTOR, 100))
    else:
        # when pwm = 0, implement Active Braking mode, better than putting duty cycle to 0 which may cause uneven stopping
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(100)
    
    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left * LEFT_MOTOR_FACTOR, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left) * LEFT_MOTOR_FACTOR, 100))
    else:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
    
    
def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0  # Zero means stop
    elif abs(pwm_value) < min_threshold:
        # Boost small values to minimum threshold, preserving direction
        return min_threshold if pwm_value > 0 else -min_threshold
    else:
        return pwm_value

def pid_control():
    # Applies PID for both forward/backward AND rotation movements with separate parameters
    global left_pwm, right_pwm, left_count, right_count, use_PID
    global LINEAR_KP, LINEAR_KI, LINEAR_KD, ROTATION_KP, ROTATION_KI, ROTATION_KD
    global prev_movement, current_movement
    
    # PID state for linear movement
    linear_integral = 0
    linear_last_error = 0
    
    # PID state for rotation movement
    rotation_integral = 0
    rotation_last_error = 0
    
    last_time = monotonic()
    
    # Ramping variables & params
    ramp_left_pwm = 0
    ramp_right_pwm = 0
    previous_left_target = 0
    previous_right_target = 0
    
    # Encoder reset for turns
    encoder_reset_interval = 0.5  # Reset encoders every 0.5 seconds during turns
    last_encoder_reset_time = monotonic()
    
    while running:          
        current_time = monotonic()
        dt = current_time - last_time
        last_time = current_time
        
        prev_movement = current_movement
        
        # Improved movement detection with threshold to avoid false turn detection
        # Check if both motors are going in the same direction (within threshold)
        if left_pwm == 0 and right_pwm == 0:
            current_movement = 'stop'
        elif (left_pwm > 0 and right_pwm > 0):
            # Both forward - check if they're similar enough for straight line
            if abs(left_pwm - right_pwm) <= movement_threshold:
                current_movement = 'forward'
            else:
                current_movement = 'turn'  # Significant difference = arc turn
        elif (left_pwm < 0 and right_pwm < 0):
            # Both backward - check if they're similar enough for straight line
            if abs(left_pwm - right_pwm) <= movement_threshold:
                current_movement = 'backward'
            else:
                current_movement = 'turn'  # Significant difference = arc turn
        else:
            # Opposite directions or one stopped = definite turn
            current_movement = 'turn'
        
        if not use_PID:
            target_left_pwm = left_pwm
            target_right_pwm = right_pwm
        else:
            if current_movement == 'forward' or current_movement == 'backward':
                # Linear movement PID with LINEAR_KP, LINEAR_KI, LINEAR_KD
                error = left_count - right_count
                proportional = LINEAR_KP * error
                linear_integral += LINEAR_KI * error * dt
                linear_integral = max(-MAX_CORRECTION, min(linear_integral, MAX_CORRECTION))  # Anti-windup
                derivative = LINEAR_KD * (error - linear_last_error) / dt if dt > 0 else 0
                correction = proportional + linear_integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                linear_last_error = error
                            
                if current_movement == 'backward':
                    correction = -correction

                target_left_pwm = left_pwm - correction
                target_right_pwm = right_pwm + correction
                
                # Reset rotation PID state when doing linear movement
                rotation_integral = 0
                rotation_last_error = 0
                
            elif current_movement == 'turn':
                # For turns, periodically reset encoders to prevent unbounded error accumulation
                if current_time - last_encoder_reset_time > encoder_reset_interval:
                    reset_encoder()
                    last_encoder_reset_time = current_time
                    rotation_integral = 0  # Also reset integral when encoders reset
                    rotation_last_error = 0
                
                # Enhanced rotation PID - focus on keeping wheels synchronized
                error = left_count - right_count
                
                proportional = ROTATION_KP * error
                rotation_integral += ROTATION_KI * error * dt
                rotation_integral = max(-MAX_CORRECTION, min(rotation_integral, MAX_CORRECTION))
                derivative = ROTATION_KD * (error - rotation_last_error) / dt if dt > 0 else 0
                correction = proportional + rotation_integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                rotation_last_error = error
                
                # Apply correction based on turn type
                if left_pwm >= 0 and right_pwm >= 0:
                    # Both motors forward - arc turn
                    # Positive error (left ahead) = slow left, speed up right
                    target_left_pwm = left_pwm - correction
                    target_right_pwm = right_pwm + correction
                elif left_pwm <= 0 and right_pwm <= 0:
                    # Both motors backward - arc turn
                    # Positive error (left ahead) = slow left, speed up right
                    target_left_pwm = left_pwm - correction
                    target_right_pwm = right_pwm + correction
                elif left_pwm < 0 and right_pwm > 0:
                    # CCW spot turn: left backward (-), right forward (+)
                    # Positive error (left ahead) = make left less negative, make right less positive
                    target_left_pwm = left_pwm + correction
                    target_right_pwm = right_pwm - correction
                else:
                    # CW spot turn: left forward (+), right backward (-)
                    # Positive error (left ahead) = make left less positive, make right less negative
                    target_left_pwm = left_pwm - correction
                    target_right_pwm = right_pwm + correction
                
                # Reset linear PID state when turning
                linear_integral = 0
                linear_last_error = 0
                
            else:
                # Stop condition - reset all PID states
                linear_integral = 0
                linear_last_error = 0
                rotation_integral = 0
                rotation_last_error = 0
                reset_encoder()
                last_encoder_reset_time = current_time
                target_left_pwm = left_pwm
                target_right_pwm = right_pwm
        
        if use_ramping and use_PID:
            # PWM Ramping Logic
            max_change_per_cycle = RAMP_RATE * dt
            
            # Calculate differences for both motors
            left_diff = target_left_pwm - ramp_left_pwm
            right_diff = target_right_pwm - ramp_right_pwm
            
            # Determine if either motor needs ramping
            left_needs_ramp = abs(left_diff) > MIN_RAMP_THRESHOLD
            right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD
            
            # Check for direction change conditions (but not stops)
            left_direction_change = (target_left_pwm * previous_left_target < 0) and target_left_pwm != 0 and previous_left_target != 0
            right_direction_change = (target_right_pwm * previous_right_target < 0) and target_right_pwm != 0 and previous_right_target != 0
            
            # Apply immediate changes for direction changes only (for safety)
            if left_direction_change:
                ramp_left_pwm = target_left_pwm
            if right_direction_change:
                ramp_right_pwm = target_right_pwm
            
            # Synchronized ramping - both motors ramp together or not at all
            if not left_direction_change and not right_direction_change:
                if left_needs_ramp or right_needs_ramp:
                    
                    # Left motor ramping (including ramp-down to zero)
                    if abs(left_diff) <= max_change_per_cycle:
                        ramp_left_pwm = target_left_pwm  # Close enough, set to target
                    else:
                        # Ramp towards target (up or down)
                        if left_diff > 0:
                            ramp_left_pwm += max_change_per_cycle
                        else:
                            ramp_left_pwm -= max_change_per_cycle
                    
                    # Right motor ramping (including ramp-down to zero)
                    if abs(right_diff) <= max_change_per_cycle:
                        ramp_right_pwm = target_right_pwm  # Close enough, set to target
                    else:
                        # Ramp towards target (up or down)
                        if right_diff > 0:
                            ramp_right_pwm += max_change_per_cycle
                        else:
                            ramp_right_pwm -= max_change_per_cycle
                else:
                    # Neither motor needs ramping - apply targets directly
                    ramp_left_pwm = target_left_pwm
                    ramp_right_pwm = target_right_pwm
            
            # Store previous targets for next iteration
            previous_left_target = target_left_pwm
            previous_right_target = target_right_pwm
        
        else:
            # Ramping disabled - apply target values directly
            ramp_left_pwm = target_left_pwm
            ramp_right_pwm = target_right_pwm
            
        final_left_pwm = apply_min_threshold(ramp_left_pwm, MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)
        
        # Only print during active movement (not stop) and when encoders are moving
        if ramp_left_pwm != 0 or ramp_right_pwm != 0:
            print(f"Mode: {current_movement}, (L_PWM, R_PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), (L_Enc, R_Enc)=({left_count}, {right_count})")
        
        time.sleep(0.01)


def camera_stream_server():
    # Initialize camera
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (640,480)})
    picam2.configure(camera_config)
    picam2.start()
    
    # Create socket for streaming
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, CAMERA_PORT))
    server_socket.listen(1)
    print(f"Camera stream server started on port {CAMERA_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Camera stream client connected")
            
            while running:
                # Capture frame and convert to bytes
                stream = io.BytesIO()
                picam2.capture_file(stream, format='jpeg')
                stream.seek(0)
                jpeg_data = stream.getvalue()
                jpeg_size = len(jpeg_data)
                
                try:
                    client_socket.sendall(struct.pack("!I", jpeg_size))
                    client_socket.sendall(jpeg_data)
                except:
                    print("Camera stream client disconnected")
                    break
                
                # Small delay to avoid hogging CPU
                time.sleep(0.01)
                
        except Exception as e:
            print(f"Camera stream server error: {str(e)}")
        
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
    
    server_socket.close()
    picam2.stop()


def pid_config_server():
    global use_PID, LINEAR_KP, LINEAR_KI, LINEAR_KD, ROTATION_KP, ROTATION_KI, ROTATION_KD
    
    # Create socket for receiving PID configuration
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_CONFIG_PORT))
    server_socket.listen(1)
    print(f"PID config server started on port {PID_CONFIG_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"PID config client connected")
            
            try:
                # Receive PID constants (7 floats: use_PID + 3 linear + 3 rotation)
                data = client_socket.recv(28)
                if data and len(data) == 28:
                    use_PID, LINEAR_KP, LINEAR_KI, LINEAR_KD, ROTATION_KP, ROTATION_KI, ROTATION_KD = struct.unpack("!fffffff", data)
                    if use_PID:
                        print(f"Updated Linear PID: KP={LINEAR_KP}, KI={LINEAR_KI}, KD={LINEAR_KD}")
                        print(f"Updated Rotation PID: KP={ROTATION_KP}, KI={ROTATION_KI}, KD={ROTATION_KD}")
                    else:
                        print("The robot is not using PID.")
                    
                    # Send acknowledgment (1 for success)
                    response = struct.pack("!i", 1)
                else:
                    # Send failure response
                    response = struct.pack("!i", 0)
                
                client_socket.sendall(response)
                    
            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    response = struct.pack("!i", 0)
                    client_socket.sendall(response)
                except:
                    pass
                    
            client_socket.close()
                    
        except Exception as e:
            print(f"PID config server error: {str(e)}")
    
    server_socket.close()
    

def wheel_server():
    global left_pwm, right_pwm, running, left_count, right_count
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    print(f"Wheel server started on port {WHEEL_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Wheel client connected")
            
            while running:
                try:
                    # Receive speed (4 bytes for each value)
                    data = client_socket.recv(8)
                    if not data or len(data) != 8:
                        print("Wheel client sending speed error")
                        break
                    
                    # Unpack speed values and convert to PWM
                    left_speed, right_speed = struct.unpack("!ff", data)
                    # print(f"Received wheel: left_speed={left_speed:.4f}, right_speed={right_speed:.4f}")
                    left_pwm, right_pwm = left_speed*100, right_speed*100
                    
                    # Send encoder counts back
                    response = struct.pack("!ii", left_count, right_count)
                    client_socket.sendall(response)
                    
                except Exception as e:
                    print(f"Wheel client disconnected")
                    break
                    
        except Exception as e:
            print(f"Wheel server error: {str(e)}")
        
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
    
    server_socket.close()


def main():
    try:
        setup_gpio()
        
        # Start PID control thread
        pid_thread = threading.Thread(target=pid_control)
        pid_thread.daemon = True
        pid_thread.start()
        
        # Start camera streaming thread
        camera_thread = threading.Thread(target=camera_stream_server)
        camera_thread.daemon = True
        camera_thread.start()
        
        # Start PID configuration server thread
        pid_config_thread = threading.Thread(target=pid_config_server)
        pid_config_thread.daemon = True
        pid_config_thread.start()
        
        # Start wheel server (main thread)
        wheel_server()
        
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
