"""mavicpython controller."""


from controller import Robot, DistanceSensor, Motor, LED, Keyboard, InertialUnit, Gyro, GPS, Compass, Camera
import math, sys

def clamp(val, lo, hi):
  if val < lo:
    return lo
  if val > hi:
    return hi
  return val

robot = Robot()
timestep = robot.getBasicTimeStep()

# Get and enable devices
camera = robot.getDevice("camera")
camera.enable(int(timestep))
front_left_led = robot.getDevice("front left led")
front_right_led = robot.getDevice("front right led")
imu = robot.getDevice("inertial unit")
imu.enable(int(timestep))
gps = robot.getDevice("gps")
gps.enable(int(timestep))
compass = robot.getDevice("compass")
compass.enable(int(timestep))
gyro = robot.getDevice("gyro")
gyro.enable(int(timestep))
keyboard = robot.getKeyboard()
keyboard.enable(int(timestep))
camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")

# Get propeller motors and set to velocity mode
front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")
motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]
for m in motors:
  m.setPosition(float('inf'))
  m.setVelocity(1.0)

# Display welcome message
print("Start the drone...\n")

# Wait 1 second
while robot.step(int(timestep)) != -1:
  if robot.getTime() > 1.0:
    break

# Display manual control message.
print("You can control the drone with your keyboard!\n")
print("- 'up': move forward.\n")
print("- 'down': move backward.\n")
print("- 'right': turn right.\n")
print("- 'left': turn left.\n")
print("- 'shift + up': increase the target altitude.\n")
print("- 'shift + down': decrease the target altitude.\n")
print("- 'shift + right': strafe right.\n")
print("- 'shift + left': strafe left.\n")

# Constants, empirically found.
k_vertical_thrust = 68.5  # with this thrust, the drone lifts.
k_vertical_offset = 0.6   # Vertical offset where the robot actually targets to stabilize itself.
k_vertical_p = 3.0        # P constant of the vertical PID.
k_roll_p = 50.0           # P constant of the roll PID.
k_pitch_p = 30.0          # P constant of the pitch PID.

# Variables.
target_altitude = 1.0  # The target altitude. Can be changed by the user.

# Main loop
while robot.step(int(timestep)) != -1:
  time = robot.getTime()  # in seconds.

  # Retrieve robot position using the sensors.
  roll = imu.getRollPitchYaw()[0] + math.pi / 2.0
  pitch = imu.getRollPitchYaw()[1]
  altitude = gps.getValues()[2]
  roll_acceleration = gyro.getValues()[0]
  pitch_acceleration = gyro.getValues()[1]

  # Blink the front LEDs alternatively with a 1 second rate.
  led_state = int(time) % 2
  front_left_led.set(led_state)
  front_right_led.set(not led_state)

  # Stabilize the Camera by actuating the camera motors according to the gyro feedback.
  camera_roll_motor.setPosition(-0.115 * roll_acceleration)
  camera_pitch_motor.setPosition(-0.1 * pitch_acceleration)

  # Transform the keyboard input to disturbances on the stabilization algorithm.
  roll_disturbance = 0.0
  pitch_disturbance = 0.0
  yaw_disturbance = 0.0
  key = keyboard.getKey()
  while key > 0:
    if key == Keyboard.UP:
      pitch_disturbance = 2.0
      break
    elif key == Keyboard.DOWN:
      pitch_disturbance = -2.0
      break
    elif key == Keyboard.RIGHT:
      yaw_disturbance = 1.3
      break
    elif key == Keyboard.LEFT:
      yaw_disturbance = -1.3
      break
    elif key == (Keyboard.SHIFT + Keyboard.RIGHT):
      roll_disturbance = -1.0
      break
    elif key == (Keyboard.SHIFT + Keyboard.LEFT):
      roll_disturbance = 1.0
      break
    elif key == (Keyboard.SHIFT + Keyboard.UP):
      target_altitude = target_altitude + 0.05
      break
    elif key == (Keyboard.SHIFT + Keyboard.DOWN):
      target_altitude = target_altitude - 0.05
      break
    key = keyboard.getKey()

  # Compute the roll, pitch, yaw and vertical inputs.
  roll_input = k_roll_p * clamp(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
  pitch_input = k_pitch_p * clamp(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance
  yaw_input = yaw_disturbance
  clamped_difference_altitude = clamp(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
  vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0)

  # Actuate the motors taking into consideration all the computed inputs.
  front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
  front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
  rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
  rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
  front_left_motor.setVelocity(front_left_motor_input)
  front_right_motor.setVelocity(-1*front_right_motor_input)
  rear_left_motor.setVelocity(-1*rear_left_motor_input)
  rear_right_motor.setVelocity(rear_right_motor_input)

