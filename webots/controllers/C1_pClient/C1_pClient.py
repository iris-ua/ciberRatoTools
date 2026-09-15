"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the e-puck motors and distance sensors.
cruiseVelocity = 3.0
num_dist_sensors = 8

# Get left and right wheel motors.
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

# Get ground sensors.
ground_sensors = [robot.getDevice('gs' + str(x)) for x in range(3)]  # ground sensors
list(map((lambda s: s.enable(timeStep)), ground_sensors))  # Enable all ground sensors

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))


# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(cruiseVelocity)
rightMotor.setVelocity(cruiseVelocity)

# Get camera.
camera = robot.getDevice("camera")
camera.enable(timeStep)

white_threshold = 500

while robot.step(timeStep) != -1:

    ground_sensor_values = [g.getValue() for g in ground_sensors]
    
    print(ground_sensor_values)

    if ground_sensor_values[2] > white_threshold:
        print('turn left')
        leftMotor.setVelocity ( 0.1*cruiseVelocity)
        rightMotor.setVelocity( 1.2*cruiseVelocity)
    elif ground_sensor_values[0] > white_threshold:
        print('turn right')
        leftMotor.setVelocity ( 1.2*cruiseVelocity)
        rightMotor.setVelocity( 0.1*cruiseVelocity)
    else:
        print('go')
        leftMotor.setVelocity(cruiseVelocity)
        rightMotor.setVelocity(cruiseVelocity)

