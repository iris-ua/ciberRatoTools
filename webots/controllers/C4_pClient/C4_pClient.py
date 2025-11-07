"""sample2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get left and right wheel motors.
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

# Get frontal distance sensors.
num_dist_sensors = 8
dist_sensors = [robot.getDevice('ps' + str(x)) for x in range(num_dist_sensors)]  # distance sensors
list(map((lambda s: s.enable(timestep)), dist_sensors))  # Enable all distance sensors

# Get camera and enable it
camera = robot.getDevice("camera")
camera.enable(timestep)

# Get compass and enable it
compass = robot.getDevice('compass')
compass.enable(timestep)

# Get left and right wheel encoders.
leftEncoder = robot.getDevice("left wheel sensor")
rightEncoder = robot.getDevice("right wheel sensor")
leftEncoder.enable(timestep)
rightEncoder.enable(timestep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set the initial velocity of the left and right wheel motors.
cruiseVelocity = 3.0
leftMotor.setVelocity(cruiseVelocity)
rightMotor.setVelocity(cruiseVelocity)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    frame = camera.getImage()

    print('compass', compass.getValues())
    print("Encoders: left = ", leftEncoder.getValue(), " right = ", rightEncoder.getValue())
    print("Distance sensors:", [f'{ds.getValue():6.3f}' for ds in dist_sensors])
    print(robot.getCustomData())

