"""sample2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
gps = robot.getDevice('gps')
gps.enable(timestep)

camera = robot.getDevice('camera')
camera.enable(timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    #if(robot.getTime()>10.0):
    #    exit(0)


    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    frame = camera.getImage()
    print(gps.getValues())
    print(robot.getCustomData())

# Enter here exit cleanup code.
