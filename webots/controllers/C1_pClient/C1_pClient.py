"""my_controller controller."""

import numpy
import xml.etree.ElementTree as ET
import atexit


# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

CELLROWS=7
CELLCOLS=14
CELL_SIZE = 0.15
KP = 10.0
MAX_SPEED = 6.27
WHEEL_RADIUS = 0.025
AXLE_LENGTH = 0.045

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1
class MyRob:
    def __init__(self):
        # create the Robot instance.
        self.robot = Robot()

        # Get simulation step length.
        self.timeStep = int(self.robot.getBasicTimeStep())

        # Constants of the e-puck motors and distance sensors.
        self.cruiseVelocity = 4.0
        self.num_dist_sensors = 8

        # Get left and right wheel motors.
        self.leftMotor = self.robot.getDevice("left wheel motor")
        self.rightMotor = self.robot.getDevice("right wheel motor")

        # Get frontal distance sensors.
        self.dist_sensors = [self.robot.getDevice('ps' + str(x)) for x in range(self.num_dist_sensors)]  # distance sensors
        list(map((lambda s: s.enable(self.timeStep)), self.dist_sensors))  # Enable all distance sensors

        # Disable motor PID control mode.
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))


        # Set the initial velocity of the left and right wheel motors.
        self.driveMotors(0.0, 0.0)

        #adding helpers!

    


        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timeStep)

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timeStep)

        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timeStep)

        # Step simulation to initialize sensors.
        self.step()

        self.abs_ref_pos = numpy.array(self.gps.getValues())
        print("Initial position:", self.abs_ref_pos)

        # settle-and-log configuration
        self.settle_time_s = 0.2
        self.step_count = 0
        self.loc_log = open("localization.csv", "w")
        self.loc_log.write("step,command," + ",".join([f"p{i}" for i in range(CELLROWS*CELLCOLS)]) + "\n")
        atexit.register(self.loc_log.close)

    def step(self):
        self.robot.step(self.timeStep)
        self.cur_pos = numpy.array(self.gps.getValues())
        self.cur_dir = (-numpy.arctan2(self.compass.getValues()[1], self.compass.getValues()[0]) + numpy.pi/2) % (2 * numpy.pi)
        self.cur_dir = (self.cur_dir + numpy.pi) % (2 * numpy.pi) - numpy.pi

    def driveMotors(self, leftSpeed, rightSpeed):
        self.leftMotor.setVelocity(max(min(leftSpeed,MAX_SPEED),-MAX_SPEED))
        self.rightMotor.setVelocity(max(min(rightSpeed,MAX_SPEED),-MAX_SPEED))

    def _wrap_pi(self, a):
        return (a + numpy.pi) % (2 * numpy.pi) - numpy.pi

    def send_motor_command(self, left_speed, right_speed, duration_s):
        self.driveMotors(left_speed, right_speed)
        elapsed = 0.0
        dt = self.timeStep / 1000.0
        while elapsed < duration_s:
            self.step()
            elapsed += dt
        self.driveMotors(0.0, 0.0)

    def execute_command(self, command):
        target_dirs = {'N': numpy.pi/2, 'E': 0.0, 'S': -numpy.pi/2, 'W': numpy.pi}
        if command not in target_dirs:
            return

        # 1) Rotate to target heading
        target_dir = target_dirs[command]
        delta = self._wrap_pi(target_dir - self.cur_dir)

        w = min(self.cruiseVelocity, MAX_SPEED)   # wheel angular speed (rad/s)
        v_w = w * WHEEL_RADIUS                    # wheel linear rim speed (m/s)

        if v_w > 1e-6 and abs(delta) > 1e-3:
            t_rot = abs(delta) * AXLE_LENGTH / (2.0 * v_w)
            sgn = 1.0 if delta > 0.0 else -1.0
            self.send_motor_command(-sgn * w, sgn * w, t_rot)
            self.step()

        # 2) Drive forward one cell
        d = CELL_SIZE
        if v_w > 1e-6:
            t_fwd = d / v_w
        self.send_motor_command(w, w, t_fwd)  
        self.step()

        # Stop and wait for sensors to stabilize
        self.driveMotors(0.0, 0.0)
        self.wait_for(self.settle_time_s)

        # Log localization probabilities
        probs = self.localization_probabilities()
        self.log_localization(command, probs)

    def wait_for(self, seconds):
        elapsed = 0.0
        dt = self.timeStep / 1000.0
        while elapsed < seconds:
            self.step()
            elapsed += dt

    def localization_probabilities(self):
        gx = int(round((self.cur_pos[0] - self.abs_ref_pos[0]) / CELL_SIZE))
        gy = int(round((self.cur_pos[1] - self.abs_ref_pos[1]) / CELL_SIZE))
        probs = [0.0] * (CELLROWS * CELLCOLS)
        if 0 <= gx < CELLCOLS and 0 <= gy < CELLROWS:
            idx = gy * CELLCOLS + gx
            probs[idx] = 1.0
        return probs

    def log_localization(self, command, probs):
        self.step_count += 1
        line = f"{self.step_count},{command}," + ",".join(f"{p:.6f}" for p in probs) + "\n"
        self.loc_log.write(line)
        self.loc_log.flush()


    def rotate(self, target_dir):
        #print("Rotating to direction:", target_dir)
        while(numpy.abs(target_dir - self.cur_dir) > 0.01):
            angle_error = target_dir - self.cur_dir
            angle_error = (angle_error + numpy.pi) % (2 * numpy.pi) - numpy.pi
            #print("Current direction:", self.cur_dir*180/numpy.pi, " Target direction    :", target_dir*180/numpy.pi, " Angle error:", angle_error*180/numpy.pi)

            self.driveMotors(-KP * angle_error, +KP * angle_error)

            self.step()
        self.driveMotors(0.0, 0.0)

    def move_to(self, target_pos):
        #print("Moving to position:", target_pos, " from ", self.cur_pos)

        target_dir = numpy.arctan2(target_pos[1] - self.cur_pos[1], target_pos[0] - self.cur_pos[0])
        target_dir = (target_dir + numpy.pi/4)//(numpy.pi/2) * numpy.pi/2  # snap to 90 degrees

        #print("Target direction:", target_dir*180/numpy.pi, " from ", self.cur_dir*180/numpy.pi)

        if(numpy.abs(target_dir - self.cur_dir) > 0.1):
            self.rotate(target_dir)

        dist = numpy.linalg.norm(target_pos - self.cur_pos)
        #print("Distance to target:", dist)
        while(dist > 0.005):

            target_pos_dir = target_pos + CELL_SIZE * numpy.array([numpy.cos(target_dir), numpy.sin(target_dir), 0.0])
            angle_error = numpy.arctan2(target_pos_dir[1] - self.cur_pos[1],
                                        target_pos_dir[0] - self.cur_pos[0]) - self.cur_dir
            angle_error = (angle_error + numpy.pi) % (2 * numpy.pi) - numpy.pi

            #print("Current norm position:", (self.cur_pos - self.abs_ref_pos)/CELL_SIZE, " Target norm position:", (target_pos-self.abs_ref_pos)/CELL_SIZE, 
            #    " Distance:", dist, " Target Angle Pos:", (target_pos_dir-self.abs_ref_pos)/CELL_SIZE, " Current Angle:", self.cur_dir*180/numpy.pi)

            #print("Angle error:", angle_error*180/numpy.pi)

            self.driveMotors( self.cruiseVelocity - KP * angle_error,
                             self.cruiseVelocity + KP * angle_error)   

            self.step()
            dist = numpy.linalg.norm(target_pos - self.cur_pos)
        self.driveMotors(0.0, 0.0)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))


# main 
if __name__ == '__main__':

    # open commands file
    commands_file = open("commands.txt", "r")

    myrob = MyRob()

    mapc = Map("../C1_supervisor/C1-lab.xml")
    myrob.setMap(mapc.labMap)
    myrob.printMap()
  
    target_pos = myrob.abs_ref_pos.copy()

    while myrob.step() != -1:
        command = commands_file.readline().strip()
        if not command:
            continue
        print ("Executing command:", command)
        myrob.execute_command(command)
  