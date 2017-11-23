#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import os, time
import thread

from math import pi as PI, degrees, radians, sin, cos
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial

import roslib

from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from tf.broadcaster import TransformBroadcaster

#from dashgo_driver.srv import *
#from SrvInt32.srv import *
from std_srvs.srv import Trigger, TriggerResponse

from sensor_msgs.msg import Range
from std_msgs.msg import Int16
 
ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]
ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]
ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


SERVO_MAX = 180
SERVO_MIN = 0

class Arduino:
    ''' Configuration Parameters
    '''    
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12
    
    def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5):
        
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.
    
        # Keep things thread safe
        self.mutex = thread.allocate_lock()
            
        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS
        
        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS
    
    def connect(self):
        try:
            print "Connecting to Arduino on port", self.port, "..."
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            test = self.get_baud()
            if test != self.baudrate:
                time.sleep(1)
                test = self.get_baud()   
                if test != self.baudrate:
                    raise SerialException
            print "Connected at", self.baudrate
            print "Arduino is ready."

        except SerialException:
            print "Serial Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)

    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd + '\r')

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            c = self.port.read(1)
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')

        return value
            
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None

    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            values = self.recv(self.timeout * self.N_ANALOG_PORTS).split()
            return map(int, values)
        except:
            return []

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            value = self.recv(self.timeout)
            while attempts < ntries and (value == '' or value == 'Invalid Command' or value == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    value = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            value = None
        
        self.mutex.release()
        return int(value)

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            values = self.recv_array()
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    values = self.recv_array()
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []
        
        try:
            values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values
        
    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            ack = self.recv(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    ack = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
            attempts += 1
        except:
            self.mutex.release()
            print "execute_ack exception when executing", cmd
            print sys.exc_info()
            return 0
        
        self.mutex.release()
        return ack == 'OK'   
    
    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print "Updating PID parameters"
        cmd = 'u ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko)
        self.execute_ack(cmd)                          

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return int(self.execute('b'));

    def get_encoder_counts(self):
        values = self.execute_array('e')
        if len(values) != 2:
            print "Encoder count was not 2"
            raise SerialException
            return None
        else:
            return values

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')
    
    def drive(self, right, left):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        return self.execute_ack('m %d %d' %(right, left))
    
    def drive_m_per_s(self, right, left):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(right_ticks_per_loop , left_ticks_per_loop )
        
    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)

    def ping(self):
        values = self.execute_array('p')
        if len(values) != 5:
            print "ping count was not 5"
            raise SerialException
            return None
        else:
            return values

    def get_voltage(self):
        return self.execute('v');

    def get_emergency_button(self):
        return self.execute('j');

    def get_pidin(self):
        values = self.execute_array('i')
        if len(values) != 2:
            print "get_pidin count was not 2"
            raise SerialException
            return None
        else:
            return values

    def get_pidout(self):
        values = self.execute_array('f')
        if len(values) != 2:
            print "get_pidout count was not 2"
            raise SerialException
            return None
        else:
            return values


""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, arduino, base_frame):
        self.arduino = arduino
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False
        self.useImu = rospy.get_param("~useImu", False)
        self.useSonar = rospy.get_param("~useSonar", False)
                 
        pid_params = dict()
        pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", "") 
        pid_params['wheel_track'] = rospy.get_param("~wheel_track", "")
        pid_params['encoder_resolution'] = rospy.get_param("~encoder_resolution", "") 
        pid_params['gear_reduction'] = rospy.get_param("~gear_reduction", 1.0)
        pid_params['Kp'] = rospy.get_param("~Kp", 20)
        pid_params['Kd'] = rospy.get_param("~Kd", 12)
        pid_params['Ki'] = rospy.get_param("~Ki", 0)
        pid_params['Ko'] = rospy.get_param("~Ko", 50)
        
        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
        
        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)
            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * PI)
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.l_wheel_mult = 0
        self.r_wheel_mult = 0
                        
        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now

        # Subscriptions
        #rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        rospy.Subscriber("smoother_cmd_vel", Twist, self.cmdVelCallback)
        
        # Clear any old odometry info
        self.arduino.reset_encoders()
        
        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")

        self.lEncoderPub = rospy.Publisher('Lencoder', Int16, queue_size=5)
        self.rEncoderPub = rospy.Publisher('Rencoder', Int16, queue_size=5)
        self.lVelPub = rospy.Publisher('Lvel', Int16, queue_size=5)
        self.rVelPub = rospy.Publisher('Rvel', Int16, queue_size=5)

        ## sonar 
        self.sonar0_pub = rospy.Publisher('sonar0', Range, queue_size=5)
        self.sonar1_pub = rospy.Publisher('sonar1', Range, queue_size=5)
        self.sonar2_pub = rospy.Publisher('sonar2', Range, queue_size=5)
        self.sonar3_pub = rospy.Publisher('sonar3', Range, queue_size=5)
        self.sonar4_pub = rospy.Publisher('sonar4', Range, queue_size=5)
        
        self.safe_ranger_0 = 0.3
        self.safe_ranger_1 = 0.6
        self.safe_ranger_2 = 1.0
        self.front_ranger_l = 10.0
        self.front_ranger_r = 10.0
    
        self.voltage_bool = False
        self.voltage_val = 0
        self.voltage_status_service = rospy.Service('voltage_status', Trigger, self.handle_voltage_status) 
	self.voltage_pub = rospy.Publisher('voltage_value', Int16, queue_size=30)
   
        self.emergencybt_bool = False
        self.emergencybt_val = 0
        self.emergencybt_status_service = rospy.Service('emergencybt_status', Trigger, self.handle_emergencybt_status) 
        self.emergencybt_pub = rospy.Publisher('emergencybt_status', Int16, queue_size=30)

        rospy.Subscriber("is_passed", Int16, self.isPassedCallback)
        self.isPassed = True
      
    def handle_voltage_status(self, req):
        try:
             self.voltage_bool = True
             self.voltage_val  = self.arduino.get_voltage()
        except:
             self.voltage_bool = False
             self.voltage_val  = 0
             rospy.logerr("get_voltage_status exception ")
        return TriggerResponse(self.voltage_bool, str(self.voltage_val*10))

    def handle_emergencybt_status(self, req):
        try:
             self.emergencybt_bool = True
             self.emergencybt_val = self.arduino.get_emergency_button()
        except:
             self.emergencybt_bool = False
             self.emergencybt_val  = 0
             rospy.logerr("get_emergencybt_status exception ")
        return TriggerResponse(self.emergencybt_bool, str(self.emergencybt_val))
 
    
    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True
        
        if missing_params:
            os._exit(1)
                
        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']
        
        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']
        
        self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            if (self.useSonar == True) :
                try:
                    r0, r1, r2, r3, r4= self.arduino.ping()
                    rospy.loginfo("r0: " + str(r0)+"r1: " + str(r1) + "r2: " + str(r2) + "r3: " + str(r3)+ "r4: " + str(r4))
                    #rospy.loginfo("r0: " + str(r0) + "r3: " + str(r3))
                    sonar0_range = Range()
                    sonar0_range.header.stamp = now
                    sonar0_range.header.frame_id = "/sonar0"
                    sonar0_range.radiation_type = Range.ULTRASOUND
                    sonar0_range.min_range = 0.05
                    sonar0_range.max_range = 4.0
                    sonar0_range.range = r0/100.0
                    self.sonar0_pub.publish(sonar0_range)
                    sonar1_range = Range()
                    sonar1_range.header.stamp = now
                    sonar1_range.header.frame_id = "/sonar1"
                    sonar1_range.radiation_type = Range.ULTRASOUND
                    sonar1_range.min_range = 0.05
                    sonar1_range.max_range = 4.0
                    sonar1_range.range = r1/100.0
                    self.sonar1_pub.publish(sonar1_range)
                    sonar2_range = Range()
                    sonar2_range.header.stamp = now
                    sonar2_range.header.frame_id = "/sonar2"
                    sonar2_range.radiation_type = Range.ULTRASOUND
                    sonar2_range.min_range = 0.05
                    sonar2_range.max_range = 4.0
                    sonar2_range.range = r2/100.0
                    self.sonar2_pub.publish(sonar2_range)
                    sonar3_range = Range()
                    sonar3_range.header.stamp = now
                    sonar3_range.header.frame_id = "/sonar3"
                    sonar3_range.radiation_type = Range.ULTRASOUND
                    sonar3_range.min_range = 0.05
                    sonar3_range.max_range = 4.0
                    sonar3_range.range = r3/100.0
                    self.sonar3_pub.publish(sonar3_range)
                    sonar4_range = Range()
                    sonar4_range.header.stamp = now
                    sonar4_range.header.frame_id = "/sonar4"
                    sonar4_range.radiation_type = Range.ULTRASOUND
                    sonar4_range.min_range = 0.05
                    sonar4_range.max_range = 4.0
                    sonar4_range.range = r4/100.0
                    self.sonar4_pub.publish(sonar4_range)
                    if(sonar0_range.range>=0.05) and (sonar0_range.range<=4.0):
                        self.front_ranger_l = sonar0_range.range
                    else:
                        self.front_ranger_l = 10.0
                    if(sonar1_range.range>=0.05) and (sonar1_range.range<=4.0):
                        self.front_ranger_r = sonar1_range.range
                    else:
                        self.front_ranger_r = 10.0
                except:
                    self.front_ranger_l = 10.0
                    self.front_ranger_r = 10.0
                    self.bad_encoder_count += 1
                    rospy.logerr("ping exception count: " + str(self.bad_encoder_count))
                    return

	    try:
	        self.voltage_val  = self.arduino.get_voltage()*10
		#print "voltage_val=",self.voltage_val
		self.voltage_pub.publish(self.voltage_val)
		#print "publish voltage_val is",self.voltage_val
	    except:
		self.voltage_pub.publish(-1)
		#rospy.logerr("get voltage value error")
		#return

	    try:
	        self.emergencybt_val  = self.arduino.get_emergency_button()
	        #print "emergencybt_val=",self.emergencybt_val
	        self.emergencybt_pub.publish(self.emergencybt_val)
	        #print "publish emergencybt_val is",self.emergencybt_val
	    except:
		self.emergencybt_pub.publish(-1)
	        #rospy.logerr("get emergencybt status error")
	        #return
 
            try:
                left_enc, right_enc = self.arduino.get_encoder_counts()
                #rospy.loginfo("left_enc: " + str(left_enc)+"right_enc: " + str(right_enc))
                self.lEncoderPub.publish(left_enc)
                self.rEncoderPub.publish(right_enc)
            except:
                self.bad_encoder_count += 1
                rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count))
                return
                            
            dt = now - self.then
            self.then = now
            dt = dt.to_sec()
            
            # Calculate odometry
            if self.enc_left == None:
                dright = 0
                dleft = 0
            else:
                if (left_enc < self.encoder_low_wrap and self.enc_left > self.encoder_high_wrap) :
                    self.l_wheel_mult = self.l_wheel_mult + 1     
                elif (left_enc > self.encoder_high_wrap and self.enc_left < self.encoder_low_wrap) :
                    self.l_wheel_mult = self.l_wheel_mult - 1
                else:
                     self.l_wheel_mult = 0
                if (right_enc < self.encoder_low_wrap and self.enc_right > self.encoder_high_wrap) :
                    self.r_wheel_mult = self.r_wheel_mult + 1     
                elif (right_enc > self.encoder_high_wrap and self.enc_right < self.encoder_low_wrap) :
                    self.r_wheel_mult = self.r_wheel_mult - 1
                else:
                     self.r_wheel_mult = 0
                #dright = (right_enc - self.enc_right) / self.ticks_per_meter
                #dleft = (left_enc - self.enc_left) / self.ticks_per_meter
                dleft = 1.0 * (left_enc + self.l_wheel_mult * (self.encoder_max - self.encoder_min)-self.enc_left) / self.ticks_per_meter 
                dright = 1.0 * (right_enc + self.r_wheel_mult * (self.encoder_max - self.encoder_min)-self.enc_right) / self.ticks_per_meter 

            self.enc_right = right_enc
            self.enc_left = left_enc
            
            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track
            vxy = dxy_ave / dt
            vth = dth / dt
                
            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
            if (dth != 0):
                self.th += dth 
    
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)
    
            # Create the odometry transform frame broadcaster.
            if (self.useImu == False) :
                self.odomBroadcaster.sendTransform(
                  (self.x, self.y, 0), 
                  (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                  rospy.Time.now(),
                  self.base_frame,
                  "odom"
                )
    
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth

            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE
            # todo sensor_state.distance == 0
            #if self.v_des_left == 0 and self.v_des_right == 0:
            #    odom.pose.covariance = ODOM_POSE_COVARIANCE2
            #    odom.twist.covariance = ODOM_TWIST_COVARIANCE2
            #else:
            #    odom.pose.covariance = ODOM_POSE_COVARIANCE
            #    odom.twist.covariance = ODOM_TWIST_COVARIANCE

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_des_left = 0
                self.v_des_right = 0
                
            if self.v_left < self.v_des_left:
                self.v_left += self.max_accel
                if self.v_left > self.v_des_left:
                    self.v_left = self.v_des_left
            else:
                self.v_left -= self.max_accel
                if self.v_left < self.v_des_left:
                    self.v_left = self.v_des_left
            
            if self.v_right < self.v_des_right:
                self.v_right += self.max_accel
                if self.v_right > self.v_des_right:
                    self.v_right = self.v_des_right
            else:
                self.v_right -= self.max_accel
                if self.v_right < self.v_des_right:
                    self.v_right = self.v_des_right
            self.lVelPub.publish(self.v_left)
            self.rVelPub.publish(self.v_right)            

            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                self.arduino.drive(self.v_left, self.v_right)
                
            self.t_next = now + self.t_delta
            
    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0)
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s


        if (self.useSonar == True) :
            if((self.front_ranger_l<=self.safe_ranger_0)or(self.front_ranger_r<=self.safe_ranger_0)) and (x>0):
                x=0
            elif((self.front_ranger_l<=self.safe_ranger_1)or(self.front_ranger_r<=self.safe_ranger_1)) and (x>0.05):
                x=0.05
            #elif((self.front_ranger_l<=self.safe_ranger_2)or(self.front_ranger_r<=self.safe_ranger_2)) and (x>0.15):
             #   x=0.15

        if not self.isPassed and x>0 :
            x = 0

        if x == 0:
            # Turn in place
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
        self.v_des_left = int(left * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.arduino.PID_RATE)

    def isPassedCallback(self, msg):
        if(msg.data>2):
            self.isPassed = False
        else:
            self.isPassed = True

class ArduinoROS():
    def __init__(self):
        rospy.init_node('Arduino', log_level=rospy.DEBUG)
                
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.        
        self.sensorstate_rate = int(rospy.get_param("~sensorstate_rate", 10))
        
        self.use_base_controller = rospy.get_param("~use_base_controller", False)
        
        
        # Set up the time for publishing the next SensorState message
        now = rospy.Time.now()
        self.t_delta_sensors = rospy.Duration(1.0 / self.sensorstate_rate)
        self.t_next_sensors = now + self.t_delta_sensors
        
        # Initialize a Twist message
        self.cmd_vel = Twist()
  
        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout)
        
        # Make the connection
        self.controller.connect()
        
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
     
        # Reserve a thread lock
        mutex = thread.allocate_lock()
              
        # Initialize the base controller if used
        if self.use_base_controller:
            self.myBaseController = BaseController(self.controller, self.base_frame)
    
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
                    
            if self.use_base_controller:
                mutex.acquire()
                self.myBaseController.poll()
                mutex.release()
            r.sleep()
    
    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Arduino Node...")
        
if __name__ == '__main__':
    myArduino = ArduinoROS()

