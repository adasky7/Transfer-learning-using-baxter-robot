#!/usr/bin/env python

#this code is for demo at ISTeC building launch. The robot is directed to perform a task and joint positions are recorded.
#Afterwards a file is played to read all joint positions recorded

import argparse
import sys
import numpy
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import (Image, )
import sensor_msgs.msg
import std_srvs.srv
from baxter_core_msgs.srv import ( CloseCamera,
                                   ListCameras,
                                   OpenCamera)

from std_msgs.msg import (
    UInt16,
)

import baxter_interface

from baxter_interface import CHECK_VERSION
from baxter_interface.camera import CameraController
from baxter_examples import JointRecorder

class Puppeteer(object):

    def __init__(self, limb, filename, rate, amplification=1.0):
        """
        Puppets one arm with the other.

        @param limb: the control arm used to puppet the other
        @param amplification: factor by which to amplify the arm movement
        """
        puppet_arm = {"left": "right", "right": "left"}
        self._control_limb = limb
        self._puppet_limb = puppet_arm[limb]
        self._control_arm = baxter_interface.limb.Limb(self._control_limb)
        self._puppet_arm = baxter_interface.limb.Limb(self._puppet_limb)
        self._amp = amplification
        
        #new parameters

        self._start_time = rospy.get_time()
        self._filename = filename
        self._rate = rospy.Rate(rate)

        self._limb_left = baxter_interface.Limb("left")
        self._limb_right = baxter_interface.Limb("right")
        self._gripper_left = baxter_interface.Gripper("left", CHECK_VERSION)
        self._gripper_right = baxter_interface.Gripper("right", CHECK_VERSION)
        self._io_left_lower = baxter_interface.DigitalIO('left_lower_button')
        self._io_left_upper = baxter_interface.DigitalIO('left_upper_button')
        self._io_right_lower = baxter_interface.DigitalIO('right_lower_button')
        self._io_right_upper = baxter_interface.DigitalIO('right_upper_button')
        self.cvbridge = cv_bridge.CvBridge()
        #self.cv_image1 = cv.CreateImage((960, 600), 8, 3)
        #self.cv_image2 = cv.CreateImage((960, 600), 8, 3)

        # Verify Grippers Have No Errors and are Calibrated
        if self._gripper_left.error():
            self._gripper_left.reset()
        if self._gripper_right.error():
            self._gripper_right.reset()
        if (not self._gripper_left.calibrated() and
            self._gripper_left.type() != 'custom'):
            self._gripper_left.calibrate()
        if (not self._gripper_right.calibrated() and
            self._gripper_right.type() != 'custom'):
            self._gripper_right.calibrate()
        #new parameters end
        
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()


        #publish camera images to screen
        self.pub2head = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=2)
        self.display_image('ntu_logo1.jpg')
        
        #self.reset_cameras()
        #self.open_camera(self._control_limb, 640, 400)
        #self.open_camera(self._puppet_limb, 480, 300)

        #self.sub_to_camera(self._control_limb)
        #self.sub_to_camera(self._puppet_limb)
    #new
    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def reset_cameras(self):
        reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
        rospy.wait_for_service('camera/reset', timeout=10)
        reset_srv()
        print "Camera reset successful"

    def open_camera(self, camera, x_res, y_res):
        if camera == "left":
            cam = CameraController('left_hand_camera')
        elif camera == "right":
            cam = CameraController('right_hand_camera')
        else:
            sys.exit("Error - open_camera -Invalid camera")
        # set camera parameters '-1' default params
        cam.resolution = [x_res, y_res]
        cam.exposure = -1
        cam.gain = -1
        cam.white_balance_blue = -1
        cam.white_balance_green = -1
        cam.white_balance_red = -1
        #cam.window = [200, 150]
        cam.open()
        print camera, "Camera is opened"

    def callback1(self, img):
        self.pub2head.publish(img)

    def callback2(self, img):
        self.pub2head.publish(img)

    def sub_to_camera(self, camera):
        if camera == "left":
            cam_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            cam_str = "/cameras/right_hand_camera/image"
        else:
            sys.exit("Error - sub_to_camera - Invalid camera")

        if camera =="left":
            self.sub = rospy.Subscriber(cam_str, Image, self.callback1)
        elif camera == "right":
            self.sub = rospy.Subscriber(cam_str, Image, self.callback2)

    def display_image(self, img_path):
        img = cv2.imread(img_path)
        img = cv2.resize(img, (1024, 600))
        pub_msg = self.cvbridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.pub2head.publish(pub_msg)
        #rospy.sleep(2)
    #new

    def _reset_control_modes(self):
        rate = rospy.Rate(100)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._control_arm.exit_control_mode()
            self._puppet_arm.exit_control_mode()
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._control_arm.move_to_neutral()
        self._puppet_arm.move_to_neutral()
        self.display_image('ntu_logo1.jpg')

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        #if not self._init_state:
        #    print("Disabling robot...")
        #    self._rs.disable()
        return True

    def puppet(self):
        """

        """
        self.set_neutral()
        rate = rospy.Rate(100)
        
        ###add neww
        if self._filename:
        ###new end
            control_joint_names = self._control_arm.joint_names()
            puppet_joint_names = self._puppet_arm.joint_names()

            print ("Puppeting:\n"
                  "  Grab %s cuff and move arm.\n"
                  "  Press Ctrl-C to stop...") % (self._control_limb,)
            ##neww
            joints_left = self._limb_left.joint_names()
            joints_right = self._limb_right.joint_names()
            with open(self._filename, 'w') as f:
                f.write('time,')
                f.write(','.join([j for j in joints_left]) + ',')
                f.write('left_gripper,')
                f.write(','.join([j for j in joints_right]) + ',')
                f.write('right_gripper\n')
            ###new end
                while not rospy.is_shutdown():
                    cmd = {}
                    for idx, name in enumerate(puppet_joint_names):
                        v = self._control_arm.joint_velocity(
                            control_joint_names[idx])
                        if name[-2:] in ('s0', 'e0', 'w0', 'w2'):
                            v = -v
                        cmd[name] = v * self._amp
                    self._puppet_arm.set_joint_velocities(cmd)

                    ##neww
                    # Look for gripper button presses
                    if self._io_left_lower.state:
                        self._gripper_left.open()
                        self._gripper_right.open()
                    elif self._io_left_upper.state:
                        self._gripper_left.close()
                        self._gripper_right.close()
                    if self._io_right_lower.state:
                        self._gripper_right.open()
                        self._gripper_left.open()
                    elif self._io_right_upper.state:
                        self._gripper_right.close()
                        self._gripper_left.close()

                    angles_left = [self._limb_left.joint_angle(j)
                                   for j in joints_left]
                    angles_right = [self._limb_right.joint_angle(j)
                                    for j in joints_right]

                    f.write("%f," % (self._time_stamp(),))

                    f.write(','.join([str(x) for x in angles_left]) + ',')
                    f.write(str(self._gripper_left.position()) + ',')

                    f.write(','.join([str(x) for x in angles_right]) + ',')
                    f.write(str(self._gripper_right.position()) + '\n')

                    self._rate.sleep()
                    ##end new

                    rate.sleep()

def main():
    """RSDK Joint Velocity Example: Puppet

    Mirrors the joint velocities measured on one arm as commands to
    the other arm. Demonstrates the use of Joint Velocity Control mode.

    Run this example, passing the 'puppeteer' limb as an argument,
    then move that limb around in zero-g mode to see the joint
    velocities mirrored out on the other arm.
    """
    max_gain = 3.0
    min_gain = 0.1

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        "-l", "--limb", required=True, choices=['left', 'right'],
        help="specify the puppeteer limb (the control limb)"
    )

    #
    required.add_argument(
        "-f", "--file", dest="filename", required=True,
        help="the file name to record to"
    )
    parser.add_argument(
        '-r', '--record-rate', type=int, default=100, metavar='RECORDRATE',
        help='rate at which to record (default: 100)'
    )
    #

    parser.add_argument(
        "-a", "--amplification", type=float, default=1.0,
        help=("amplification to apply to the puppeted arm [%g, %g]"
              % (min_gain, max_gain))
    )

    args = parser.parse_args(rospy.myargv()[1:])
    if (args.amplification < min_gain or max_gain < args.amplification):
        print("Exiting: Amplification must be between: [%g, %g]" %
              (min_gain, max_gain))
        return 1

    print("Initializing node... ")
    rospy.init_node("My_Demo_Node")

    puppeteer = Puppeteer(args.limb, args.filename, args.record_rate, args.amplification)
    rospy.on_shutdown(puppeteer.clean_shutdown)
    puppeteer.puppet()

    print("Done.")
    return 0

if __name__ == '__main__':
    sys.exit(main())
