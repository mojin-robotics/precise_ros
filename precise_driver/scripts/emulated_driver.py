#!/usr/bin/env python

import rospy

from precise_driver.srv import Gripper, GripperRequest, GripperResponse
from precise_driver.srv import Plate, PlateRequest, PlateResponse
from precise_driver.srv import SetFreeMode, SetFreeModeRequest, SetFreeModeResponse
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from cob_srvs.srv import SetString, SetStringResponse


class EmulatedPreciseDriver(object):
    def __init__(self):
        self.init_srv = rospy.Service('/arm/driver/init', Trigger, self.init_cb)
        self.recover_srv = rospy.Service('/arm/driver/recover', Trigger, self.recover_cb)
        self.teachmode_srv = rospy.Service('/arm/driver/teach_mode', SetFreeMode, self.teachmode_cb)
        self.home_srv = rospy.Service('/arm/driver/home', Trigger, self.home_cb)
        self.power_srv = rospy.Service('/arm/driver/power', SetBool, self.power_cb)
        self.command_srv = rospy.Service('/arm/driver/command', SetString, self.command_cb)
        self.grasp_plate_srv = rospy.Service('/arm/driver/grasp_plate', Plate, self.grasp_plate_cb)
        self.release_plate_srv = rospy.Service('/arm/driver/release_plate', Plate, self.release_plate_cb)
        self.gripper_srv = rospy.Service('/arm/driver/gripper', Gripper, self.gripper_cb)

        rospy.loginfo('Kevin EmulatedPreciseDriver is running')

    def init_cb(self, req):
        res = TriggerResponse()
        res.success = True
        return res

    def recover_cb(self, req):
        res = TriggerResponse()
        res.success = True
        return res

    def teachmode_cb(self, req):
        output_message = "Teach mode ACTIVATED!" if req.data else "Teach mode DEACTIVATED!"
        rospy.loginfo("{}".format(output_message))
        res = SetFreeModeResponse()
        res.success = True
        return res

    def home_cb(self, req):
        res = TriggerResponse()
        res.success = True
        return res

    def power_cb(self, req):
        output_message = "Power is ACTIVATED!" if req.data else "Power is DEACTIVATED!"
        rospy.loginfo("{}".format(output_message))
        res = SetBoolResponse()
        res.success = True
        return res

    def command_cb(self, req):
        output_message = "Command is {}".format(req.data)
        rospy.loginfo("{}".format(output_message))
        res = SetStringResponse()
        res.success = True
        return res

    def grasp_plate_cb(self, req):
        res = PlateResponse()
        res.success = True
        return res

    def release_plate_cb(self, req):
        res = PlateResponse()
        res.success = True
        return res

    def gripper_cb(self, req):
        res = GripperResponse()
        res.success = True
        return res

if __name__ == '__main__':
    rospy.init_node('emulated_precise_driver')

    EmulatedPreciseDriver()
    rospy.spin()
