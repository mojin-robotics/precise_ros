#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <precise_driver/pflex_device.h>
#include <precise_driver/precise_tcp_interface.h>
#include <precise_driver/Gripper.h>
#include <ros/ros.h>

#include <mutex>
#include <condition_variable>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <cob_srvs/SetString.h>

namespace precise_driver
{

    class PreciseHWInterface : public ros_control_boilerplate::GenericHWInterface
    {
    public:
        PreciseHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
        ~PreciseHWInterface();

        virtual void init();
        virtual void read(ros::Duration &elapsed_time);
        virtual void write(ros::Duration &elapsed_time);
        virtual void enforceLimits(ros::Duration &period);

    private:
        ros::ServiceServer _init_srv;
        ros::ServiceServer _teachmode_srv;
        ros::ServiceServer _home_srv;
        ros::ServiceServer _power_srv;
        ros::ServiceServer _attach_srv;
        ros::ServiceServer _cmd_srv;
        ros::ServiceServer _open_gripper_srv;
        ros::ServiceServer _close_gripper_srv;

        ros::ServiceClient _switch_controller_srv;

        std::shared_ptr<PFlexDevice> _device;
        Profile _profile;
        int _profile_no;

        std::mutex _mutex_init;
        std::mutex _mutex_write;
        std::condition_variable _cond_init;

        bool _write_enabled;

        void enableWrite(bool value);
        bool isWriteEnabled();

        bool resetController(bool active);

        bool initCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool teachmodeCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool homeCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool powerCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool attachCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool cmdCb(cob_srvs::SetString::Request &req, cob_srvs::SetString::Response &res);

        bool openGripperCB(precise_driver::Gripper::Request &req, precise_driver::Gripper::Response &res);
        bool closeGripperCB(precise_driver::Gripper::Request &req, precise_driver::Gripper::Response &res);
    }; // class

} // namespace precise_driver
