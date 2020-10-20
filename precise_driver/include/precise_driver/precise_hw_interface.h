#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <precise_driver/pflex_device.h>
#include <precise_driver/precise_tcp_interface.h>
#include <ros/ros.h>

#include <mutex>
#include <condition_variable>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

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

        std::shared_ptr<PFlexDevice> _device;
        Profile _profile;
        int _profile_no;

        std::mutex _mutex_init;
        std::condition_variable _cond_init;

        bool init_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool teachmode_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool home_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool power_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool attach_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    }; // class

} // namespace precise_driver
