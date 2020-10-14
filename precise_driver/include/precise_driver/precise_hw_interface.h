#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

namespace precise_driver
{

    class PreciseHWInterface : public ros_control_boilerplate::GenericHWInterface
    {
    public:
        PreciseHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

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


        bool init_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool teachmode_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool home_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool power_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool attach_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    }; // class

} // namespace precise_driver
