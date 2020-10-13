#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <std_msgs/Float64.h>
#include <map>

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

    }; // class

} // namespace precise_driver
