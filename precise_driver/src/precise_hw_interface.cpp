#include <precise_driver/precise_hw_interface.h>
#include <angles/angles.h>

namespace precise_driver
{
    PreciseHWInterface::PreciseHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
    {
        ros::NodeHandle pnh(nh_, "hardware_interface");
    }

    void PreciseHWInterface::init()
    {
        GenericHWInterface::init();
    }


    void PreciseHWInterface::read(ros::Duration &elapsed_time)
    {
        //TODO: Read current state from hardware
    }

    void PreciseHWInterface::write(ros::Duration &elapsed_time)
    {
        // Safety
        enforceLimits(elapsed_time);

        //TODO: Write desired state to hardware
    }

    void PreciseHWInterface::enforceLimits(ros::Duration &period)
    {
        pos_jnt_sat_interface_.enforceLimits(period);
    }

} // namespace precise_driver
