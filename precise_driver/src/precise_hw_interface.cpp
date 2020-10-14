#include <precise_driver/precise_hw_interface.h>
#include <angles/angles.h>

namespace precise_driver
{
    PreciseHWInterface::PreciseHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
    {
        ros::NodeHandle pnh(nh_, "hardware_interface");
        //TODO: init ros stuff

        _init_srv = nh_.advertiseService("init", &PreciseHWInterface::init_cb, this);
        _teachmode_srv = nh_.advertiseService("teach_mode", &PreciseHWInterface::teachmode_cb, this);
        _home_srv = nh_.advertiseService("teach_mode", &PreciseHWInterface::home_cb, this);
        _power_srv = nh_.advertiseService("teach_mode", &PreciseHWInterface::power_cb, this);
        _attach_srv = nh_.advertiseService("attach", &PreciseHWInterface::attach_cb, this);
    }

    void PreciseHWInterface::init()
    {
        GenericHWInterface::init();

        //TODO: open connection to hardware
        //TODO: fill current joint state values with values from hardware

        std::vector<double> joints;
        joints.assign(num_joints_, 0.0);

        for(size_t i = 0; i < num_joints_; i++)
        {
            joint_position_[i] = joints[i];
        }
    }


    void PreciseHWInterface::read(ros::Duration &elapsed_time)
    {
        //TODO: Read current state from hardware
        std::vector<double> joints;
        joints.assign(num_joints_, 0.0);

        for(size_t i = 0; i < num_joints_; i++)
        {
            joint_position_[i] = joints[i];
        }
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

    bool PreciseHWInterface::init_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        return true;
    }

    bool PreciseHWInterface::teachmode_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        return true;
    }

    bool PreciseHWInterface::home_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        return true;
    }

    bool PreciseHWInterface::power_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        return true;
    }

    bool PreciseHWInterface::attach_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        return true;
    }

} // namespace precise_driver
