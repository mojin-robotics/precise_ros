#include <precise_driver/precise_hw_interface.h>
#include <angles/angles.h>

#include <controller_manager_msgs/SwitchController.h>

namespace precise_driver
{
    PreciseHWInterface::PreciseHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
    {
        ros::NodeHandle pnh(nh_, "hardware_interface");
        ros::NodeHandle driver_nh(nh_, "driver");

        std::string ip;
        pnh.param<std::string>("ip_address", ip, ip);
        int control_port = 10100;
        pnh.param<int>("control_port", control_port, control_port);
        int status_port = 10000;
        pnh.param<int>("status_port", status_port, status_port);
        pnh.param<int>("profile_no", _profile_no, _profile_no);
        pnh.param<int>("speed", _profile.speed, _profile.speed);
        pnh.param<int>("speed2", _profile.speed2, _profile.speed2);
        pnh.param<int>("accel", _profile.accel, _profile.accel);
        pnh.param<int>("decel", _profile.decel, _profile.decel);
        pnh.param<double>("accel_ramp", _profile.accel_ramp, _profile.accel_ramp);
        pnh.param<double>("decel_ramp", _profile.decel_ramp, _profile.decel_ramp);
        pnh.param<int>("in_range", _profile.in_range, _profile.in_range);
        pnh.param<int>("straight", _profile.straight, _profile.straight);

        _device.reset(new PFlexDevice(std::make_shared<PreciseTCPInterface>(ip, control_port),
                                std::make_shared<PreciseTCPInterface>(ip, status_port)));

        _init_srv = driver_nh.advertiseService("init", &PreciseHWInterface::initCb, this);
        _teachmode_srv = driver_nh.advertiseService("teach_mode", &PreciseHWInterface::teachmodeCb, this);
        _home_srv = driver_nh.advertiseService("home", &PreciseHWInterface::homeCb, this);
        _power_srv = driver_nh.advertiseService("power", &PreciseHWInterface::powerCb, this);
        _cmd_srv = driver_nh.advertiseService("command", &PreciseHWInterface::cmdCb, this);
        _open_gripper_srv = driver_nh.advertiseService("open_gripper", &PreciseHWInterface::openGripperCB, this);
        _close_gripper_srv = driver_nh.advertiseService("close_gripper", &PreciseHWInterface::closeGripperCB, this);

        _switch_controller_srv = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
    }

    PreciseHWInterface::~PreciseHWInterface()
    {
    }

    void PreciseHWInterface::init()
    {
        //Wait for hardware to init
        std::unique_lock<std::mutex> lock(_mutex_init);
        ROS_INFO("Waiting for robot init");
        _cond_init.wait(lock);

        GenericHWInterface::init();
        std::vector<double> joints = _device->getJointPositions();

        for(size_t i = 0; i < num_joints_; i++)
        {
            joint_position_[i] = joints[i];
        }
        ROS_INFO("PreciseHWInterface Ready.");
    }

    void PreciseHWInterface::read(ros::Duration &elapsed_time)
    {
        std::vector<double> joints = _device->getJointPositions();

        for(size_t i = 0; i < num_joints_; i++)
        {
            joint_position_[i] = joints[i];
        }
    }

    void PreciseHWInterface::write(ros::Duration &elapsed_time)
    {
        // Safety
        enforceLimits(elapsed_time);

        if(isWriteEnabled() && _device->operable())
        {
            //_device->moveJointSpace(_profile_no, joint_position_command_);
            _device->queueJointSpace(_profile_no, joint_position_command_);
        }
    }

    void PreciseHWInterface::enforceLimits(ros::Duration &period)
    {
        pos_jnt_sat_interface_.enforceLimits(period);
    }

    bool PreciseHWInterface::initCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        enableWrite(false);
        if(_device->init(_profile_no, _profile) && _device->home())
        {
            _device->startMoveJThread();
            res.success = true;
            _cond_init.notify_one();
        }
        else
            res.success = false;
        enableWrite(true);
        return true;
    }

    bool PreciseHWInterface::teachmodeCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        enableWrite(false);

        if(req.data)
        {
            res.success = resetController(false) && _device->freeMode(req.data);
        }
        else
        {
            res.success = resetController(true) && _device->freeMode(req.data);
        }

        enableWrite(true);

        return true;
    }

    bool PreciseHWInterface::homeCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        enableWrite(false);

        if(_device->home())
            res.success = true;
        else
            res.success = false;

        enableWrite(true);

        return true;
    }

    bool PreciseHWInterface::powerCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        enableWrite(false);
        if(_device->setHp(req.data, 5))
            res.success = true;
        else
            res.success = false;

        enableWrite(true);
        return true;
    }

    bool PreciseHWInterface::attachCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        enableWrite(false);
        if(_device->attach(req.data))
            res.success = true;
        else
            res.success = false;
        enableWrite(true);
        return true;
    }

    bool PreciseHWInterface::cmdCb(cob_srvs::SetString::Request &req, cob_srvs::SetString::Response &res)
    {
        enableWrite(false);
        res.message = _device->command(req.data);
        res.success = true;

        enableWrite(true);
        return true;
    }

    bool PreciseHWInterface::openGripperCB(precise_driver::Gripper::Request &req, precise_driver::Gripper::Response &res)
    {
        enableWrite(false);
        double range = joint_position_upper_limits_[4] - joint_position_lower_limits_[4];
        int perc = (req.width / range) * 100;
        res.success = _device->graspPlate(perc, req.speed, req.force);
        resetController();
        enableWrite(true);
        return true;
    }

    bool PreciseHWInterface::closeGripperCB(precise_driver::Gripper::Request &req, precise_driver::Gripper::Response &res)
    {
        enableWrite(false);
        double range = joint_position_upper_limits_[4] - joint_position_lower_limits_[4];
        int perc = (req.width / range) * 100;
        res.success = _device->releasePlate(req.width, req.speed, req.force);
        resetController();
        enableWrite(true);
        return true;
    }

    void PreciseHWInterface::enableWrite(bool value)
    {
        std::lock_guard<std::mutex> guard(_mutex_write);
        _write_enabled = value;
    }

    bool PreciseHWInterface::isWriteEnabled()
    {
        bool ret;
        {
            std::lock_guard<std::mutex> guard(_mutex_write);
            ret = _write_enabled;
        }
        return ret;
    }

    bool PreciseHWInterface::resetController(bool active)
    {
        controller_manager_msgs::SwitchController::Request req;
        req.strictness = req.BEST_EFFORT;
        if(active)
            req.start_controllers.push_back("joint_trajectory_controller");
        else
            req.stop_controllers.push_back("joint_trajectory_controller");

        controller_manager_msgs::SwitchController::Response res;
        bool ret = _switch_controller_srv.call(req, res);

        if(! (ret && res.ok))
        {
            ROS_ERROR("Can not switch (start/stop) joint_trajectory_controller");
            return false;
        }

        pos_jnt_sat_interface_.reset();

        for(size_t i = 0; i < num_joints_; ++i)
        {
            joint_position_command_[i] = joint_position_[i];
            ROS_INFO_STREAM("reset joint to: "<<joint_position_[i]);

            try{
                position_joint_interface_.getHandle(joint_names_[i]).setCommand(joint_position_[i]);
            }
            catch(const hardware_interface::HardwareInterfaceException&)
            {
                ROS_ERROR("can not set command for position_joint_jointerface");
                return false;
            }
        }
        return (ret && res.ok);
    }

} // namespace precise_driver
