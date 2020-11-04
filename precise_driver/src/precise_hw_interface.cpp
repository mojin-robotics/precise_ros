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
        pnh.param<int>("profile_no", profile_no_, profile_no_);
        pnh.param<int>("speed", profile_.speed, profile_.speed);
        pnh.param<int>("speed2", profile_.speed2, profile_.speed2);
        pnh.param<int>("accel", profile_.accel, profile_.accel);
        pnh.param<int>("decel", profile_.decel, profile_.decel);
        pnh.param<double>("accel_ramp", profile_.accel_ramp, profile_.accel_ramp);
        pnh.param<double>("decel_ramp", profile_.decel_ramp, profile_.decel_ramp);
        pnh.param<int>("in_range", profile_.in_range, profile_.in_range);
        pnh.param<int>("straight", profile_.straight, profile_.straight);

        device_.reset(new Device(std::make_shared<TCPClient>(ip, control_port),
                                std::make_shared<TCPClient>(ip, status_port)));

        init_srv_ = driver_nh.advertiseService("init", &PreciseHWInterface::initCb, this);
        teachmode_srv_ = driver_nh.advertiseService("teach_mode", &PreciseHWInterface::teachmodeCb, this);
        home_srv_ = driver_nh.advertiseService("home", &PreciseHWInterface::homeCb, this);
        power_srv_ = driver_nh.advertiseService("power", &PreciseHWInterface::powerCb, this);
        cmd_srv_ = driver_nh.advertiseService("command", &PreciseHWInterface::cmdCb, this);
        grasp_plate_srv_ = driver_nh.advertiseService("grasp_plate", &PreciseHWInterface::graspPlateCB, this);
        release_plate_srv_ = driver_nh.advertiseService("release_plate", &PreciseHWInterface::releasePlateCB, this);
        gripper_srv_ = driver_nh.advertiseService("gripper", &PreciseHWInterface::gripperCB, this);

        switch_controller_srv_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
    }

    PreciseHWInterface::~PreciseHWInterface()
    {
    }

    void PreciseHWInterface::init()
    {
        //Wait for hardware to init
        std::unique_lock<std::mutex> lock(mutex_init_);
        ROS_INFO("Waiting for robot init");
        cond_init_.wait(lock);

        GenericHWInterface::init();
        std::vector<double> joints = device_->getJointPositions();

        for(size_t i = 0; i < num_joints_; i++)
        {
            joint_position_[i] = joints[i];
        }
        ROS_INFO("PreciseHWInterface Ready.");
    }

    void PreciseHWInterface::read(ros::Duration &elapsed_time)
    {
        std::vector<double> joints = device_->getJointPositions();

        for(size_t i = 0; i < num_joints_; i++)
        {
            joint_position_[i] = joints[i];
        }
    }

    void PreciseHWInterface::write(ros::Duration &elapsed_time)
    {
        // Safety
        enforceLimits(elapsed_time);

        if(isWriteEnabled() && device_->operational())
        {
            //device_->moveJointPosition(_profile_no, joint_position_command_);
            device_->queueJointPosition(profile_no_, joint_position_command_);
        }
    }

    void PreciseHWInterface::enforceLimits(ros::Duration &period)
    {
        pos_jnt_sat_interface_.enforceLimits(period);
    }

    bool PreciseHWInterface::initCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        enableWrite(false);
        if(device_->init(profile_no_, profile_) && device_->home())
        {
            device_->startCommandThread();
            res.success = true;
            cond_init_.notify_one();
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
            res.success = resetController(false) && device_->freeMode(req.data);
        }
        else
        {
            res.success = resetController(true) && device_->freeMode(req.data);
        }

        enableWrite(true);

        return true;
    }

    bool PreciseHWInterface::homeCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        enableWrite(false);

        if(device_->home())
            res.success = true;
        else
            res.success = false;

        enableWrite(true);

        return true;
    }

    bool PreciseHWInterface::powerCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        enableWrite(false);
        if(device_->setHp(req.data, 5))
            res.success = true;
        else
            res.success = false;

        enableWrite(true);
        return true;
    }

    bool PreciseHWInterface::attachCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        enableWrite(false);
        if(device_->attach(req.data))
            res.success = true;
        else
            res.success = false;
        enableWrite(true);
        return true;
    }

    bool PreciseHWInterface::cmdCb(cob_srvs::SetString::Request &req, cob_srvs::SetString::Response &res)
    {
        enableWrite(false);
        res.message = device_->command(req.data);
        res.success = true;

        enableWrite(true);
        return true;
    }

    bool PreciseHWInterface::graspPlateCB(precise_driver::Plate::Request &req, precise_driver::Plate::Response &res)
    {
        enableWrite(false);
        resetController(false);
        //                                m to mm
        res.success = device_->graspPlate(req.width*1000, req.speed, req.force);
        resetController(true);
        enableWrite(true);
        return true;
    }

    bool PreciseHWInterface::releasePlateCB(precise_driver::Plate::Request &req, precise_driver::Plate::Response &res)
    {
        enableWrite(false);
        resetController(false);
        res.success = device_->releasePlate(req.width*1000, req.speed);
        res.success = device_->waitForEom();
        resetController(true);
        enableWrite(true);
        return true;
    }

    bool PreciseHWInterface::gripperCB(precise_driver::Gripper::Request &req, precise_driver::Gripper::Response &res)
    {
        enableWrite(false);
        resetController(false);
        double pos;
        if(req.mode == req.MODE_PERCENT)
        {
            //linear interval transformation
            //f(x) = min + ((max - min)/(b-a)) * (x - a)
            double a, b, min, max;
            a = 0.0; b = 1.0;
            max = joint_position_lower_limits_[4]; min = joint_position_upper_limits_[4];
            pos = min + ((max - min)/(b-a)) * (req.command - a);
        }
        else if(req.mode == req.MODE_POSITION)
        {
            pos = req.command;
        }
        else
        {
            ROS_ERROR_STREAM("Gripper mode "<<req.mode<<" is not supported.");
            res.success = false;
            return true;
        }

        std::vector<double> joints = joint_position_;
        joints[4] = pos;
        bool ret;
        ret = device_->moveJointPosition(profile_no_, joints);
        ret &= device_->waitForEom();
        res.success = ret;

        resetController(true);
        enableWrite(true);
        return true;
    }

    void PreciseHWInterface::enableWrite(bool value)
    {
        std::lock_guard<std::mutex> guard(mutex_write_);
        write_enabled_ = value;
    }

    bool PreciseHWInterface::isWriteEnabled()
    {
        bool ret;
        {
            std::lock_guard<std::mutex> guard(mutex_write_);
            ret = write_enabled_;
        }
        return ret;
    }

    bool PreciseHWInterface::resetController(bool active)
    {
        device_->clearCommandQueue();
        controller_manager_msgs::SwitchController::Request req;
        req.strictness = req.BEST_EFFORT;
        if(active)
            req.start_controllers.push_back("joint_trajectory_controller");
        else
            req.stop_controllers.push_back("joint_trajectory_controller");

        controller_manager_msgs::SwitchController::Response res;
        bool ret = switch_controller_srv_.call(req, res);

        if(! (ret && res.ok))
        {
            ROS_ERROR("Can not switch (start/stop) joint_trajectory_controller");
            return false;
        }

        for(size_t i = 0; i < num_joints_; ++i)
        {
            joint_position_command_[i] = joint_position_[i];

            try{
                position_joint_interface_.getHandle(joint_names_[i]).setCommand(joint_position_[i]);
            }
            catch(const hardware_interface::HardwareInterfaceException&)
            {
                ROS_ERROR("can not set command for position_joint_jointerface");
                return false;
            }
        }
        pos_jnt_sat_interface_.reset();
        return (ret && res.ok);
    }

} // namespace precise_driver
