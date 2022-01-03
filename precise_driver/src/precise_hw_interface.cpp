#include <precise_driver/precise_hw_interface.h>

#include <controller_manager_msgs/SwitchController.h>

namespace precise_driver
{
    PreciseHWInterface::PreciseHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : ros_control_boilerplate::GenericHWInterface(nh, urdf_model), doosan_hack_enabled_(false)
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

        if(pnh.hasParam("post_init_configuration"))
        {
            pnh.getParam("post_init_configuration", post_init_configuration_);
            if(post_init_configuration_.size() != joint_names_.size())
            {
                ROS_ERROR_STREAM_NAMED("precise_hw_interface", "post_init_configuration size "<<post_init_configuration_.size()<<" does not match joint_names size " <<joint_names_.size());
                post_init_configuration_.clear();
            }
        }

        device_.reset(new Device(std::make_shared<TCPClient>(ip, control_port),
                                std::make_shared<TCPClient>(ip, status_port)));

        init_srv_ = driver_nh.advertiseService("init", &PreciseHWInterface::initCb, this);
        recover_srv_ = driver_nh.advertiseService("recover", &PreciseHWInterface::recoverCb, this);
        teachmode_srv_ = driver_nh.advertiseService("teach_mode", &PreciseHWInterface::teachmodeCb, this);
        home_srv_ = driver_nh.advertiseService("home", &PreciseHWInterface::homeCb, this);
        power_srv_ = driver_nh.advertiseService("power", &PreciseHWInterface::powerCb, this);
        cmd_srv_ = driver_nh.advertiseService("command", &PreciseHWInterface::cmdCb, this);
        grasp_plate_srv_ = driver_nh.advertiseService("grasp_plate", &PreciseHWInterface::graspPlateCB, this);
        release_plate_srv_ = driver_nh.advertiseService("release_plate", &PreciseHWInterface::releasePlateCB, this);
        gripper_srv_ = driver_nh.advertiseService("gripper", &PreciseHWInterface::gripperCB, this);

        switch_controller_srv_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

        diagnostic_updater_.setHardwareID("precise_driver");
        diagnostic_updater_.add("precise_driver", this, &PreciseHWInterface::produce_diagnostics);
        diagnostic_timer_ = driver_nh.createTimer(ros::Duration(1.0), &PreciseHWInterface::diagnostics_timer_thread, this);

        //Doosan like hack
        pnh.param<bool>("doosan_hack_enabled", doosan_hack_enabled_, doosan_hack_enabled_);
        sub_follow_joint_goal = driver_nh.subscribe<control_msgs::FollowJointTrajectoryActionGoal>
                                    ("/arm/joint_trajectory_controller/follow_joint_trajectory/goal", 1,
                                    &PreciseHWInterface::followJointTrajectoryActionGoalCB, this);
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
        ros::Time start = ros::Time::now();
        std::vector<double> joints = device_->getJointPositions();

        for(size_t i = 0; i < num_joints_; i++)
        {
            joint_position_[i] = joints[i];
        }
        device_->updateRobotState();
        ros::Time end = ros::Time::now();
        ROS_DEBUG_STREAM_NAMED("precise_hw_interface","duration read: "<<(end-start).toSec());
        ROS_DEBUG_STREAM_NAMED("precise_hw_interface","elapsed read: "<<elapsed_time.toSec());
    }

    void PreciseHWInterface::write(ros::Duration &elapsed_time)
    {
        ros::Time start = ros::Time::now();
        // Safety
        enforceLimits(elapsed_time);

        if(isWriteEnabled() && device_->is_operational())
        {
            //device_->moveJointPosition(_profile_no, joint_position_command_);
            device_->queueJointPosition(profile_no_, joint_position_command_);
        }
        ros::Time end = ros::Time::now();
        ROS_DEBUG_STREAM_NAMED("precise_hw_interface","duration write: "<<(end-start).toSec());
        ROS_DEBUG_STREAM_NAMED("precise_hw_interface","elapsed write: "<<elapsed_time.toSec());
    }

    void PreciseHWInterface::enforceLimits(ros::Duration &period)
    {
        pos_jnt_sat_interface_.enforceLimits(period);
    }

    bool PreciseHWInterface::initCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        if(device_->is_init())
        {
            std::string msg = "already initialized";
            ROS_INFO_STREAM_NAMED("precise_hw_interface",msg);
            res.success = true;
            res.message = msg;
        }
        else
        {
            enableWrite(false);
            if(device_->init(profile_no_, profile_))
            {
                if(device_->home())
                {
                    bool ret = true;
                    if(post_init_configuration_.size() > 0)
                    {
                        ret = device_->moveJointPosition(profile_no_, post_init_configuration_);
                        ret &= device_->waitForEom();
                    }

                    if(!ret)
                    {
                        std::string msg = "Failed to move to post init configuration";
                        ROS_ERROR_STREAM_NAMED("precise_hw_interface",msg);
                        res.success = false;
                        res.message = msg;
                    }
                    else
                    {
                        device_->startCommandThread();
                        std::string msg = "successfully initialized";
                        ROS_INFO_STREAM_NAMED("precise_hw_interface",msg);
                        res.success = true;
                        res.message = msg;
                        cond_init_.notify_one();
                    }
                }
                else
                {
                    std::string msg = "device home failed";
                    ROS_ERROR_STREAM_NAMED("precise_hw_interface",msg);
                    res.success = false;
                    res.message = msg;
                }
            }
            else
            {
                std::string msg = "device init failed";
                ROS_ERROR_STREAM_NAMED("precise_hw_interface",msg);
                res.success = false;
                res.message = msg;
            }
            enableWrite(true);
        }
        return true;
    }

    bool PreciseHWInterface::recoverCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        if(!device_->is_init())
        {
            std::string msg = "not yet initialized";
            ROS_ERROR_STREAM_NAMED("precise_hw_interface",msg);
            res.success = false;
            res.message = msg;
        }
        else
        {
            //Wait for hardware to init
            ROS_INFO("Try recover arm");
            enableWrite(false);
            resetController(false);

            if(device_->recover())
            {
                std::string msg = "recover successful";
                ROS_INFO_STREAM_NAMED("precise_hw_interface",msg);
                res.success = true;
                res.message = msg;
            }
            else
            {
                std::string msg = "recover failed";
                ROS_ERROR_STREAM_NAMED("precise_hw_interface",msg);
                res.success = false;
                res.message = msg;
            }

            resetController(true);
            enableWrite(true);
        }
        return true;
    }

    bool PreciseHWInterface::teachmodeCb(precise_driver::SetFreeMode::Request &req, precise_driver::SetFreeMode::Response &res)
    {
        // Check bitmask size
        if (req.data && req.axes > 31)
        {
            res.success = false;
            res.message = "axes is interpreted as bitmask and must be in [0, 31]";
            return true;
        }

        enableWrite(false);

        if(req.data)
        {
            res.success = resetController(false) && device_->freeMode(req.data, req.axes);
        }
        else
        {
            res.success = resetController(true) && device_->freeMode(req.data, req.axes);
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
        // m to mm
        res.success = device_->graspPlate(req.width*1000, req.speed, req.force);
        //hack
        res.success = true;
        ros::Duration(0.25).sleep();
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
        //hack
        res.success = true;
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
        if(doosan_hack_enabled_)
            return false;
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

    void PreciseHWInterface::followJointTrajectoryActionGoalCB(const control_msgs::FollowJointTrajectoryActionGoalConstPtr &msg)
    {
        for(auto point : msg->goal.trajectory.points)
        {
            bool ret;
            point.positions.push_back(joint_position_.back());
            ret = device_->moveJointPosition(profile_no_, point.positions);
        }
    }

    void PreciseHWInterface::diagnostics_timer_thread(const ros::TimerEvent& event)
    {
        diagnostic_updater_.update();
    }

    void PreciseHWInterface::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if(device_->is_operational())
        {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Driver operational");
        }
        else
        {
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Driver NOT operational");
        }
        stat.add("is_init", device_->is_init());
        stat.add("is_operational", device_->is_operational());
        device_->fill_diagnostics(stat);
    }

} // namespace precise_driver
