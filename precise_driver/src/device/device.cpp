#include <precise_driver/device/device.h>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

//TODO: decide on methods return values. Either return parsed response or pass result variables as reverences
//and give the return values a success state

namespace precise_driver
{
    Device::Device(std::shared_ptr<TCPClient> connection,
                            std::shared_ptr<TCPClient> status_connection)
    : is_attached_(false), is_hp_(false), is_homed_(false), is_teachmode_(false), sys_state_(-1)
    {
        connection_ = connection;
        status_connection_ = status_connection;
        command_queue_.setMaxSize(2);
    }

    Device::~Device()
    {
        std::cout<<"exiting"<<std::endl;
        command_queue_.push(std::string("nop"));
        command_thread_.join();
        setHp(false);
        exit();
    }

    bool Device::init(const int profile_no, const Profile profile)
    {
        ROS_INFO("initializing...");

        //connect to robot
        try{
            connection_->connect();
            status_connection_->connect();
        }
        catch(boost::system::system_error &e)
        {
            ROS_ERROR_STREAM(e.what());
            return false;
        }

        ROS_DEBUG_NAMED("device","selecting Robot 1 on status connection");
        status_connection_->send("selectRobot 1");
        ROS_DEBUG_NAMED("device","Robot 1 selected");

        ROS_DEBUG_NAMED("device","setting Mode to 0");
        setMode(0);

        //test connection
        ROS_DEBUG_NAMED("device","testing connection...");
        if(!nop())
        {
            ROS_ERROR("connection test failed");
            return false;
        }
        ROS_DEBUG_NAMED("device","success");

        sys_state_ = getSysState(true);
        ROS_DEBUG_STREAM_NAMED("device","SysState is: "<<sys_state_);

        ROS_DEBUG_NAMED("device","detach");
        is_attached_ = attach(false);

        ROS_DEBUG_NAMED("device","set High Power");
        if(!setHp(true, 10))
        {
            ROS_ERROR("Could not Power Robot");
            return false;
        }

        setProfile(profile_no, profile);
        is_attached_ = attach(true);

        sys_state_ = getSysState(true);
        ROS_DEBUG_STREAM_NAMED("device","SysState is: "<<sys_state_);

        ROS_INFO("initialized");

        return is_init();
    }

    bool Device::home()
    {
        std::stringstream ss;
        ss << "home";
        Response res = connection_->send(ss.str());

        {
            std::lock_guard<std::mutex> guard(mutex_state_data_);
            is_homed_ = res.success;
        }

        return res.success;
    }

    bool Device::halt()
    {
        std::stringstream ss;
        ss << "halt";
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    bool Device::recover()
    {
        nop();
        bool ret = setHp(true, 5);
        ret &= attach(true);
        return is_operational();
    }

    bool Device::nop()
    {
        std::stringstream ss;
        ss << "nop";
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    bool Device::exit()
    {
        std::stringstream ss;
        ss << "exit";
        Response res = connection_->send(ss.str());
        Response res2 = status_connection_->send(ss.str());
        return (res.error == 0) && (res2.error == 0);
    }

    bool Device::updateRobotState()
    {
        std::stringstream ss;
        ss << "hp";
        Response res = status_connection_->send(ss.str());
        bool hp = static_cast<bool>(std::stoi(res.message));
        ss.str("");
        ss.clear();
        ss << "sysState";
        res = status_connection_->send(ss.str());
        int sys_state;
        if(res.message!="")
            sys_state = std::stoi(res.message);

        std::lock_guard<std::mutex> guard(mutex_state_data_);
        {
            is_hp_ = hp;
            sys_state_ = sys_state;
        }
        return true;
    }

    bool Device::setHp(const bool enabled, const int timeout)
    {
        std::stringstream ss;
        if(timeout != 0)
            ss << "hp " << static_cast<int>(enabled) << " " << timeout;
        else
            ss << "hp " << static_cast<int>(enabled);

        Response res = connection_->send(ss.str());
        {
            std::lock_guard<std::mutex> guard(mutex_state_data_);
            is_hp_ = res.success && enabled;
        }

        return (res.error == 0);
    }

    bool Device::getHp()
    {
        std::stringstream ss;
        ss << "hp";
        bool is_hp = false;
        Response res = connection_->send(ss.str());
        {
            std::lock_guard<std::mutex> guard(mutex_state_data_);
            is_hp = static_cast<bool>(std::stoi(res.message));
        }
        return is_hp;
    }

    int Device::getSysState(const bool mute)
    {
        std::stringstream ss;
        ss << "sysState " << static_cast<int>(mute);
        Response res = status_connection_->send(ss.str());
        if(res.message!="")
        {
            {
                std::lock_guard<std::mutex> guard(mutex_state_data_);
                sys_state_ = std::stoi(res.message);
            }
            return sys_state_;
        }
        else
            return res.error;
    }

    bool Device::selectRobot(const int robot)
    {
        std::stringstream ss;
        if(robot!=-1)
            ss << "selectRobot "<< robot;
        else
            ss << "selectRobot";
        Response res = connection_->send(ss.str());

        return (res.error == 0);
    }

    bool Device::attach(const bool flag)
    {
        std::stringstream ss;
        ss << "attach "<<static_cast<int>(flag);
        Response res = connection_->send(ss.str());

        {
            std::lock_guard<std::mutex> guard(mutex_state_data_);
            is_attached_ = res.success && flag;
        }

        return (res.error == 0);
    }

    //TODO: There is a freeMode command described the TCS Documentation
    // axes is interpreted as bitmask:
    //   All axes free:                11111 = 31
    //   All axes except z-Axis free:  11110 = 30
    //   All axes except gripper free: 01111 = 15
    bool Device::freeMode(const bool enabled, const int axes)
    {
        std::stringstream ss;
        if(enabled)
            ss << "zeroTorque " << static_cast<int>(enabled) << " " << axes;
        else
            ss << "zeroTorque " << static_cast<int>(enabled);

        Response res = connection_->send(ss.str());

        {
            std::lock_guard<std::mutex> guard(mutex_state_data_);
            is_teachmode_ = res.success && enabled;
        }

        return (res.error == 0);
    }

    bool Device::is_operational()
    {
        bool ret;
        {
            std::lock_guard<std::mutex> guard(mutex_state_data_);
            ret = is_attached_ && is_homed_ && is_hp_ && !is_teachmode_ && (sys_state_ == 21);
        }
        return ret;
    }

    bool Device::is_init()
    {
        bool ret;
        {
            std::lock_guard<std::mutex> guard(mutex_state_data_);
            ret = is_attached_ && is_homed_ && (sys_state_ != -1);
        }
        return ret;
    }

    int Device::getMode()
    {
        std::stringstream ss;
        ss << "mode";
        Response res = connection_->send(ss.str());
        return std::stoi(res.message);
    }

    bool Device::setMode(const int mode)
    {
        std::stringstream ss;
        ss << "mode " << mode;
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    Profile Device::getProfile(const int profile_no)
    {
        std::stringstream ss;
        ss << "profile " << profile_no;
        Response res = connection_->send(ss.str());
        ss.clear();
        ss.str(res.message);
        Profile profile;
        ss >> profile.speed >> profile.speed2 >> profile.accel >> profile.decel >>
            profile.accel_ramp >> profile.decel_ramp >> profile.in_range >> profile.straight;

        return profile;
    }

    bool Device::setProfile(const int profile_no, const Profile& profile)
    {
        std::stringstream ss;
        ss << "profile " << profile_no << " "
            << profile.speed << " "
            << profile.speed2 << " "
            << profile.accel << " "
            << profile.decel << " "
            << profile.accel_ramp << " "
            << profile.decel_ramp << " "
            << profile.in_range << " "
            << profile.straight;

        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    bool Device::setBase(const geometry_msgs::Pose& pose)
    {
        std::stringstream ss;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);


        ss << "setBase " << pose.position.x << " "
                        << pose.position.y << " "
                        << pose.position.z << " "
                        << yaw;

        Response res = connection_->send(ss.str());

        //TODO: find correct pflex internal base position that fits to the urdf description
        return (res.error == 0);
    }

    geometry_msgs::Pose Device::getBase()
    {
        std::stringstream ss;
        ss << "setBase";
        Response res = connection_->send(ss.str());
        ss.clear();
        ss.str(res.message);
        geometry_msgs::Pose pose;
        ss >> pose.position.x >> pose.position.y >> pose.position.z;
        double yaw;
        ss >> yaw;

        tf::Quaternion q;
        q.setRPY(0,0,yaw);
        tf::quaternionTFToMsg(q, pose.orientation);

        return pose;
    }

    //TODO: is this important to us? do we need to know the weight of a plate?
    bool Device::setPayload(const int payload)
    {
        std::stringstream ss;
        ss << "payload " << payload;
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    int Device::getPayload()
    {
        std::stringstream ss;
        ss << "payload";
        Response res = connection_->send(ss.str());
        return std::stoi(res.message);
    }

    bool Device::setSpeed(const int profile_no, const int speed)
    {
        std::stringstream ss;
        ss << "speed " << profile_no << " " << speed;
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    int Device::getSpeed(const int profile_no)
    {
        std::stringstream ss;
        ss << "speed " << profile_no;
        Response res = connection_->send(ss.str());
        return std::stoi(res.message);
    }

    std::vector<double> Device::getJointPositions()
    {
        std::stringstream ss;
        ss << "wherej";
        Response res = status_connection_->send(ss.str());
        ss.clear();
        ss.str(res.message);
        std::vector<double> joints;
        double joint;
        while(ss >> joint) joints.push_back(joint);

        std::transform(transform_vec_.begin(), transform_vec_.end(),
                        joints.begin(), joints.begin(),
                        std::multiplies<double>() );

        return joints;
    }

    bool Device::moveJointPosition(const int profile_no, const std::vector<double>& joints)
    {
        std::vector<double> joints_transformed(joints.size());
        std::transform(joints.begin(), joints.end(),
                        transform_vec_.begin(), joints_transformed.begin(),
                        std::divides<double>() );

        std::stringstream ss;
        ss.precision(3);
        ss << "movej " << profile_no;
        for(size_t i = 0; i < joints.size(); ++i)
        {
            ss << " " << std::fixed << joints_transformed[i];
        }

        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    bool Device::queueJointPosition(const int profile_no, const std::vector<double>& joints)
    {
        std::vector<double> joints_transformed(joints.size());
        std::transform(joints.begin(), joints.end(),
                        transform_vec_.begin(), joints_transformed.begin(),
                        std::divides<double>() );

        std::stringstream ss;
        ss.precision(3);
        ss << "movej " << profile_no;
        for(size_t i = 0; i < joints_transformed.size(); ++i)
            ss << " " << std::fixed << joints_transformed[i];

        command_queue_.push(ss.str());
        return true;
    }

    //TODO: in what coordiation system are the cartesian positions? sync with urdf!
    geometry_msgs::Pose Device::getCartesianPosition()
    {
        std::stringstream ss;
        ss << "wherec";
        Response res = connection_->send(ss.str());
        ss.clear();
        ss.str(res.message);
        geometry_msgs::Pose pose;
        ss >> pose.position.x >> pose.position.y >> pose.position.z;
        double roll, pitch, yaw;
        ss >> roll >> pitch >> yaw;

        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf::quaternionTFToMsg(q, pose.orientation);

        return pose;
    }

    //TODO: in what coordiation system are the cartesian positions? sync with urdf!
    bool Device::moveCartesianPosition(const int profile_no, const geometry_msgs::Pose& pose)
    {
        std::stringstream ss;
        ss.precision(3);
        ss << std::fixed;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        ss << "movec " << profile_no << " "
                        << pose.position.x << " "
                        << pose.position.y << " "
                        << pose.position.z << " "
                        << roll << " "
                        << pitch << " "
                        << yaw;

        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    //TODO: be aware, blocking operations are not allowed in the ros_control loop
    bool Device::waitForEom()
    {
        std::stringstream ss;
        ss << "waitForEom";
        Response res = connection_->send(ss.str());
        //TODO for some reason waitForEom always returns error code -1501. Code is undocumented
        return (res.error == -1501)||(res.error == 0);
    }

    bool Device::graspPlate(const int width, const int speed, const double force)
    {
        std::stringstream ss;
        ss << "graspPlate " << width << " " << speed << " "<< force;
        Response res = connection_->send(ss.str());
        bool success = std::stoi(res.message) == -1;
        return (res.error == 0 && success);
    }

    bool Device::releasePlate(const int width, const int speed)
    {
        std::stringstream ss;
        ss << "releasePlate " << width << " " << speed;
        Response res = connection_->send(ss.str());
        bool success;
        if(res.error == -1501 || res.error == 0)
            success = true;
        else
            success = false;
        return success;
    }

    std::string Device::command(const std::string& cmd)
    {
        Response res = connection_->send(cmd);
        if(res.error == 0)
            return res.message;
        else
        {
            std::stringstream ss;
            ss << "Error: " << res.error;
            return ss.str();
        }
    }

    void Device::startCommandThread()
    {
        command_thread_ = std::thread{&Device::updateCommand, this};
    }

    void Device::clearCommandQueue()
    {
        command_queue_.clear();
    }

    void Device::updateCommand()
    {
        while(ros::ok())
        {
            std::string cmd = command_queue_.pop();
            connection_->send(cmd);
        }
    }
}
