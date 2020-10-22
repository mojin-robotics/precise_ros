#include <precise_driver/pflex_device.h>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

//TODO: decide on methods return values. Either return parsed response or pass result variables as reverences
//and give the return values a success state

namespace precise_driver
{
    PFlexDevice::PFlexDevice(std::shared_ptr<PreciseTCPInterface> connection)
    {
        connection_ = connection;
    }

    PFlexDevice::~PFlexDevice()
    {
        std::cout<<"exiting"<<std::endl;
        setHp(false);
        exit();
    }

    bool PFlexDevice::init(const int& profile_no, const Profile& profile)
    {
        //TODO: check for correct init routine
        ROS_INFO("initializing...");

        //connect to robot
        try{
            connection_->connect();
        }
        catch(boost::system::system_error &e)
        {
            ROS_ERROR_STREAM(e.what());
            return false;
        }

        int sysState = getSysState(true);
        ROS_INFO_STREAM("SysState is: "<<sysState);

        ROS_DEBUG("setting Mode to 0");
        setMode(0);

        //test connection
        ROS_DEBUG("testing connection...");
        if(!nop())
        {
            ROS_ERROR("connection test failed");
            return false;
        }
        ROS_DEBUG("success");

        sysState = getSysState(true);
        ROS_DEBUG_STREAM("SysState is: "<<sysState);

        ROS_DEBUG("detach");
        is_attached_ = attach(false);

        ROS_DEBUG("set High Power");
        setHp(true, 5);

        sysState = getSysState(true);
        ROS_DEBUG_STREAM("SysState is: "<<sysState);

        setProfile(profile_no, profile);
        is_attached_ = attach(true);

        ROS_INFO("initialized");
        return is_attached_;
    }

    bool PFlexDevice::operable()
    {
        bool ret;
        {
            std::lock_guard<std::mutex> guard(mutex_state_data_);
            ret = is_attached_ && is_homed_ && is_hp_ && !is_teachmode_;
        }
        return ret;
    }

    bool PFlexDevice::exit()
    {
        std::stringstream ss;
        ss << "exit";
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    bool PFlexDevice::halt()
    {
        std::stringstream ss;
        ss << "halt";
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    bool PFlexDevice::home()
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

    bool PFlexDevice::attach(const bool& flag)
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

    bool PFlexDevice::selectRobot(const int& robot)
    {
        std::stringstream ss;
        if(robot!=-1)
            ss << "selectRobot "<< robot;
        else
            ss << "selectRobot";
        Response res = connection_->send(ss.str());

        return (res.error == 0);
    }

    bool PFlexDevice::setBase(const geometry_msgs::Pose& pose)
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

    geometry_msgs::Pose PFlexDevice::getBase()
    {
        std::stringstream ss;
        ss << "setBase";
        //Response res = connection_->send(ss.str());
        ss.clear();
        //ss.str(res.message);
        geometry_msgs::Pose pose;
        ss >> pose.position.x >> pose.position.y >> pose.position.z;
        double yaw;
        ss >> yaw;

        tf::Quaternion q;
        q.setRPY(0,0,yaw);
        tf::quaternionTFToMsg(q, pose.orientation);

        return pose;
    }

    Profile PFlexDevice::getProfile(const int& profile_no)
    {
        std::stringstream ss;
        ss << "profile " << profile_no;
        Response res = connection_->send(ss.str());
        ss.clear();
        ss.str(res.message);
        Profile profile;
        ss >> profile.speed >> profile.speed2 >> profile.accel >> profile.decel >>
            profile.accel_ramp >> profile.decel_ramp >> profile.in_range >> profile.straight;

        //TODO: the ros driver should allways use the same profile, so fjt interpolation can be correctly calculated
        //what means 100% speed and 100% accel/decel? what meens 100% accel_ramp/decel_ramp?
        //This needs to fit to the ros controller configuration. Same accel, decel, speed values!
        return profile;
    }

    bool PFlexDevice::setProfile(const int& profile_no, const Profile& profile)
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

    bool PFlexDevice::nop()
    {
        std::stringstream ss;
        ss << "nop";
        Response res = connection_->send(ss.str());


        return (res.error == 0);
    }

    //TODO: is this important to us? do we need to know the weight of a plate?
    bool PFlexDevice::setPayload(const int& payload)
    {
        std::stringstream ss;
        ss << "payload " << payload;
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    int PFlexDevice::getPayload()
    {
        std::stringstream ss;
        ss << "payload";
        Response res = connection_->send(ss.str());
        return std::stoi(res.message);
    }

    bool PFlexDevice::setSpeed(const int& profile_no, const int& speed)
    {
        std::stringstream ss;
        ss << "speed " << speed;
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    int PFlexDevice::getSpeed(const int& profile_no)
    {
        std::stringstream ss;
        ss << "speed";
        Response res = connection_->send(ss.str());
        return std::stoi(res.message);
    }

    bool PFlexDevice::setHp(const bool& enabled, const int& timeout)
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

    bool PFlexDevice::getHp()
    {
        std::stringstream ss;
        ss << "hp";
        Response res = connection_->send(ss.str());
        return (bool)std::stoi(res.message);
    }

    //TODO: be aware, blocking operations are not allowed in the ros_control loop
    bool PFlexDevice::waitForEom(const double& timeout)
    {
        std::stringstream ss;
        ss << "waitForEom";
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    std::vector<double> PFlexDevice::getJointPositions()
    {
        std::stringstream ss;
        ss << "wherej";
        Response res = connection_->send(ss.str());
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

    //TODO: in what coordiation system are the cartesian positions? sync with urdf!
    geometry_msgs::Pose PFlexDevice::getCartesianPosition()
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
    bool PFlexDevice::moveCartesian(const int& profile_no,  const geometry_msgs::Pose& pose)
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

    //TODO: are joint states in deg or rad? is there a conversion needed?
    bool PFlexDevice::moveJointSpace(const int& profile_no, const std::vector<double>& joints)
    {
        std::transform(joints.begin(), joints.end(),
                        transform_vec_.begin(), joints.begin(),
                        std::divides<double>() );

        std::stringstream ss;
        ss.precision(3);
        ss << "movej " << profile_no;
        for(size_t i = 0; i < joints.size(); ++i)
        {
            ss << " " << std::fixed << joints[i];
        }

        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    //TODO: There is a freeMode command described the TCS Documentation
    bool PFlexDevice::freeMode(const bool& enabled)
    {
        std::stringstream ss;
        if(enabled)
            ss << "zeroTorque " << static_cast<int>(enabled)<<" 31"; //bitmask 1=axis1, 2=axis2, 4=axis3 ...
        else
            ss << "zeroTorque " << static_cast<int>(enabled);

        Response res = connection_->send(ss.str());

        {
            std::lock_guard<std::mutex> guard(mutex_state_data_);
            is_teachmode_ = res.success && enabled;
        }

        return (res.error == 0);
    }

    int PFlexDevice::getSysState(const bool& mute)
    {
        std::stringstream ss;
        ss << "sysState " << static_cast<int>(mute);
        Response res = connection_->send(ss.str());
        if(res.message!="")
            return std::stoi(res.message);
        else
            return res.error;
    }

    int PFlexDevice::getMode()
    {
        std::stringstream ss;
        ss << "mode";
        Response res = connection_->send(ss.str());
        return std::stoi(res.message);
    }

    bool PFlexDevice::setMode(const int& mode)
    {
        std::stringstream ss;
        ss << "mode " << mode;
        Response res = connection_->send(ss.str());
        return (res.error == 0);
    }

    std::string PFlexDevice::command(const std::string& cmd)
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

}