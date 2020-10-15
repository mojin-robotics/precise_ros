#include <precise_driver/pflex_device.h>
#include <sstream>
#include <string>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

namespace precise_driver {
    PFlexDevice::PFlexDevice(TCPConnection &connection)
    {
        this->connection_ = &connection;
    }

    PFlexDevice::~PFlexDevice(){

    }

    bool PFlexDevice::init(int profile_no, Profile profile)
    {
        //connect to robot
        this->connection_->connect();

        //test connection
        if(!this->nop())
            return false;

        this->is_attached_ = this->attach(false);
        this->setHp(true);

        this->setProfile(profile_no, profile);
        this->is_attached_ = this->attach(true);

    }

    bool PFlexDevice::halt()
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "halt";
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::home()
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "home";
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::attach(bool flag)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "attach "<<static_cast<int>(flag);
        Response res = connection_->sendCommand(ss.str());
        return (flag) ? (bool) std::stoi(res.message) : !(bool)std::stoi(res.message);
    }

    bool PFlexDevice::selectRobot(int robot)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        if(robot!=-1)
            ss << "selectRobot "<< robot;
        else
            ss << "selectRobot";
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::setBase(geometry_msgs::Pose pose)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);


        ss << "setBase " << pose.position.x << " "
                        << pose.position.y << " "
                        << pose.position.z << " "
                        << yaw;

        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    geometry_msgs::Pose PFlexDevice::getBase()
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "setBase";
        //Response res = connection_->sendCommand(ss.str());
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

    Profile PFlexDevice::getProfile(int profile_no)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "profile " << profile_no;
        Response res = connection_->sendCommand(ss.str());
        ss.clear();
        ss.str(res.message);
        Profile profile;
        ss >> profile.speed >> profile.speed2 >> profile.accel >> profile.decel >>
            profile.accel_ramp >> profile.decel_ramp >> profile.in_range >> profile.straight;
        return profile;
    }

    bool PFlexDevice::setProfile(int profile_no, Profile profile)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

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

        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::nop()
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "nop";
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::setPayload(int payload)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "payload " << payload;
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    int PFlexDevice::getPayload()
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "payload";
        Response res = connection_->sendCommand(ss.str());
        return std::stoi(res.message);
    }

    bool PFlexDevice::setSpeed(int speed)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "speed " << speed;
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    int PFlexDevice::getSpeed()
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "speed";
        Response res = connection_->sendCommand(ss.str());
        return std::stoi(res.message);
    }

    bool PFlexDevice::setHp(bool enabled)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "hp " << static_cast<int>(enabled);
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::getHp()
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "hp";
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::waitForEom(double timeout)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "waitForEom";
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    std::vector<double> PFlexDevice::getJointPositions()
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "wherej";
        Response res = connection_->sendCommand(ss.str());
        ss.clear();
        ss.str(res.message);
        std::vector<double> joints;
        double joint;
        while(ss >> joint) joints.push_back(joint);
        
    }

    geometry_msgs::Pose PFlexDevice::getCartesianPosition()
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "wherec";
        Response res = connection_->sendCommand(ss.str());
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

    bool PFlexDevice::moveCartesian(geometry_msgs::Pose pose)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);


        ss << "movec " << pose.position.x << " "
                        << pose.position.y << " "
                        << pose.position.z << " "
                        << roll << " "
                        << pitch << " "
                        << yaw;

        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::moveJointSpace(std::vector<double> joints)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "movej ";
        for(size_t i = 0; i < joints.size(); ++i)
        {
            ss << joints[i];
        }
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::freeMode(bool enabled)
    {
        //TODO: move lock_guard to sendCommand
        std::lock_guard<std::mutex> guard(this->comm_mutex);

        std::stringstream ss;
        ss << "freemode " << static_cast<int>(enabled);
        Response res = connection_->sendCommand(ss.str());
        return (bool)std::stoi(res.message);
    }
}