#include <precise_driver/pflex_device.h>
#include <sstream>
#include <string>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

namespace precise_driver {
    PFlexDevice::PFlexDevice(/*TCPConnection &connection*/)
    {
        //this->connection_ = &connection;
    }

    PFlexDevice::~PFlexDevice(){

    }

    bool PFlexDevice::init(Profile profile)
    {
        //connect to robot
        //connection.connect();

        //test connection
        if(!this->nop())
            return false;

        this->is_attached_ = this->attach(false);
        this->setHp(true);

        this->setProfile(profile);
        this->is_attached_ = this->attach(true);

    }

    bool PFlexDevice::halt()
    {
        std::stringstream ss;
        ss << "halt";
        //Response res = connection->sendCommand(ss.str());
        //return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::home()
    {
        std::stringstream ss;
        ss << "home";
        //Response res = connection->sendCommand(ss.str());
        //return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::attach(bool flag)
    {
        std::stringstream ss;
        ss << "attach "<<static_cast<int>(flag);
        //Response res = connection->sendCommand(ss.str());
        //return (flag) ? (bool) std::stoi(res.message) : !(bool)std::stoi(res.message);
    }

    bool PFlexDevice::selectRobot(int robot=-1)
    {
        std::stringstream ss;
        if(robot!=-1)
            ss << "selectRobot "<< robot;
        else
            ss << "selectRobot";
        //Response res = connection->sendCommand(ss.str());
        //return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::setBase(geometry_msgs::Pose pose)
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

        //Response res = connection->sendCommand(ss.str());
        //return (bool)std::stoi(res.message);
    }

    geometry_msgs::Pose PFlexDevice::getBase()
    {
        std::stringstream ss;
        ss << "setBase";
        //Response res = connection->sendCommand(ss.str());
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
        std::stringstream ss;
        ss << "profile " << profile_no;
        //Response res = connection->sendCommand(ss.str());
        ss.clear();
        //ss.str(res.message);
        Profile profile;
        ss >> profile.speed >> profile.speed2 >> profile.accel >> profile.decel >>
            profile.accel_ramp >> profile.decel_ramp >> profile.in_range >> profile.straight;
        return profile;
    }

    bool PFlexDevice::setProfile(int profile_no, Profile profile)
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

        //Response res = connection->sendCommand(ss.str());
        //return (bool)std::stoi(res.message);

    }

    bool PFlexDevice::nop()
    {
        std::stringstream ss;
        ss << "nop";
        //Response res = connection->sendCommand(ss.str());
        //return (bool)std::stoi(res.message);
    }

    bool PFlexDevice::setPayload(int payload)
    {

    }

    int PFlexDevice::getPayload()
    {

    }

    bool PFlexDevice::setSpeed(double speed)
    {

    }

    bool PFlexDevice::getSpeed(double speed)
    {

    }

    bool PFlexDevice::setHp(bool enabled)
    {

    }

    bool PFlexDevice::getHp()
    {

    }

    bool PFlexDevice::waitForEom(double timeout)
    {

    }

    std::vector<double> PFlexDevice::getJointPositions()
    {

    }

    geometry_msgs::Pose PFlexDevice::getCartesianPosition()
    {

    }

    bool PFlexDevice::moveCartesian(geometry_msgs::Pose pose)
    {

    }

    bool PFlexDevice::moveJointSpace(std::vector<double> joints)
    {

    }

    bool PFlexDevice::freeMode(bool enabled)
    {

    }
}