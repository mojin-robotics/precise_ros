#include <precise_driver/pflex_device.h>
#include <sstream>
#include <string>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

//TODO: decide on methods return values. Either return parsed response or pass result variables as reverences
//and give the return values a success state

namespace precise_driver {
    PFlexDevice::PFlexDevice(TCPConnection &connection)
    {
        this->connection_ = &connection;
    }

    PFlexDevice::~PFlexDevice(){

    }

    bool PFlexDevice::init(int profile_no, Profile profile)
    {
        //TODO: check for correct init routine

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

        //TODO: check right conditions for return value
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

        //TODO: check right conditions for return value
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

        //TODO: find correct pflex internal base position that fits to the urdf description
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

        //TODO: the ros driver should allways use the same profile, so fjt interpolation can be correctly calculated
        //what means 100% speed and 100% accel/decel? what meens 100% accel_ramp/decel_ramp?
        //This needs to fit to the ros controller configuration. Same accel, decel, speed values!
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

        //TODO: handle no result (timeout?)
        return (bool)std::stoi(res.message);
    }

    //TODO: is this important to us? do we need to know the weight of a plate?
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

    //TODO: be aware, blocking operations are not allowed in the ros_control loop
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

        //TODO: are the results in rads or degs? Is there a convertion factor? Compare to urdf!
        return joints;
    }

    //TODO: in what coordiation system are the cartesian positions? sync with urdf!
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

    //TODO: in what coordiation system are the cartesian positions? sync with urdf!
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

    //TODO: are joint states in deg or rad? is there a conversion needed?
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

    //TODO: does freemode unlocks all breaks?
    //There is also a "setTorque" command that can set the desired torque per joint. Need to implement?
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