#pragma once

#include <precise_driver/precise_tcp_interface.h>
#include <precise_driver/queue.h>

#include <iostream>
#include <vector>
#include <mutex>
#include <thread>

#include <geometry_msgs/Pose.h>

namespace precise_driver
{
    struct Profile{
        int speed;
        int speed2;
        int accel;
        int decel;
        double accel_ramp;
        double decel_ramp;
        int in_range;
        int straight;
    };

    class PFlexDevice
    {
    public:
        explicit PFlexDevice(std::shared_ptr<PreciseTCPInterface> connection,
                    std::shared_ptr<PreciseTCPInterface> status_connection);
        ~PFlexDevice();

        void startMoveJThread();

        bool init(const int& profile_no, const Profile& profile);
        bool exit();
        bool halt();
        bool home();
        bool attach(const bool& flag);
        bool selectRobot(const int& robot=-1);
        bool setBase(const geometry_msgs::Pose& pose);
        geometry_msgs::Pose getBase();
        Profile getProfile(const int& profile_no);
        bool setProfile(const int& profile_no, const Profile& profile);
        bool nop();
        bool setPayload(const int& payload);
        int getPayload();
        bool setSpeed(const int& profile_no, const int& speed);
        int getSpeed(const int& profile_no);
        bool setHp(const bool& enabled, const int& timeout=0);
        bool getHp();
        bool waitForEom(const double& timeout);
        std::vector<double> getJointPositions();
        geometry_msgs::Pose getCartesianPosition();
        bool moveCartesian(const int& profile_no, const geometry_msgs::Pose& pose);
        bool moveJointSpace(const int& profile_no, const std::vector<double>& joints);
        bool queueJointSpace(const int& profile_no, const std::vector<double>& joints);
        bool freeMode(const bool& enabled);
        int getSysState(const bool& mute);
        int getMode();
        bool setMode(const int& mode);
        std::string command(const std::string& cmd);
        bool operable();
        void update_movej();

    private:
        std::shared_ptr<PreciseTCPInterface> connection_;
        std::shared_ptr<PreciseTCPInterface> status_connection_;

        bool is_attached_;
        bool is_hp_;
        bool is_homed_;
        bool is_teachmode_;

        std::mutex mutex_state_data_;

        std::vector<double> transform_vec_ = {0.001, M_PI / 180, M_PI / 180, M_PI / 180, 0.0005};

        Queue<std::string> movej_queue_;
        std::thread movej_thread_;
    };
}