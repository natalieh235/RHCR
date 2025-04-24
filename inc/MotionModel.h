#include "common.h"
#include <iostream>

struct Profile
{
    int length;
    double start_time;
    double arrival_time;
    std::vector<std::pair<double, double>> entries; // per cell: <touch, departure>
};

std::ostream &operator<<(std::ostream &os, const Profile &profile);

class MotionModel {
    public:
        MotionModel(double max_vel, double accel, double robot_len)
            : max_velocity(max_vel), acceleration(accel), robot_length(robot_len) {}

        MotionModel() 
            : max_velocity(0), acceleration(0), robot_length(0) {}

        const Profile& getTrapezoidalProfile(int length);
    
    private:
        unordered_map<int, Profile> cache;
        double max_velocity;
        double acceleration;
        double robot_length;
};