#include "MotionModel.h"

std::ostream &operator<<(std::ostream &os, const Profile &profile)
{
    os << "Motion Profile:\n";
    int idx = 0;
    for (const auto &entry : profile.entries)
    {
        os << "  Cell " << std::setw(2) << idx++
           << " | Touch Time: " << std::fixed << std::setprecision(3) << std::get<0>(entry)
           << " | Sweeping Time: " << std::fixed << std::setprecision(3) << std::get<1>(entry) << "\n";
    }
    os << "Total Arrival Time: " << std::fixed << std::setprecision(3) << profile.arrival_time << "\n";
    return os;
}

const Profile &MotionModel::getTrapezoidalProfile(int length)
{
    if (cache.find(length) != cache.end())
        return cache[length];

    int cell_length = 1;
    double d = length * cell_length;
    double t_accel = max_velocity / acceleration;
    double d_accel = 0.5 * acceleration * t_accel * t_accel;

    Profile profile;
    double t_total = 0;

    // Helper: time to reach distance x
    auto timeToReach = [&](double x) -> double {
        if (2 * d_accel <= d) {  // Trapezoidal
            double d_cruise = d - 2 * d_accel;
            double t_cruise = d_cruise / max_velocity;

            if (x < d_accel) {
                return std::sqrt(2 * x / acceleration);
            } else if (x < d_accel + d_cruise) {
                return t_accel + (x - d_accel) / max_velocity;
            } else {
                double x_decel = x - d_accel - d_cruise;
                return t_accel + t_cruise +
                       (max_velocity - std::sqrt(max_velocity * max_velocity - 2 * acceleration * x_decel)) / acceleration;
            }
        } else {  // Triangular
            double t_peak = std::sqrt(d / acceleration);
            double v_peak = acceleration * t_peak;

            if (x < 0.5 * d) {
                return std::sqrt(2 * x / acceleration);
            } else {
                double x_decel = d - x;
                return 2 * t_peak - std::sqrt(2 * x_decel / acceleration);
            }
        }
    };
 

    // Compute profile entries
    for (int i = 0; i <= length; i++) {
        // time the front end touches the cell is the distance from the start of the robot to the start of the cell
        // if robot is at x, front end is at x + (robot_length/2). the cell start is at (i - 0.5) * cell_length
        // double front_touch = (i + 0.5) * cell_length;
        double front_touch_distance = std::max((i-0.5)*cell_length - robot_length/2, 0.0);

        double exit_distance = (i+0.5)*cell_length - robot_length/2;

        if (i == length) {
            exit_distance = d;
        }

        double t_touch = timeToReach(front_touch_distance);
        double t_depart = timeToReach(exit_distance);

        // std::cout << "i: " << i << " | front_touch: " << front_touch_distance
        //           << " | front_exit: " << exit_distance << " | t_touch: " << t_touch
        //           << " | t_depart: " << t_depart << std::endl;

        profile.entries.emplace_back(t_touch, t_depart - t_touch);
    }

    // Store profile
    profile.length = length;
    profile.arrival_time = timeToReach(d + robot_length); // arrival = front reaches end + robot
    profile.start_time = 0;
    cache[length] = profile;
    return cache[length];
}
