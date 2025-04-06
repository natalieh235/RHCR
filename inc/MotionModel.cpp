#include "MotionModel.h"

// class MotionModel {
// public:
//     MotionModel(double max_velocity, double max_acceleration, double max_deceleration, double robot_length)
//         : max_velocity(max_velocity), max_acceleration(max_acceleration), max_deceleration(max_deceleration), robot_length(robot_length) {}

//     // Default constructor
//     MotionModel() : max_velocity(0), max_acceleration(0), max_deceleration(0), robot_length(0) {}

//     // Compute the touch and departure times for a path from u to u' considering trapezoidal trajectory
//     void computeTimesForPath(double start_velocity, const std::vector<std::pair<double, double>>& path) {
//         double current_velocity = start_velocity;
//         double current_time = 0;
//         double current_position = path[0].first;  // start at the first vertex in the path
//         double total_distance = 0.0;

//         for (size_t i = 1; i < path.size(); ++i) {
//             double next_position = path[i].first;
//             double distance = next_position - current_position;
//             total_distance += distance;

//             // Compute the times based on trapezoidal trajectory
//             double touch_time = computeTouchTime(current_velocity, distance);
//             double departure_time = computeDepartureTime(touch_time, distance);
            
//             // Update current state for the next segment
//             current_velocity = computeVelocityAtNextStep(current_velocity, distance);
//             current_time = departure_time;
//             current_position = next_position;

//             // Store or output the times for each vertex
//             std::cout << "From " << current_position << " to " << next_position << " -> "
//                       << "Touch Time: " << touch_time << ", Departure Time: " << departure_time << std::endl;
//         }

//         // Compute final arrival time at the last vertex (u')
//         double arrival_time = computeArrivalTime(current_time, total_distance);
//         std::cout << "Final Arrival Time at u': " << arrival_time << std::endl;
//     }

// private:
//     // Helper function to compute touch time (when the front end touches a vertex)
//     double computeTouchTime(double start_velocity, double distance) {
//         double acceleration_time = (max_velocity - start_velocity) / max_acceleration;
//         double acceleration_distance = (max_velocity * max_velocity - start_velocity * start_velocity) / (2 * max_acceleration);

//         if (distance < 2 * acceleration_distance) {
//             // If the distance is too small for full acceleration and deceleration
//             return std::sqrt(2 * distance / max_acceleration);
//         }

//         double constant_velocity_distance = distance - 2 * acceleration_distance;
//         return acceleration_time + constant_velocity_distance / max_velocity;
//     }

//     // Helper function to compute departure time (when the back end leaves a vertex)
//     double computeDepartureTime(double touch_time, double distance) {
//         return touch_time + distance / max_velocity;
//     }

//     // Compute final arrival time considering deceleration at the final vertex
//     double computeArrivalTime(double departure_time, double total_distance) {
//         double deceleration_distance = (max_velocity * max_velocity) / (2 * max_deceleration);

//         if (total_distance <= 2 * deceleration_distance) {
//             // The robot has to decelerate immediately after the acceleration phase
//             double deceleration_time = std::sqrt(2 * total_distance / max_deceleration);
//             return departure_time + deceleration_time;
//         }

//         double constant_velocity_distance = total_distance - 2 * deceleration_distance;
//         return departure_time + constant_velocity_distance / max_velocity + (max_velocity / max_deceleration);
//     }

//     // Helper function to compute the velocity at the next step (after the current segment)
//     double computeVelocityAtNextStep(double current_velocity, double distance) {
//         // If the robot has accelerated, and there is enough distance, it will reach max_velocity
//         double time_to_accelerate = (max_velocity - current_velocity) / max_acceleration;
//         double distance_to_accelerate = (max_velocity * max_velocity - current_velocity * current_velocity) / (2 * max_acceleration);

//         if (distance >= distance_to_accelerate) {
//             return max_velocity;  // Reached max velocity
//         } else {
//             return current_velocity + max_acceleration * time_to_accelerate;
//         }
//     }

//     double max_velocity;       // Max velocity of the agent
//     double max_acceleration;    // Max acceleration of the agent
//     double max_deceleration;    // Max deceleration of the agent
//     double robot_length;        // Length of the robot
// };


class CachedMotionModel {
public:
    struct Profile {
        int start_time;
        int arrival_time;
        std::vector<std::pair<double, double>> touch_departure_times; // per cell: <touch, departure>
    };

    CachedMotionModel(double max_vel, double accel)
        : max_velocity(max_vel), acceleration(accel) {}

    const Profile& getProfile(int length) {
        // auto key = std::make_tuple(start_cell, direction, length);
        if (cache.find(length) != cache.end())
            return cache[key];

        Profile prof;
        prof.start_time = 0;

        double v = 0;
        double x = 0;
        double t = 0;
        std::vector<std::pair<int, int>> tp;

        // Compute trapezoidal timing
        int segment_count = length;
        for (int i = 0; i < segment_count; ++i) {
            double t_entry = t;
            v = std::min(max_velocity, v + acceleration);
            t += 1.0 / v;
            tp.push_back({t_entry, t});
        }

        prof.touch_departure_times = tp;
        prof.arrival_time = t;
        cache[key] = prof;
        return cache[key];
    }

private:
    unordered_map<int, Profile> cache;
    double max_velocity;
    double acceleration;
};;
    
