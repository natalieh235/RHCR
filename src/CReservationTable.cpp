#include "CReservationTable.h"

void ContinuousReservationTable::addReservation(int loc, double start, double end) {
    if (start >= end) return;
    table[loc].insert({start, end});
    std::cout << "add reservation: loc " << loc << ", start " << start << ", end " << end << std::endl;

}

CInterval ContinuousReservationTable::getFirstSafeInterval(int loc, double t_start) {
    auto& reserved = table[loc];
    double last_end = 0.0;

    for (auto& interval : reserved) {
        double r_start = std::get<0>(interval);
        double r_end = std::get<1>(interval);

        if (t_start < r_start && t_start >= last_end) {
            return {t_start, r_start, 0};
        }
        last_end = max(last_end, r_end);
    }

    // If we didn't return early, there's space after the last reservation
    return {max(t_start, last_end), INTERVAL_MAX, 0};
}

vector<CInterval> ContinuousReservationTable::getSafeIntervals(int loc, double t_start, double t_end) {
    vector<CInterval> safe;
    auto& reserved = table[loc];
    double last_end = 0.0;

    for (auto& interval : reserved) {
        double r_start = std::get<0>(interval);
        double r_end = std::get<1>(interval);

        if (r_end <= t_start || r_start > t_end) continue;

        if (r_start > last_end) {
            // bool c = std::
            safe.push_back({last_end, r_start, 0});
        }

        last_end = max(last_end, r_end);
    }

    safe.push_back({last_end, INTERVAL_MAX, 0});
    return safe;
}

list<CInterval> ContinuousReservationTable::getConflictIntervals(int loc, double t_start, double t_end) {
    list<CInterval> conflicts;
    auto& reserved = table[loc];

    // std::cout << "      Reserved for loc " << loc << " has size " << reserved.size() << std::endl;

    for (auto& interval : reserved) {
        double r_start = std::get<0>(interval);
        double r_end = std::get<1>(interval);

        if (r_end <= t_start || r_start > t_end) continue;

        double conflict_start = max(t_start, r_start);
        double conflict_end = min(t_end, r_end);
        conflicts.push_back({conflict_start, conflict_end, 0});
    }

    return conflicts;
}

void ContinuousReservationTable::insertPath(const CPath& path) {
    std::cout << "inserting path of length " << path.size() << std::endl;
    double prev_time = 0.0;
    for (int i = 0; i < path.size() - 1; i++) {
        const CState &s1 = std::get<0>(path[i]);
        const CState &s2 = std::get<0>(path[i + 1]);

        // double start_time = 0.0;

        int path_len = (s2.location - s1.location) / G.move[s2.orientation];
        int turn_time = G.get_rotate_degree(s1.orientation, s2.orientation);

        std::cout << "cur path segment length " << path_len << ", turn time " << turn_time << std::endl;
        Profile profile = motion_model.getTrapezoidalProfile(path_len);

        // double start_time = s1.timestep;
        double start_time = prev_time;
        std::cout << "start time " << start_time << std::endl;
        std::cout << "s1.timestep " << s1.timestep << std::endl;
        std::cout << "profile" << profile << std::endl;
        double wait_time = std::get<1>(path[i]);
        for (int j = 0; j < profile.length; j++) {
            // std::cout << j << std::endl;
            int cur_loc = s1.location + j * G.move[s2.orientation];

            double start_interval;
            double end_interval;
            if (j == 0) {
                start_interval = start_time + std::get<0>(profile.entries[j]);
                end_interval = start_interval + std::get<1>(profile.entries[j]) + turn_time + wait_time;
            } else {
                start_interval = start_time + std::get<0>(profile.entries[j]) + turn_time + wait_time;
                end_interval = start_interval + std::get<1>(profile.entries[j]);
            }

            if (end_interval < start_interval) {
                std::cout << "error: end interval < start interval" << std::endl;
            }

            addReservation(cur_loc, start_interval, end_interval);

            if (j == profile.length-1) {
                prev_time = end_interval;
            }
        }
    }
}