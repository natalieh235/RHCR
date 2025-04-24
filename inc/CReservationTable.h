#pragma once
#include "common.h"
#include "CStates.h"
#include "MotionModel.h"
#include "BasicGraph.h"

class ContinuousReservationTable {
    public:
        // unordered_map<int, set<tuple<double, double>>> rt;
        void addReservation(int loc, double start, double end);

        CInterval getFirstSafeInterval(int loc, double t_start = 0);
        vector<CInterval> getSafeIntervals(int loc, double t_start, double t_end);
        list<CInterval> getConflictIntervals(int loc, double t_start, double t_end);
        void insertPath(const CPath& path);

        ContinuousReservationTable(const BasicGraph& G, const MotionModel& motion_model): 
            G(G), motion_model(motion_model) {}
        MotionModel motion_model;
    
    private:
        unordered_map<int, set<tuple<double, double>>> table; // location -> list of reserved intervals
        const BasicGraph& G;
};
