
#pragma once
#include "common.h"

struct CState
{
    int location;
    double timestep;
    int orientation;
    int velocity;

    CState wait() const {return CState(location, timestep + 1, orientation, velocity); }

    struct Hasher
    {
        std::size_t operator()(const CState& n) const
        {
            size_t loc_hash = std::hash<int>()(n.location);
            size_t time_hash = std::hash<double>()(n.timestep);
            size_t ori_hash = std::hash<int>()(n.orientation);
            size_t vel_hash = std::hash<int>()(n.velocity);
            return (time_hash ^ (loc_hash << 1) ^ (ori_hash << 2) ^ (vel_hash << 3));
        }
    };

    void operator = (const CState& other)
    {
        timestep = other.timestep;
        location = other.location;
        orientation = other.orientation;
        velocity = other.velocity;
    }

    bool operator == (const CState& other) const
    {
        return timestep == other.timestep && 
            location == other.location && 
            orientation == other.orientation &&
            velocity == other.velocity;
    }

    bool operator != (const CState& other) const
    {
        return timestep != other.timestep || 
            location != other.location || 
            orientation != other.orientation ||
            velocity != other.velocity;
    }

    CState(): location(-1), timestep(-1), orientation(-1), velocity(0) {}
    CState(int location, double timestep = -1, int orientation = -1, int velocity = 0):
            location(location), timestep(timestep), orientation(orientation), velocity(velocity) {}
    CState(const CState& other) {
        location = other.location; 
        timestep = other.timestep; 
        orientation = other.orientation; 
        velocity = other.velocity;
    }
};

std::ostream & operator << (std::ostream &out, const CState &s);
typedef std::vector<CState> CPath;
std::ostream & operator << (std::ostream &out, const CPath &path);