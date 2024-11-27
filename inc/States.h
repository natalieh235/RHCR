#pragma once
#include "common.h"

struct State
{
    int location;
    int timestep;
    int orientation;
    int velocity;

    State wait() const {return State(location, timestep + 1, orientation, velocity); }

    struct Hasher
    {
        std::size_t operator()(const State& n) const
        {
            size_t loc_hash = std::hash<int>()(n.location);
            size_t time_hash = std::hash<int>()(n.timestep);
            size_t ori_hash = std::hash<int>()(n.orientation);
            size_t vel_hash = std::hash<int>()(n.velocity);
            return (time_hash ^ (loc_hash << 1) ^ (ori_hash << 2) ^ (vel_hash << 3));
        }
    };

    void operator = (const State& other)
    {
        timestep = other.timestep;
        location = other.location;
        orientation = other.orientation;
        velocity = other.velocity;
    }

    bool operator == (const State& other) const
    {
        return timestep == other.timestep && 
            location == other.location && 
            orientation == other.orientation &&
            velocity == other.velocity;
    }

    bool operator != (const State& other) const
    {
        return timestep != other.timestep || 
            location != other.location || 
            orientation != other.orientation ||
            velocity != other.velocity;
    }

    State(): location(-1), timestep(-1), orientation(-1), velocity(0) {}
    State(int location, int timestep = -1, int orientation = -1, int velocity = 0):
            location(location), timestep(timestep), orientation(orientation), velocity(velocity) {}
    State(const State& other) {
        location = other.location; 
        timestep = other.timestep; 
        orientation = other.orientation; 
        velocity = other.velocity;
    }
};

std::ostream & operator << (std::ostream &out, const State &s);


typedef std::vector<State> Path;

std::ostream & operator << (std::ostream &out, const Path &path);