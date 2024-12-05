#include "States.h"


std::ostream & operator << (std::ostream &out, const State &s)
{
    out << "Loc:" << s.location << "," 
        << "Orient:" << s.orientation << "," 
        << "Timestep:" << s.timestep << "," 
        << "Vel:" << s.velocity;
    return out;
}

std::ostream & operator << (std::ostream &out, const Path &path)
{
    for (auto state : path)
    {
        if(state.location < 0)
            continue;
        out << "(" << state.location << ","
            << state.orientation << "," 
            << state.timestep << ","
            << state.velocity << ")->";
    }
    out << std::endl;
    return out;
}