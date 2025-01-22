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
    for (auto step : path)
    {
        State s = step.state;
        if(s.location < 0)
            continue;
        
        if (step.primitive != "default") {
            out << step.primitive << "->";
        }

        out << "(" << s.location << ","
            << s.orientation << "," 
            << s.timestep << ","
            << s.velocity << ")->";
    }
    out << std::endl;
    return out;
}