#include "CStates.h"


std::ostream & operator << (std::ostream &out, const CState &s)
{
    out << "Loc:" << s.location << "," 
        << "Orient:" << s.orientation << "," 
        << "Timestep:" << s.timestep << "," 
        << "Vel:" << s.velocity;
    return out;
}

std::ostream & operator << (std::ostream &out, const CPath &path)
{
    for (auto s : path)
    {
        // CState s = step.state;
        if(s.location < 0)
            continue;

        out << "(" << s.location << ","
            << s.orientation << "," 
            << s.timestep << ","
            << s.velocity << ")->";
    }
    out << std::endl;
    return out;
}