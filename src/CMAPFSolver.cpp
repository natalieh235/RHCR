#include "CMAPFSolver.h"
#include <ctime>
#include <iostream>
#include "PathTable.h"


// TODO: implement validate_solution function
bool CMAPFSolver::validate_solution()
{
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            /* find_conflicts(conflict, a1, a2);
             if (!conflict.empty())
             {
                 int a1, a2, loc1, loc2, t;
                 std::tie(a1, a2, loc1, loc2, t) = conflict.front();
                 if (loc2 < 0)
                     std::cout << "Agents "  << a1 << " and " << a2 << " collides at " << loc1 <<
                     " at timestep " << t << std::endl;
                 else
                     std::cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
                               loc1 << "-->" << loc2 << ") at timestep " << t << std::endl;
                 return false;
             }*/
        }
    }
    return true;
}

void CMAPFSolver::print_solution() const
{
    for (int i = 0; i < num_of_agents; i++)
    {
        cout << "Agent " << i << ":\t";
        // for (const auto & loc : solution[i])
        // {
        //     // cout << loc.state.location << ",";
        //     cout << loc << ",";
        // }
        cout << solution[i] << endl;
        cout << endl;
    }
}