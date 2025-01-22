#include "PathTable.h"


PathTable::PathTable(const vector<Path*>& paths, int window, int k_robust):
    window(window), k_robust(k_robust)
{
    num_of_agents = (int)paths.size();
    for (int i = 0; i < num_of_agents; i++)
    {
        for (auto step : (*paths[i]))
        {
            if (step.state.timestep > window)
                break;
            PT[step.state.location].emplace_back(step.state.timestep, i);
        }
    }
}


void PathTable::remove(const Path* old_path, int agent)
{
    if (old_path == nullptr)
        return;
    for (auto step : (*old_path))
    {
        if (step.state.timestep > window)
            break;
        for (auto it = PT[step.state.location].begin(); it != PT[step.state.location].end(); ++it)
        {
            if (it->first == step.state.timestep && it->second == agent)
            {
                PT[step.state.location].erase(it);
                break;
            }
        }
    }
}


list<std::shared_ptr<Conflict> > PathTable::add(const Path* new_path, int agent)
{
    list<std::shared_ptr<Conflict> > conflicts;
    vector<bool> conflicting_agents(num_of_agents, false);
    for (auto step : (*new_path))
    {
        if (step.state.timestep > window)
            break;
        for (auto it = PT[step.state.location].begin(); it != PT[step.state.location].end(); ++it)
        {
            if (conflicting_agents[it->second])
                continue;
            else if (abs(it->first - step.state.timestep) <= k_robust)
            {
                conflicts.push_back(std::shared_ptr<Conflict>(
                        new Conflict(agent, it->second, step.state.location, -1, min(it->first, step.state.timestep))));
                conflicting_agents[it->second] = true;
            }
        }
    }
    for (auto step : (*new_path))
    {
        if (step.state.timestep > window)
            break;
        PT[step.state.location].emplace_back(step.state.timestep, agent);
    }
    return conflicts;
}