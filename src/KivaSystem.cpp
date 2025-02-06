#include "KivaSystem.h"
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"
#include "PBS.h"

KivaSystem::KivaSystem(const KivaGrid& G, MAPFSolver& solver): BasicSystem(G, solver), G(G) {}


KivaSystem::~KivaSystem()
{
}

void KivaSystem::initialize()
{
	initialize_solvers();

	// solver.fill_primitives();

	std::cout << " intialized solvers "<< std::endl;

	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);

	std::cout << " resized"<< std::endl;
	consider_rotation = true;
	bool succ = load_records(); // continue simulating from the records
	std::cout << " load records"<< std::endl;
	if (!succ)
	{
		timestep = 0;
		succ = load_locations();
		if (!succ)
		{
			cout << "Randomly generating initial locations" << endl;
			initialize_start_locations();
			initialize_goal_locations();


			// manual testing:
			// starts[0] = State(G.agent_home_locations[0], 0, 1);
			// std::cout << "start for agent " << 0 << " is " << starts[0] << std::endl;
			// paths[0].emplace_back(starts[0]);
			// finished_tasks[0].emplace_back(G.agent_home_locations[0], 0);

			// starts[1] = State(G.agent_home_locations[1], 0, 1);
			// std::cout << "start for agent " << 1 << " is " << starts[1] << std::endl;
			// paths[1].emplace_back(starts[1]);
			// finished_tasks[1].emplace_back(G.agent_home_locations[1], 0);

			// int goal1 = 20;
			// int goal2 = 37;
			// goal_locations[0].emplace_back(goal1, 0);
			// std::cout << "goal for agent " << 0 << " is " << goal1 << std::endl;
			// goal_locations[1].emplace_back(goal2, 0);
			// std::cout << "goal for agent " << 1 << " is " << goal2 << std::endl;
		}
	}
}

void KivaSystem::initialize_start_locations()
{
	// Choose random start locations
	// Any non-obstacle locations can be start locations
	// Start locations should be unique
	// std::cout << "initializing start location" << consider_rotation << std::endl;
	for (int k = 0; k < num_of_drives; k++)
	{
		int orientation = -1;
		if (consider_rotation)
		{
			orientation = rand() % 4;
			while (!G.valid_move(G.agent_home_locations[k], orientation)) {
				orientation = rand() % 4;
			}
		}

		starts[k] = State(G.agent_home_locations[k], 0, orientation);
		std::cout << "start for agent " << k << " is " << starts[k] << std::endl;
		paths[k].emplace_back(starts[k]);
		finished_tasks[k].emplace_back(G.agent_home_locations[k], 0);
	}
}


void KivaSystem::initialize_goal_locations()
{
	if (hold_endpoints || useDummyPaths)
		return;
	// Choose random goal locations
	// Goal locations are not necessarily unique
	for (int k = 0; k < num_of_drives; k++)
	{
		// int goal_idx = rand() % (int)G.endpoints.size();
		// std::cout << "goal idx " << goal_idx << std::endl;
		int goal = G.endpoints[rand() % (int)G.endpoints.size()];
		std::cout << "goal for agent " << k << " is " << goal << std::endl;
		goal_locations[k].emplace_back(goal, 0);
		
	}
}



void KivaSystem::update_goal_locations()
{
	std::cout << "update goal locations, " << hold_endpoints << std::endl;
    if (!LRA_called)
        new_agents.clear();
	if (hold_endpoints)
	{
		unordered_map<int, int> held_locations; // <location, agent id>
		for (int k = 0; k < num_of_drives; k++)
		{
			int curr = paths[k][timestep].state.location; // current location
			if (goal_locations[k].empty())
			{
				int next = G.endpoints[rand() % (int)G.endpoints.size()];
				while (next == curr || held_endpoints.find(next) != held_endpoints.end())
				{
					next = G.endpoints[rand() % (int)G.endpoints.size()];
				}
				goal_locations[k].emplace_back(next, 0);
				held_endpoints.insert(next);
			}
			if (paths[k].back().state.location == goal_locations[k].back().first &&  // agent already has paths to its goal location
				paths[k].back().state.timestep >= goal_locations[k].back().second) // after its release time
			{
				int agent = k;
				int loc = goal_locations[k].back().first;
				auto it = held_locations.find(loc);
				while (it != held_locations.end()) // its start location has been held by another agent
				{
					int removed_agent = it->second;
					if (goal_locations[removed_agent].back().first != loc)
						cout << "BUG" << endl;
					new_agents.remove(removed_agent); // another agent cannot move to its new goal location
					cout << "Agent " << removed_agent << " has to wait for agent " << agent << " because of location " << loc << endl;
					held_locations[loc] = agent; // this agent has to keep holding this location
					agent = removed_agent;
					loc = paths[agent][timestep].state.location; // another agent's start location
					it = held_locations.find(loc);
				}
				held_locations[loc] = agent;
			}
			else // agent does not have paths to its goal location yet
			{
				if (held_locations.find(goal_locations[k].back().first) == held_locations.end()) // if the goal location has not been held by other agents
				{
					held_locations[goal_locations[k].back().first] = k; // hold this goal location
					new_agents.emplace_back(k); // replan paths for this agent later
					continue;
				}
				// the goal location has already been held by other agents 
				// so this agent has to keep holding its start location instead
				int agent = k;
				int loc = curr;
				cout << "Agent " << agent << " has to wait for agent " << held_locations[goal_locations[k].back().first] << " because of location " <<
					goal_locations[k].back().first << endl;
				auto it = held_locations.find(loc);
				while (it != held_locations.end()) // its start location has been held by another agent
				{
					int removed_agent = it->second;
					if (goal_locations[removed_agent].back().first != loc)
						cout << "BUG" << endl;
					new_agents.remove(removed_agent); // another agent cannot move to its new goal location
					cout << "Agent " << removed_agent << " has to wait for agent " << agent << " because of location " << loc << endl;
					held_locations[loc] = agent; // this agent has to keep holding its start location
					agent = removed_agent;
					loc = paths[agent][timestep].state.location; // another agent's start location
					it = held_locations.find(loc);
				}
				held_locations[loc] = agent;// this agent has to keep holding its start location
			}
		}
	}
	else
	{
		for (int k = 0; k < num_of_drives; k++)
		{
			int curr = paths[k][timestep].state.location; // current location
			if (useDummyPaths)
			{
				if (goal_locations[k].empty())
				{
					goal_locations[k].emplace_back(G.agent_home_locations[k], 0);
				}
				if (goal_locations[k].size() == 1)
				{
					int next;
					do {
						next = G.endpoints[rand() % (int)G.endpoints.size()];
					} while (next == curr);
					goal_locations[k].emplace(goal_locations[k].begin(), next, 0);
					new_agents.emplace_back(k);
				}
			}
			else
			{
				pair<int, int> goal; // The last goal location
				if (goal_locations[k].empty())
				{
					goal = make_pair(curr, 0);
					std::cout << "update goal locations: goal locations r empty " << curr << std::endl;
				}
				else
				{
					goal = goal_locations[k].back();
				}
				double min_timesteps = G.get_Manhattan_distance(goal.first, curr); // G.heuristics.at(goal)[curr];
				while (min_timesteps <= simulation_window)
					// The agent might finish its tasks during the next planning horizon
				{
					// assign a new task
					pair<int, int> next;
					if (G.types[goal.first] == "Endpoint")
					{
						do
						{
							next = make_pair(G.endpoints[rand() % (int)G.endpoints.size()], 0);
						} while (next == goal);
					}
					else
					{
						std::cout << "ERROR in update_goal_function()" << std::endl;
						std::cout << "The fiducial type should not be " << G.types[curr] << std::endl;
						exit(-1);
					}
					std::cout << "update goal locations: next location is for " << k << " is " << next.first << std::endl;
					std::cout << "num goal locations: " << goal_locations[k].size() << std::endl;
					goal_locations[k].emplace_back(next);
					min_timesteps += G.get_Manhattan_distance(next.first, goal.first); // G.heuristics.at(next)[goal];
					goal = next;
				}
			}
		}
	}

}


void KivaSystem::simulate(int simulation_time)
{
	std::cout << "*** Simulating " << seed << " ***" << std::endl;
	this->simulation_time = simulation_time;
	initialize();

	std::cout << "Nat :initalized" << std::endl;

	for (; timestep < simulation_time; timestep += simulation_window)
	{
		std::cout << "\n SYSTEM SIMULATION: Timestep " << timestep << "\n" << std::endl;

		update_start_locations();
		update_goal_locations();
		// std::cout << "start size" << starts.size() << " end size" << goal_locations.size() << std::endl;
		// std::cout << "Calling solve..." << std::endl;
		solve();

		// move drives
		auto new_finished_tasks = move();

		// std::cout << "Timestep 2 " << timestep << std::endl;
		std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;

		// update tasks
		for (auto task : new_finished_tasks)
		{
			int id, loc, t;
			std::tie(id, loc, t) = task;
			finished_tasks[id].emplace_back(loc, t);
			num_of_tasks++;
			if (hold_endpoints)
				held_endpoints.erase(loc);
		}

		if (congested())
		{
			cout << "***** Too many traffic jams ***" << endl;
			break;
		}
	}

	update_start_locations();
	std::cout << std::endl << "AM I DONE!" << std::endl;
	save_results();
}

