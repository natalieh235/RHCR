#include "CKivaSystem.h"
#include "LRAStar.h"
#include "PBS.h"

CKivaSystem::CKivaSystem(const KivaGrid& G, CMAPFSolver& solver): CBasicSystem(G, solver), G(G) {}


CKivaSystem::~CKivaSystem()
{
}

void CKivaSystem::initialize()
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
	// bool succ = load_records(); // continue simulating from the records
	// std::cout << " load records"<< std::endl;
	// if (!succ)
	// {
		timestep = 0;
		// succ = load_locations();
		// if (!succ)
		// {
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
		// }
	// }
}

void CKivaSystem::initialize_start_locations()
{
	// Choose random start locations
	// Any non-obstacle locations can be start locations
	// Start locations should be unique
	// std::cout << "initializing start location" << consider_rotation << std::endl;

	// std::vector<std::pair<int, int>> hardcoded_starts = {
	// 	{131, 1}, 
	// 	{134, 1},
	// 	{137, 1},
	// };

	for (int k = 0; k < num_of_drives; k++)
	{
		int orientation = -1;
		int start_location = G.agent_home_locations[k];
		if (consider_rotation) {
			orientation = rand() % 4;
			while (!G.valid_move(start_location, orientation)) {
				// std::cout << "invalid orientation " << orientation << std::endl;
				orientation = rand() % 4;
			}
		}

		// int start_location = hardcoded_starts[k].first;
    	// int orientation = hardcoded_starts[k].second;

		starts[k] = CState(start_location, 0, orientation);
		// starts[k] = State(G.agent_home_locations[k], 0, orientation);
		std::cout << "start for agent " << k << " is " << starts[k] << std::endl;
		paths[k].emplace_back(std::make_tuple(starts[k], 0.0));
		finished_tasks[k].emplace_back(start_location, 0);
	}
}


void CKivaSystem::initialize_goal_locations()
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



void CKivaSystem::update_goal_locations()
{
	std::cout << "update goal locations, " << hold_endpoints << std::endl;
    if (!LRA_called)
        new_agents.clear();
	

    for (int k = 0; k < num_of_drives; k++)
    {
        // int curr = paths[k][timestep].state.location; // current location
        int curr = std::get<0>(paths[k].back()).location;

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


void CKivaSystem::simulate(int simulation_time)
{
	std::cout << "*** Simulating " << seed << " ***" << std::endl;
	this->simulation_time = simulation_time;
	initialize();

	std::cout << "Kiva System initialized" << std::endl;

	for (; timestep < simulation_time; timestep += simulation_window)
	{
		std::cout << "\n SYSTEM SIMULATION: Timestep " << timestep << "\n" << std::endl;

		update_start_locations();
		update_goal_locations();
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
		}

		// if (congested())
		// {
		// 	cout << "***** Too many traffic jams ***" << endl;
		// 	break;
		// }
	}

	update_start_locations();
	std::cout << std::endl << "AM I DONE!" << std::endl;
	std::cout << "Solver total runtime: " << solver.total_runtime << std::endl;
	save_results();
}

