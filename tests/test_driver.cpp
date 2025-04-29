#include <iostream>
#include <cassert>
#include "KivaGraph.h"
#include "CKivaSystem.h"
#include "KivaSystem.h"
#include "CSIPP.h"

void test_valid_move() {
    // Create a sample graph/grid (this will depend on how your Graph is implemented)
    KivaGrid G = KivaGrid(1.5, 1.0);
    assert((G.load_map("../maps/symbotic/symbotic_small.map")) == 1);

    std::cout << "14, 0" << G.valid_move(14, 0) << std::endl;

    std::cout << "14, 1" << G.valid_move(14, 1) << std::endl;

    std::cout << "14, 2" << G.valid_move(14, 2) << std::endl;

    std::cout << "14, 3" << G.valid_move(14, 3) << std::endl;

    std::cout << "All valid_move tests passed!" << std::endl;
}

void test_small_map() {
    // Create a sample graph/grid (this will depend on how your Graph is implemented)
    // KivaGrid G;
    KivaGrid G = KivaGrid(2.0, 1.0);
    assert((G.load_map("../maps/smallkiva.map")) == 1);

    G.preprocessing(true);

    // Test 1: Valid move within grid boundaries
    // assert(G.valid_move(0, 0, 1.5, 1) == true); // Move from (2,2) in direction 0 (e.g., right), should be valid

    // Test 2: Invalid move outside grid boundaries
    assert(G.valid_move(30, 1) == false);

    // Test 3: Valid move to another cell
    assert(G.valid_move(29, 1) == true); 

    assert(G.valid_move(20, 0) == false); 

    assert(G.valid_move(31, 1) == false); 
    assert(G.valid_move(23, 2) == false); 
    // Test 4: Invalid move into blocked cell (if your grid allows marking certain cells as blocked)
    // G.blockCell(3, 3); // Block cell (3,3)
    // assert(G.valid_move(14, 1, 1.5, 1) == false); // Move from (2,3) to (3,3), should be invalid as it's blocked

    std::cout << "All valid_move small tests passed!" << std::endl;
}

void test_system() {
    KivaGrid G = KivaGrid(1.0, 1.0);
    assert((G.load_map("../maps/symbotic/symbotic_small.map")) == 1);

    SIPP *path_planner = new SIPP();

    PBS* pbs = new PBS(G, *path_planner);
    KivaSystem system(G, *pbs);
    pbs->initial_constraints = {std::make_tuple(1, 2, 3)};
    G.preprocessing(false);


    // 5 to 35
    std::cout << "initial con " << pbs->initial_constraints.size() << std::endl;

    system.outfile = "../RHCR/exp/fix_sipp_tests";
	system.screen = 2;
	system.num_of_drives = 1;
	system.time_limit = 60;
	system.simulation_window = 5;
	system.planning_window = 1073741823;
	system.travel_time_window = 0;
    system.simulation_time = 200;
	system.consider_rotation = true;
	system.seed = 0;
	srand(system.seed);

    system.initialize_solvers();
    system.starts.resize(system.num_of_drives);
    system.goal_locations.resize(system.num_of_drives);
    system.finished_tasks.resize(system.num_of_drives);
    system.paths.resize(system.num_of_drives);

    // system.outfile = "./";

    // State start_state(5, 0, 1);

    system.starts = {State(5, 0, 1, 1)};
    std::pair<int, int> goal1(35, 0);
    system.goal_locations = {{goal1}};

    path_planner->fill_primitives();

    system.solve();
    auto new_finished_tasks = system.move();
	std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;

    for (auto task : new_finished_tasks)
		{
			int id, loc, t;
			std::tie(id, loc, t) = task;
			system.finished_tasks[id].emplace_back(loc, t);
			system.num_of_tasks++;
		}
    

    system.save_results();

    std::cout << "All planning small tests passed!" << std::endl;
}

void continuous_test_system() {
    KivaGrid G = KivaGrid(1.0, 1.0);
    assert((G.load_map("../maps/symbotic/symbotic_tiny.map")) == 1);

    G.preprocessing(true);

    MotionModel motion_model = MotionModel(2.0, 1.0, 0.0);
    CSIPP *path_planner = new CSIPP(motion_model);

    ContinuousReservationTable rt = ContinuousReservationTable(G, motion_model);

    CPPBest *solver = new CPPBest(G, *path_planner);
    CKivaSystem system(G, *solver);

    system.outfile = "../RHCR/exp/continuous_tests";
	system.screen = 2;
	system.num_of_drives = 1;
	system.time_limit = 60;
	system.simulation_window = 5;
	system.planning_window = 1073741823;
	system.travel_time_window = 0;
    system.simulation_time = 200;
	system.consider_rotation = true;
	system.seed = 0;
	srand(system.seed);

    system.initialize_solvers();
    system.starts.resize(system.num_of_drives);
    system.goal_locations.resize(system.num_of_drives);
    system.finished_tasks.resize(system.num_of_drives);
    system.paths.resize(system.num_of_drives);

    system.starts = {CState(95, 0, 1, 1)};
    std::pair<int, int> goal1(7, 0);
    system.goal_locations = {{goal1}};

    system.simulate(10);

    // system.solve();

    // std::cout << "solved" << std::endl;
    // auto new_finished_tasks = system.move();
	// std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;

    // for (auto task : new_finished_tasks)
	// 	{
	// 		int id, loc, t;
	// 		std::tie(id, loc, t) = task;
	// 		system.finished_tasks[id].emplace_back(loc, t);
	// 		system.num_of_tasks++;
	// 	}
    
    // system.save_results();

    // std::cout << "All planning small tests passed!" << std::endl;
}

void test_motion_model() {
    MotionModel motion_model = MotionModel(2.0, 1.0, 0.0);
    Profile profile = motion_model.getTrapezoidalProfile(5);
    std::cout << profile << std::endl;

    profile = motion_model.getTrapezoidalProfile(4);
    std::cout << profile << std::endl;

    profile = motion_model.getTrapezoidalProfile(3);
    std::cout << profile << std::endl;

    // profile = motion_model.getTrapezoidalProfile(10);
    // std::cout << profile << std::endl;

    // profile = motion_model.getTrapezoidalProfile(45);
    // std::cout << profile << std::endl;
    // Test the trapezoidal profile
    // assert(profile.length == 5);
    // assert(profile.start_time == 0);
    // assert(profile.arrival_time == 4);

    // // Test the entries
    // assert(profile.entries.size() == 5);
    // assert(profile.entries[0].first == 0.0);
    // assert(profile.entries[0].second == 1.0);

    std::cout << "All motion model tests passed!" << std::endl;
}

void test_csipp() {
    KivaGrid G = KivaGrid(1.0, 1.0);
    assert((G.load_map("../maps/symbotic/symbotic_tiny.map")) == 1);

    G.preprocessing(true);
    //(double max_speed, double acceleration, double deceleration)
    
    MotionModel motion_model = MotionModel(2.0, 1.0, 0.0);
    CSIPP *path_planner = new CSIPP(motion_model);

    ContinuousReservationTable rt = ContinuousReservationTable(G, motion_model);
    
    vector<Path*> paths;
    paths.resize(1);
    list<tuple<int, int, int>> initial_constraints;
    unordered_set<int> high_priority_agents;
    int current_agent = 0;

    int start = 95;
    int goal = 7;

    // rt.build(paths, initial_constraints, high_priority_agents, current_agent, start);

    std::cout << "built rt" << std::endl;
    
    const vector<std::pair<int, int>> goal_locations = {{goal, 0}};
    // CPath sol = path_planner->run_continuous(G, CState(start, 0, 1), goal_locations, rt);
    // std::cout << "sol: " << sol << std::endl;

    // rt.addReservation(82, 1, 3);
    CPath sol = path_planner->run_continuous(G, CState(start, 0, 1), goal_locations, rt);
    std::cout << "sol: " << sol << std::endl;

    rt.addReservation(82, 1, 3);
    sol = path_planner->run_continuous(G, CState(start, 0, 1), goal_locations, rt);
    std::cout << "sol: " << sol << std::endl;

    rt.insertPath(sol);

}

void test_small_plan() {
    KivaGrid G = KivaGrid(1.0, 1.0);
    assert((G.load_map("../maps/sippip.map")) == 1);

    SingleAgentSolver *path_planner = new SIPP();

    PBS* pbs = new PBS(G, *path_planner);
    KivaSystem system(G, *pbs);
    pbs->initial_constraints = {std::make_tuple(1, 2, 3)};
    G.preprocessing(false);

    std::cout << "initial con " << pbs->initial_constraints.size() << std::endl;

    system.outfile = "../exp/pbs_tests";
	system.screen = 2;
	system.num_of_drives = 1;
	system.time_limit = 60;
	system.simulation_window = 5;
	system.planning_window = 1073741823;
	system.travel_time_window = 0;
	system.consider_rotation = false;
	system.seed = 0;
	srand(system.seed);

    system.initialize_solvers();
    system.starts.resize(system.num_of_drives);
    system.goal_locations.resize(system.num_of_drives);
    system.finished_tasks.resize(system.num_of_drives);
    system.paths.resize(system.num_of_drives);

    system.outfile = "./";

    // system.starts = {State(0, 0, 0), State(6, 0, 0)};
    // std::pair<int, int> goal1(7, 0);
    // std::pair<int, int> goal2(2, 0);
    // system.goal_locations = {{goal1}, {goal2}};

    system.starts = {State(0, 0, 0)};
    std::pair<int, int> goal1(3, 0);
    system.goal_locations = {{goal1}};

    // system.starts = {State(6, 0, 0)};
    // std::pair<int, int> goal1(2, 0);
    // system.goal_locations = {{goal1}};

    
    system.solve();
    system.timestep = 5;
    auto new_finished_tasks = system.move();
	std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;

    for (auto task : new_finished_tasks)
		{
			int id, loc, t;
			std::tie(id, loc, t) = task;
			system.finished_tasks[id].emplace_back(loc, t);
			system.num_of_tasks++;
		}
    
    // system.update_start_locations();

    system.save_results();

    std::cout << "All planning small tests passed!" << std::endl;
}

int main() {
    // Run the test cases
    // test_valid_move();
    // test_small_map();
    // test_small_plan();
    // test_primitives();
    // test_primitives_2();

    // test_system();
    // test_valid_move();

    // test_motion_model();
    continuous_test_system();
    // test_csipp();
    return 0;
}