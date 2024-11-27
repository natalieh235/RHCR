#include <iostream>
#include <cassert>
#include "KivaGraph.h"
#include "KivaSystem.h"
#include "KivaSystem.h"

void test_valid_move() {
    // Create a sample graph/grid (this will depend on how your Graph is implemented)
    KivaGrid G = KivaGrid(2.0, 1.0);
    G.load_map("../maps/testkiva.map");
    G.preprocessing(true);

    // Test 1: Valid move within grid boundaries
    assert(G.valid_move(0, 0) == true); // Move from (2,2) in direction 0 (e.g., right), should be valid

    // Test 3: Valid move to another cell
    assert(G.valid_move(5, 0) == true); 

    // Test 3: Valid move to another cell
    assert(G.valid_move(5, 1) == true); 

    // Test 4: Invalid move into blocked cell (if your grid allows marking certain cells as blocked)
    // G.blockCell(3, 3); // Block cell (3,3);
    assert(G.valid_move(14, 1) == false); // Move from (2,3) to (3,3), should be invalid as it's blocked

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

void test_generate_primitive() {
    KivaGrid G = KivaGrid(2.0, 1.0);
    G.load_map("../maps/testkiva.map");
    G.preprocessing(true);

    SIPP *planner = new SIPP();
}

void test_system() {
     KivaGrid G = KivaGrid(1.0, 1.0);
    assert((G.load_map("../maps/symbotic_small.map")) == 1);

    SIPP *path_planner = new SIPP();

    PBS* pbs = new PBS(G, *path_planner);
    KivaSystem system(G, *pbs);
    pbs->initial_constraints = {std::make_tuple(1, 2, 3)};
    G.preprocessing(false);

    std::cout << "initial con " << pbs->initial_constraints.size() << std::endl;

    system.outfile = "../exp/symbotic_pbs_tests";
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

    system.starts = {State(131, 0, 1)};
    std::pair<int, int> goal1(20, 0);
    system.goal_locations = {{goal1}};

    path_planner->fill_primitives();

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
    

    system.save_results();

    std::cout << "All planning small tests passed!" << std::endl;
}

void test_primitives_2() {
    KivaGrid G = KivaGrid(1.0, 1.0);
    assert((G.load_map("../maps/sippip.map")) == 1);

    G.preprocessing(true);

    SIPP *planner = new SIPP();
    ReservationTable *rt = new ReservationTable(G);

    vector<tuple<int, int, int>> constraints = {
        {0, 6, INTERVAL_MAX},
        {2, 0, 5}
    };

    for (auto c: constraints) {
        rt->insertConstraint2SIT(std::get<0>(c), std::get<1>(c), std::get<2>(c));
    }

    planner->fill_primitives();

    SIPPNode curr = SIPPNode(
        State(1, 0, 0, 0), 
        0,
        1,
        {0, 6, 0},
        nullptr,
        0
    );

    planner->run(G, State(0, 0, 0, 0), {make_pair(3, 0)}, *rt);
}

void test_primitives() {
    KivaGrid G = KivaGrid(1.0, 1.0);
    assert((G.load_map("../maps/sippip_prim.map")) == 1);

    G.preprocessing(false);

    SIPP *planner = new SIPP();
    ReservationTable *rt = new ReservationTable(G);

    vector<tuple<int, int, int>> constraints = {{3, 0, 2}, {3, 20, INTERVAL_MAX},
        {1, 0, 2}, {1, 18, INTERVAL_MAX},
        {2, 2, 5}, {2, 15, 16}, {2, 21, INTERVAL_MAX}, 
        {0, 1, 6}, {0, 11, 12}, {0, 22, INTERVAL_MAX}};

    for (auto c: constraints) {
        rt->insertConstraint2SIT(std::get<0>(c), std::get<1>(c), std::get<2>(c));
    }

    Primitive turn;
    turn.mvs.push_back(Primitive::move(1, 0, 0, 3, 0, 0));
    turn.mvs.push_back(Primitive::move(1, -1, 2, 2, 0, 0));
    turn.mvs.push_back(Primitive::move(0, -1, 3, 2, 0, 1));

    SIPPNode curr = SIPPNode(
        State(1, 0, -1, 0), 
        0,
        1,
        {2, 17, 0},
        nullptr,
        0
    );

    // planner->apply_primitive(&curr, G, *rt, 2, 17, turn);

}

void test_small_plan() {
    // Create a sample graph/grid (this will depend on how your Graph is implemented)
    // KivaGrid G;
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

    test_system();
    return 0;
}