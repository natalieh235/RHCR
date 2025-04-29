#pragma once
#include "CBasicSystem.h"
#include "KivaGraph.h"

class CKivaSystem :
	public CBasicSystem
{
public:
	CKivaSystem(const KivaGrid& G, CMAPFSolver& solver);
	~CKivaSystem();

	void simulate(int simulation_time);


private:
	const KivaGrid& G;
	unordered_set<int> held_endpoints;

	void initialize();
	void initialize_start_locations();
	void initialize_goal_locations();
	void update_goal_locations();
};

