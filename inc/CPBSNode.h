#pragma once
#include "common.h"
#include "PriorityGraph.h"
#include "CStates.h"

class CPBSNode
{
public:
	// the following is used to comapre nodes in the OPEN list
	struct compare_node 
	{
		bool operator()(const CPBSNode* n1, const CPBSNode* n2) const
		{
			// order by cost, break ties by # of collisions
            if (n1->f_val == n2->f_val)
            {
                return n1->num_of_collisions >= n2->num_of_collisions;
            }
			return n1->f_val >= n2->f_val;
		}
	};  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)


	typedef fibonacci_heap< CPBSNode*, compare<CPBSNode::compare_node> >::
	    handle_type open_handle_t;
	// typedef fibonacci_heap< CPBSNode*, compare<CPBSNode::secondary_compare_node> >::
	//     handle_type focal_handle_t;
	// open_handle_t open_handle;
	// focal_handle_t focal_handle;


	// conflicts in the current paths
	std::list<CConflict> conflicts;
	
	// The chosen conflict
	CConflict conflict;

	CPBSNode* parent;


    list< pair<int, CPath> > paths; // <agent_id, path>
    std::pair<int, int> priority; // a1 < a2

    PriorityGraph priorities;

	double g_val;
	double h_val;
	double f_val;
	size_t depth; // depath of this CT node
	size_t makespan; // makespan over all paths
	int num_of_collisions; // number of conflicts in the current paths
    int earliest_collision;

	uint64_t time_expanded;
	uint64_t time_generated;


    void print_priorities() const;

	void clear();

	CPBSNode(): parent(nullptr), g_val(0), h_val(0), earliest_collision(INT_MAX), time_expanded(0) {}
	~CPBSNode(){};


private:
};

