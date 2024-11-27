#pragma once
#include "StateTimeAStar.h"
#include "SingleAgentSolver.h"
// TODO: make SIPP work with edge-weighted graphs


class SIPPNode: public StateTimeAStarNode
{
public:
	SIPPNode* parent;
    Interval interval;

    // the following is used to comapre nodes in the OPEN list
    struct compare_node
    {
        // returns true if n1 > n2 (note -- this gives us *min*-heap).
        bool operator()(const SIPPNode* n1, const SIPPNode* n2) const
        {
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        }
    };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

    // the following is used to comapre nodes in the FOCAL list
    struct secondary_compare_node
    {
        bool operator()(const SIPPNode* n1, const SIPPNode* n2) const // returns true if n1 > n2
        {
            if (n1->conflicts == n2->conflicts)
            {
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                    return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
                return n1->g_val + n1->h_val >= n2->g_val + n2->h_val; // break ties towards smaller f_vals
            }
            return n1->conflicts >= n2->conflicts;  // n1 > n2 if it has more conflicts
        }
    };  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


    // define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
    fibonacci_heap< SIPPNode*, compare<SIPPNode::compare_node> >::
    handle_type open_handle;
    fibonacci_heap< SIPPNode*, compare<SIPPNode::secondary_compare_node> >::
    handle_type focal_handle;


    SIPPNode(): StateTimeAStarNode(), parent(nullptr) {}

    SIPPNode(const State& state, double g_val, double h_val, const Interval& interval,
            SIPPNode* parent, int conflicts):
			StateTimeAStarNode(state, g_val, h_val, nullptr, conflicts), parent(parent), interval(interval)
	{
		if (parent != nullptr)
		{
			depth = parent->depth + 1;
			goal_id = parent->goal_id;
		}
		else
		{
			depth = 0;
			goal_id = 0;
		}
	}


    // The following is used to  check whether two nodes are equal
    // we say that two nodes are equal iff
    // both agree on the id and timestep
    struct EqNode
    {
        bool operator() (const SIPPNode* n1, const SIPPNode* n2) const
        {
            return (n1 == n2) ||
                  (n1 && n2 && n1->state.location == n2->state.location &&
                  n1->state.orientation == n2->state.orientation &&
                  n1->state.velocity == n2->state.velocity &&
                  n1->interval == n2->interval &&
                  n1->goal_id == n2->goal_id);
        }
    };
};

class Primitive{
public:
    struct move{
        // ftt (first-touch-time) = (t_lower) in the paper. 
        // swt (sweeping time) = (t_upper - t_lower) in the paper.
        move(int dx, int dy, int ftt, int swt, int cur_o, bool isEndCell):dx(dx),dy(dy),ftt(ftt),swt(swt),cur_o(cur_o),isEndCell(isEndCell){}
        int dx, dy, ftt, swt, cur_o;
        bool isEndCell; // flag if this cell is still touched at the end of the primitive. `swt` in this case is equal to the remaining time until the end of the primitive.
    };
    vector<move> mvs;
    int o, v;
    Primitive(){
        mvs.clear();
        o = v = 0;
    }
    Primitive(vector<move> mvs, int o, int v):
    mvs(std::move(mvs)),o(o),v(v){}
    friend ostream &operator<<( ostream &output, const Primitive &p ) {
        output << "Moves:"<<endl;
        for(auto it:p.mvs){
            output << "  dx:"<<it.dx<<", dy:"<<it.dy<<", ftt:"<<it.ftt<<", swt:"<<it.swt<<endl;
        }
        output<<"Final_o:"<<p.o<<", Final_v:"<<p.v<<endl;
        return output;
    }
};


class SIPP: public SingleAgentSolver
{
public:
    Path run(const BasicGraph& G, const State& start,
             const vector<pair<int, int> >& goal_location,
             ReservationTable& RT);
    
    void apply_primitive(SIPPNode* curr, const BasicGraph &G, ReservationTable &rt, 
        int t_lower, int t_upper, Primitive mp, double h_val);
    void fill_primitives();

	string getName() const { return "SIPP"; }
    SIPP(): SingleAgentSolver() {}

private:
    vector<Primitive> motion_primitives[MXO][MXV];
    // define typedefs and handles for heap and hash_map
    fibonacci_heap< SIPPNode*, compare<SIPPNode::compare_node> > open_list;
    fibonacci_heap< SIPPNode*, compare<SIPPNode::secondary_compare_node> > focal_list;
    unordered_set< SIPPNode*, SIPPNode::Hasher, SIPPNode::EqNode> allNodes_table;
	inline void releaseClosedListNodes();

    // float max_velocity;
    // float max_acc;

    void generate_node(const Interval& interval, SIPPNode* curr, State next_state, const BasicGraph& G,
                       int min_timestep, double h_val);
    // Updates the path
    Path updatePath(const BasicGraph& G, const SIPPNode* goal);

    void generate_successors(SIPPNode* curr, const BasicGraph &G, 
        ReservationTable &rt, int t_lower, int t_upper, 
        const vector<pair<int, int> >& goal_location);
    // void apply_primitive(SIPPNode* curr, BasicGraph &G, 
    //     ReservationTable &rt, int t_lower, int t_upper, Primitive mp);

};


