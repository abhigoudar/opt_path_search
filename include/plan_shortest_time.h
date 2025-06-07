/* ----------------------------------------------------------------------------

 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef _PLAN_SHORT_TIME_HEADER_H_
#define _PLAN_SHORT_TIME_HEADER_H_

#include <iostream>
#include <fstream>
#include "json.hpp"
#include <queue>
#include <set>

namespace opt_path_search
{
    //
    #define NULL_STATE_STR "NULL"
    //
    using json = nlohmann::json;
    // Structure for holding action-related attributes
    // We treat time as cost in this setting.
    // Shortest time -> Lowest cost
    struct ActionInfo{
        // name of this action
        std::string name;
        // starting state for this action
        std::string state_start;
        // goal state for this action
        std::string state_end;
        // cost associated with this action
        uint32_t cost;
    };
    // alias for shared pointer for type StateInfo
    using ActionInfoPtr = std::shared_ptr<ActionInfo>;
    //
    // Structure for holding state-related attributes.
    // The optimal previous state, action, and cost from start
    // for this state are tored.
    // cost from start -> time from start
    struct StateInfo
    {
        StateInfo() = delete;
        //
        StateInfo(const StateInfo& other)
        {
            name = other.name;
            cost_from_start = other.cost_from_start;
            action_list = other.action_list;
            prev_state = other.prev_state;
            opt_action = other.opt_action;
        }
        //
        StateInfo(const std::string name_)
        {
            name = name_;
            cost_from_start = std::numeric_limits<uint32_t>::max();
            action_list.clear(); 
            prev_state = NULL_STATE_STR;
            opt_action = NULL_STATE_STR;
        }
        //
        std::string name; // name of this state
        std::string prev_state; // previous lowest cost state
        std::string opt_action; // optimal action to previous state
        uint32_t cost_from_start; // cost of reaching this state from start_state
        std::vector<std::string> action_list; // list of actions from this vertex
    };
    //
    // alias for shared pointer for type StateInfo
    using StateInfoPtr = std::shared_ptr<StateInfo>;
    // 
    // Function to compare costs to reach two states
    // 
    struct StateCostCompare{
        bool operator()(const StateInfo v1, const StateInfo v2){
            return v1.cost_from_start > v2.cost_from_start;
        }
        //
        bool operator()(const StateInfoPtr& v1, const StateInfoPtr& v2){
            return v1->cost_from_start > v2->cost_from_start;
        }
    };
    // Container to store states. The above function (StateCostCompare)
    // ensures states arr stored with decreasing cost from start.
    using StateQueue = std::priority_queue<StateInfoPtr,
        std::vector<StateInfoPtr>, StateCostCompare>;

    // Class with necessary methods for calculating
    // the shortest time path
    class PlannerShortestTime{
        public:
        /*
        * @brief Constructor
        */
        PlannerShortestTime();
        /*
        * @brief default destructor
        */
        ~PlannerShortestTime();
        /*
        * @brief Loads data from JSON files
        * @param fully-resolved path of JSON problem file
        * @param fully-resolved path of JSON list of states file
        * @param fully-resolved path of JSON list of actions file
        * @return True if data is successfully loaded, false otherwise
        */
        bool load_problem_data(const std::string problem_json_file_,
            const std::string states_json_file_,
            const std::string actions_json_file_);
        /*
        * @brief Checks if problem data from JSON files is consistent
        * Problem is inconsistent if a path from start to goal does not exist
        * @return True if data is problem data is valid, false otherwise
        */
        bool validate_problem_data();
        /*
        * @brief calculate the shortes time path using Dijkstra's method
        * @return True if path was caculated, False otherwise
        */
        bool calc_shortest_time_path();
        /*
        * @brief prints the shortes time path
        * @param fully-resolved path for output JSON data
        */
        void echo_shortest_time_path(const std::string);
        //
        private:
        /*
        * @brief prints problem data
        */
        void echo_problem_data();
        //
        std::string start_state, goal_state;
        // list of states
        std::set<std::string> list_of_states;
        // container for action-related info
        std::map<std::string, ActionInfoPtr> action_info_map;
        // container for state-related info
        std::map<std::string, StateInfoPtr> state_info_map;
        // container for states in the shortest path
        std::deque<std::string> shortest_path;
        // container for optimal actions for shortest path
        std::deque<std::string> optimal_actions;
        // flag indicating if data is valid
        bool prob_data_valid;
    }; // class PlannerShortTime

} // namespace opt_path_search

#endif // _PLAN_SHORT_TIME_HEADER_H_