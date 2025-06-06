/* ----------------------------------------------------------------------------

 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef _PLAN_SHORT_TIME_HEADER_H_
#define _PLAN_SHORT_TIME_HEADER_H_

#include <iostream>
#include <fstream>
#include "json.hpp"

namespace opt_path_search
{
    using json = nlohmann::json;
    // forward declaration
    template <typename CostType> struct ActionInfo;
    template <typename CostType> std::ostream& operator<<(std::ostream& os, const ActionInfo<CostType> a);
    // Structure for holding action-related attributes
    // We treat time as cost in this setting.
    // Shortest time -> Lowest cost
    template <typename CostType>
    struct ActionInfo{
        // starting state for this action
        std::string state_start;
        // goal state for this action
        std::string state_end;
        // cost associated with this action
        CostType cost;
        // overload ostream for custom Action structure
        friend std::ostream& operator<< <CostType>(std::ostream& os, const ActionInfo<CostType> a);
    };
    //
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
        * @brief calculate the shortes time path
        * @return True if path was caculated, False otherwise
        */
        bool calc_shortest_time_path();
        /*
        * @brief prints the shortes time path
        */
        void echo_shortest_time_path();
        //
        private:
        /*
        * @brief prints problem data
        */
        void echo_problem_data();
        //
        std::string start_state, goal_state;
        //
        std::vector<std::string> list_of_states;
        //
        std::map<std::string, ActionInfo<int>> action_info_map;
        //
        std::map<std::string, std::vector<std::string>> state_action_map;
        //
        bool prob_data_valid;
    }; // class PlannerShortTime

} // namespace opt_path_search

#endif // _PLAN_SHORT_TIME_HEADER_H_