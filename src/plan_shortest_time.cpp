#include "plan_shortest_time.h"

namespace opt_path_search
{
    using json = nlohmann::json;
    //
    std::ostream& operator<<(std::ostream& os, const ActionInfo<int> action) { 
        os << " State start: " << action.state_start << "\n"
        << " State end: " << action.state_end << "\n"
        << " Cost(time): " << action.cost << "\n";
        return os;
    }
    // /
    PlannerShortestTime::PlannerShortestTime()
    {
        //
        prob_data_valid = false;
    }
    //
    PlannerShortestTime::~PlannerShortestTime()
    {}
    //
    bool PlannerShortestTime::load_problem_data(
            const std::string prob_json_file_,
            const std::string states_json_file_,
            const std::string actions_json_file_)
    {
        // Check if the provided JSON files exist in the system
        if(!std::filesystem::exists(prob_json_file_))
        {
            std::cerr << " File:" << prob_json_file_ << " does not exist";
            return false;
        }
        if(!std::filesystem::exists(states_json_file_))
        {
            std::cerr << " File:" << states_json_file_ << " does not exist";
            return false;
        }
        if(!std::filesystem::exists(actions_json_file_))
        {
            std::cerr << " File:" << actions_json_file_ << " does not exist";
            return false;
        }
        //
        std::cout << "\n\n JSON files provided:\n"
            << " Problem: " << prob_json_file_ << std::endl
            << " States: " << states_json_file_ << std::endl
            << " Actions: " << actions_json_file_ << std::endl;
        //
        try
        {
            // Parse data from JSON files
            json prob_json = json::parse(std::ifstream(prob_json_file_));
            json states_json = json::parse(std::ifstream(states_json_file_));
            json actions_json = json::parse(std::ifstream(actions_json_file_));            
            //
            // Load start and goal state
            //
            start_state = prob_json["init"];
            goal_state = prob_json["goal"];
            std::cout << "\n Problem description: \n";
            std::cout << "\t Start state: " << start_state << std::endl
                << "\t Goal state: " << goal_state << std::endl;
            int idx;
            //
            // Load the list of states
            //
            list_of_states = states_json["states"];
            std::cout << "\n List of states:" << std::endl;
            for(idx = 0; idx < (int)list_of_states.size(); idx++)
                std::cout << "\t [" << idx << "] " << list_of_states.at(idx) << std::endl;
            //
            // Load the list of actonis
            //
            action_info_map.clear();
            idx = 0;
            std::cout << "\n List of actions:" << std::endl;
            //
            for(const auto& action_ : (*actions_json.begin()))
            {
                // We want to maintain a map of possible actions
                // that can be taken from given state.
                state_action_map[action_["state_start"]].push_back(action_["action"]);
                //
                ActionInfo<int> info_;
                info_.state_start = action_["state_start"];
                info_.state_end = action_["state_end"];
                info_.cost = action_["time"];
                // Also store relevant action information
                action_info_map[action_["action"]] = info_;
                //
                std::cout << " Action id [" << idx++ << "]: " 
                    << action_["action"]  << "\n"
                    << action_info_map[action_["action"]] << std::endl;
            }
            //
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
        //
        return true;
    }
    //
    bool PlannerShortestTime::validate_problem_data()
    {
        if(std::find(list_of_states.begin(), list_of_states.end(), start_state) == list_of_states.end())
        {
            std::cerr << " Start state: " << start_state << " not found in list of states";
            return false;
        }
        //
        if(std::find(list_of_states.begin(), list_of_states.end(), goal_state) == list_of_states.end())
        {            
            std::cerr << " Goal state: " << goal_state << " not found in list of states";
            return false;
        }
        //
        // TODO (abhi): Do we want to make sure a path exists or 
        // existence of path is delegated to the calculation of 
        // shortest time path method.
        //
        prob_data_valid = true;
        //
        return true;
    }
    //
    bool PlannerShortestTime::calc_shortest_time_path()
    {
        if(!prob_data_valid)
        {
            std::cerr << " Problem data not validated yet.\n";
            return false;
        }
        //

        return true;
    }
    //
    void PlannerShortestTime::echo_shortest_time_path()
    {

    }   

} // namespace opt_path_search
