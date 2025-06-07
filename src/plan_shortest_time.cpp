#include "plan_shortest_time.h"

namespace opt_path_search
{
    using json = nlohmann::json;
    //
    std::ostream& operator<<(std::ostream& os, const ActionInfoPtr action) { 

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
            // Load the list of states
            //
            list_of_states = states_json["states"];
            std::cout << "\n List of states:" << std::endl;
            //
            uint idx = 0;
            for(const auto& state_ : list_of_states)
            {
                std::cout << "\t [" << idx++ << "] " << state_ << std::endl;
                state_info_map[state_] = std::make_shared<StateInfo>(state_);
            }
            //
            // Load start and goal state
            //
            start_state = prob_json["init"];
            goal_state = prob_json["goal"];
            std::cout << "\n Problem description: \n";
            std::cout << "\t Start state: " << start_state << std::endl
                << "\t Goal state: " << goal_state << std::endl;
            //
            // Load the list of actonis
            //
            action_info_map.clear();
            std::cout << "\n List of actions:" << std::endl;
            //
            idx = 0;
            for(const auto& action_ : (*actions_json.begin()))
            {
                //
                ActionInfoPtr action_info_ = std::make_shared<ActionInfo>();
                action_info_->name = action_["action"];
                action_info_->state_start = action_["state_start"];
                action_info_->state_end = action_["state_end"];
                action_info_->cost = action_["time"];
                // Also store relevant action information
                action_info_map[action_["action"]] = action_info_;
                // Populate list of actions possible from this vertex
                StateInfoPtr state_info_ = state_info_map[action_info_->state_start];
                state_info_->action_list.push_back(action_info_->name);
                //
                std::cout << "\n\n Action id ["
                    << idx++ << "]: " << action_info_->name 
                    << "\n State start: " << action_info_->state_start << "\n"
                    << " State end: " << action_info_->state_end << "\n"
                    << " Cost(time): " << action_info_->cost << "\n\n";
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
        //
        StateQueue state_queue_;
        std::set<std::string> state_visited_;
        //
        std::string curr_vertex = start_state;
        state_info_map[curr_vertex]->cost_from_start = 0;
        //
        state_queue_.push(state_info_map[curr_vertex]);
        //
        state_visited_.clear();
        // TODO: Add max number of iterations
        while(!state_queue_.empty() &&
            state_visited_ != list_of_states)
        {
            // Get node with least cost
            const StateInfoPtr curr_vertex_info_ = state_queue_.top();
            state_queue_.pop();
            //
            // Get neighbours
            for(const auto& action_  : curr_vertex_info_->action_list)
            {
                const ActionInfoPtr action_info_ = action_info_map[action_];
                std::string neighbour_ = action_info_->state_end;
                // std::cout << " Node: " << curr_vertex_info_->name 
                //     << " checking neighbour:" << neighbour_ << std::endl;
                // check if neighbour is visited
                if(state_visited_.find(neighbour_) == state_visited_.end())
                {
                    StateInfoPtr neighbour_vertex_info_  = state_info_map[neighbour_];
                    // if not visited, check cost to get there
                    // cost of going from current vertex to next vertex
                    const uint32_t action_cost_ = action_info_->cost;
                    // 
                    const uint32_t cost_from_start = 
                        curr_vertex_info_->cost_from_start + action_cost_;
                    //
                    // std::cout << " Cost from start:" << cost_from_start 
                    //     << " current cost:" <<  neighbour_vertex_info_->cost_from_start 
                    //     << std::endl;
                    //
                    // if cost to get there is lower than its current cost
                    if(cost_from_start < neighbour_vertex_info_->cost_from_start)
                    {
                        // std::cout << " Updating lowest cost\n";
                        // update cost-to-get there
                        neighbour_vertex_info_->cost_from_start = cost_from_start;
                        // update previous node
                        neighbour_vertex_info_->prev_state = curr_vertex_info_->name;
                        // if update, push on queue
                        state_queue_.push(neighbour_vertex_info_);
                    }

                }
                //
            }
            // after visiting all neighbours mark this node as visited
            state_visited_.insert(curr_vertex_info_->name);
        }
        //
        std::cout << "\n\n Done traversing graph\n\n";
        //
        for(const auto& info_ : state_info_map)
        {
            std::cout << " Node: " << info_.first 
                << " cost: "  << info_.second->cost_from_start
                << " prev node:" << info_.second->prev_state << std::endl;
        }
        //
        std::string cv_ = goal_state;
        shortest_path.push_back(cv_);
        //
        while(cv_ != start_state ||
            state_info_map[cv_]->prev_state != NULL_STATE_STR)
        {
            // std::cout << " Adding vertex:" << cv_ << std::endl;
            cv_ = state_info_map[cv_]->prev_state;
            shortest_path.push_back(cv_);
        }
        //
        std::cout << "\n\n Done calculating shortest path \n\n";
        for(const std::string& state_ : shortest_path)
            std::cout << state_ << "<--";
        std::cout << std::endl;

        return true;
    }
    //
    void PlannerShortestTime::echo_shortest_time_path()
    {

    }   

} // namespace opt_path_search
