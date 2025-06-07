#include "plan_shortest_time.h"

namespace opt_path_search
{
    using json = nlohmann::json;
    // /
    PlannerShortestTime::PlannerShortestTime()
    {
        prob_data_valid = false;
    }
    //
    PlannerShortestTime::~PlannerShortestTime()
    {}
    //
    bool PlannerShortestTime::check_file_exists(const std::string path)
    {
        std::ifstream file(path);
        return file.good();
    }
    //
    bool PlannerShortestTime::load_problem_data(
            const std::string prob_json_file_,
            const std::string states_json_file_,
            const std::string actions_json_file_)
    {
        // Check if the provided JSON files exist in the system
        if(!check_file_exists(prob_json_file_))
        {
            std::cerr << " File:" << prob_json_file_ << " does not exist";
            return false;
        }
        if(!check_file_exists(states_json_file_))
        {
            std::cerr << " File:" << states_json_file_ << " does not exist";
            return false;
        }
        if(!check_file_exists(actions_json_file_))
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
            std::cerr <<" Failed to load JSON data\n";
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
        try
        {
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
            // The following loop implements Dijkstra's algorithm
            // Nodes and States are used interchangeably
            // Actions and Edges are used interchangeably
            while(!state_queue_.empty() &&
                state_visited_ != list_of_states)
            {
                // Get node with least cost
                const StateInfoPtr curr_vertex_info_ = state_queue_.top();
                state_queue_.pop();
                //
                // Get neighbours of this node
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
                            // record the action to get to previous node
                            neighbour_vertex_info_->opt_action = action_;
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
                    << " prev node:" << info_.second->prev_state
                    << " opt. action:" << info_.second->opt_action << std::endl;
            }
            //
            std::string cv_ = goal_state;
            // Traverse backwards from goal to store
            // the optimal path and the actions
            while(cv_ != start_state)
            {
                // TODO: handle the bottom case
                // state_info_map[cv_]->prev_state != NULL_STATE_STR
                // std::cout << " Adding vertex:" << cv_ << std::endl;
                shortest_path.push_front(cv_);
                optimal_actions.push_front(state_info_map[cv_]->opt_action);
                cv_ = state_info_map[cv_]->prev_state;
            }
            //
            shortest_path.push_front(start_state);

            return true;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
    }
    //
    void PlannerShortestTime::echo_shortest_time_path(
        const std::string output_file_path)
    {
        //
        std::cout << "\n\n Done calculating shortest path \n\n";
        while(!shortest_path.empty())
        {
            auto state_ = shortest_path.front();
            std::cout << "-->" << state_;
            shortest_path.pop_front();
        }
        std::cout << std::endl;
        //
        uint32_t total_cost = 0;
        std::cout << "\n\n Optimal action list (end to start) \n\n";
        auto opt_act_it = optimal_actions.begin();
        while(opt_act_it != optimal_actions.end())
        {
            auto action_ = *opt_act_it;
            std::cout << "-->" << action_;
            total_cost += action_info_map[action_]->cost;
            opt_act_it++;
        }
        std::cout << std::endl;
        //
        std::cout << "\n ----------- Total time of shortest path: " << 
            total_cost << "\n\n";
        //
        json output_json;
        output_json["actions"] = optimal_actions;
        output_json["cost"] = total_cost;
        std::cout <<" Writing optimal action list to: " 
            << output_file_path << "\n\n";
        std::ofstream output_file(output_file_path, std::ios_base::out);
        output_file << output_json << std::endl;
        output_file.close();
    }
} // namespace opt_path_search
