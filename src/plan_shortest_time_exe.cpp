#include "plan_shortest_time.h"
#include <filesystem>


void print_usage()
{
    std::cout << " Usage: ./plan_short_time <problem JSON file>";
    std::cout << " <states JSON file> <actions JSON file>";
    // TODO: Possible add options?
}

int main(int argc, char** argv)
{
    if(argc < 4)
    {
        std::cerr << " No JSON files provided";
        print_usage();
    }
    else
    {
        //
        const char* prob_json_file_ = argv[1];
        const char* states_json_file_ = argv[2];
        const char* actions_json_file_ = argv[3];
        //
        opt_path_search::PlannerShortestTime planner;
        // TODO: Add -h (help) option support
        try
        {
            if(planner.load_problem_data(prob_json_file_,
                states_json_file_, actions_json_file_))
            {
                if(planner.validate_problem_data())
                {
                    if(planner.calc_shortest_time_path())
                        planner.echo_shortest_time_path();
                    else
                        std::cerr << " Failed to calculate shortest time path\n";
                }
                else
                    std::cerr << " Problem data is inconsistent\n";
            }
            else
            {
                std::cerr << " Failed to load JSON data\n";
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        return 0;        
    }
}