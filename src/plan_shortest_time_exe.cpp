#include "plan_shortest_time.h"

void print_usage()
{
    std::cout << " Usage: ./plan_short_time <problem JSON file>";
    std::cout << " <states JSON file> <actions JSON file> \n";
}

int main(int argc, char** argv)
{
    if(std::string(argv[1]) == "--help"){
        print_usage();
        return 0;
    }

    if(argc < 4)
    {
        std::cerr << " No JSON files provided \n";
        print_usage();
    }
    else
    {
        //
        std::string prob_json_file_(argv[1]);
        std::string states_json_file_(argv[2]);
        std::string actions_json_file_(argv[3]);
        //
        opt_path_search::PlannerShortestTime planner;
        //
        try
        {
            if(planner.load_problem_data(prob_json_file_,
                states_json_file_, actions_json_file_))
            {
                if(planner.validate_problem_data())
                {
                    if(planner.calc_shortest_time_path()){
                        //
                        std::string output_file_path = 
                            prob_json_file_.substr(0, prob_json_file_.rfind("/"));
                        output_file_path += "/optimal_actions.json";
                        //
                        planner.echo_shortest_time_path(output_file_path);
                    }
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
        //
        return 0;
    }
}