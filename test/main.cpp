///
/// @file
///

#include <3rdparty/json.hpp>
#include <fstream>

#include <PathPlanner.h>
#include <SimpleSplineBasedPlanner.h>
#include <WebSocketMessageHandler.h>

using json = nlohmann::json;

std::vector<json> ReadJsonFromFile(std::string file_name)
{
    std::vector<json> jsons;
    std::ifstream file(file_name);

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        json j = json::parse(line);
        jsons.push_back(j);
    }
    file.close();

    return jsons;
}

std::vector<std::string> ReadStringFromFile(std::string file_name)
{
    std::vector<std::string> jsons;
    std::ifstream file(file_name);

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        jsons.push_back(line);
    }
    file.close();

    return jsons;
}

int main(int argc, char* argv[])
{
    auto jsons_input = ReadJsonFromFile("../test-data/input.json");
    auto strings_response = ReadStringFromFile("../test-data/response.json");

    std::cout << jsons_input.size() << std::endl;
    std::cout << strings_response.size() << std::endl;

    assert(jsons_input.size() == strings_response.size());

    auto kStartingLane = 1;
    HighwayMap map("../data/highway_map.csv");
    SimpleSplineBasedPlanner path_planner(map, kStartingLane);

    bool result = true;
    for (int i = 0; i < jsons_input.size(); ++i)
    {
        auto input = jsons_input[i];
        std::string expected_response = strings_response[i];

        auto path_planner_input = WebSocketMessageHandler::ReadPlannerInput(input);
        auto output_path = path_planner.GeneratePath(path_planner_input);
        std::string actual_response = WebSocketMessageHandler::CreateResponseMessage(output_path);

        bool step_ok = (expected_response == actual_response);
        std::cout << "Step " << i << ": " << (step_ok ? "True" : "False") << std::endl;
        result = result && step_ok;

        if (!step_ok)
        {
            std::cout << "Failed with in/out: " << std::endl;
            std::cout << expected_response << std::endl;
            std::cout << actual_response << std::endl;

            std::cout << "Failed with sizes: " << std::endl;
            std::cout << expected_response.size() << std::endl;
            std::cout << actual_response.size() << std::endl;
            break;
        }
    }
    std::cout << "Overall: " << (result ? "True" : "False") << std::endl;

    return 0;
}
