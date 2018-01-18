///
/// @file
///

#include "WebSocketMessageHandler.h"

string WebSocketMessageHandler::CreateResponseMessage(const std::vector<CartesianPoint>& path)
{
    json json_msg;

    std::vector<double> path_x;
    std::vector<double> path_y;
    for (auto& p : path)
    {
        path_x.push_back(p.x);
        path_y.push_back(p.y);
    }

    json_msg["next_x"] = path_x;
    json_msg["next_y"] = path_y;

    auto msg = "42[\"control\"," + json_msg.dump() + "]";
    return msg;
}

string WebSocketMessageHandler::ProcessMessageContent(string& content)
{
    auto json_content = json::parse(content);
    string event_type = json_content[0].get<string>();

    string response;
    if (event_type == "telemetry")
    {
        auto data = json_content[1];
        auto path_planner_input = ReadPlannerInput(data);
        auto output = path_planner_.GeneratePath(path_planner_input);
        response = CreateResponseMessage(output);
    }
    return response;
}

PathPlannerInput WebSocketMessageHandler::ReadPlannerInput(json data)
{
    PathPlannerInput path_planner_input;

    path_planner_input.cartesian_location = {data["x"], data["y"], data["yaw"]};
    path_planner_input.fenet_location = {data["s"], data["d"]};
    path_planner_input.speed = data["speed"];
    path_planner_input.previous_path_x = data["previous_path_x"].get<std::vector<double>>();
    path_planner_input.previous_path_y = data["previous_path_y"].get<std::vector<double>>();

    assert(path_planner_input.previous_path_x.size() == path_planner_input.previous_path_y.size());
    for (int i = 0; i < path_planner_input.previous_path_x.size(); i++)
    {
        path_planner_input.path.emplace_back(path_planner_input.previous_path_x[i],
                                             path_planner_input.previous_path_y[i]);
    }

    path_planner_input.path_endpoint_frenet = {data["end_path_s"], data["end_path_d"]};
    auto sensor_fusion_data = data["sensor_fusion"].get<std::vector<std::vector<double>>>();
    for (auto& other_car_data : sensor_fusion_data)
    {
        OtherCar other_car;
        other_car.cartesian_location = {other_car_data[1], other_car_data[2]};
        other_car.x_axis_speed = other_car_data[3];
        other_car.y_axis_speed = other_car_data[4];
        other_car.frenet_location = {other_car_data[5], other_car_data[6]};

        path_planner_input.other_cars.push_back(other_car);
    }

    return path_planner_input;
}

string WebSocketMessageHandler::GetMessageContent(const string& message)
{
    string content;

    bool has_null_content = (message.find("null") != string::npos);
    if (has_null_content)
        return content;

    auto b1 = message.find_first_of('[');
    auto b2 = message.find_last_of(']');

    if (b1 != string::npos && b2 != string::npos)
        content = message.substr(b1, b2 - b1 + 1);

    return content;
}

bool WebSocketMessageHandler::MessageHasExpectedPrefix(const string& message)
{
    // "42" at the start of the message means there's a websocket message event.
    const string prefix{"42"};
    return (message.substr(0, prefix.size()) == prefix);
}

void WebSocketMessageHandler::HandleMessage(const string& message, uWS::WebSocket<uWS::SERVER>& ws)
{
    if (!MessageHasExpectedPrefix(message))
        return;

    auto content = GetMessageContent(message);
    if (content.empty())
    {
        SendDefaultResponse(ws);
        return;
    }

    auto response = ProcessMessageContent(content);

    if (!response.empty())
        ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
}

void WebSocketMessageHandler::SendDefaultResponse(uWS::WebSocket<uWS::SERVER>& ws) const
{
    string response = "42[\"manual\",{}]";
    std::cout << response << std::endl;
    ws.send(response.data(), response.length(), uWS::TEXT);
}
