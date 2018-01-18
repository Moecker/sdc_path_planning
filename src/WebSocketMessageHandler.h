//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#ifndef PF_WEBSOCKETMESSAGEHANDLER_H
#define PF_WEBSOCKETMESSAGEHANDLER_H

#include <string>

#include <uWS/uWS.h>
#include "json.hpp"

#include "PathPlanner.h"

using std::string;
using json = nlohmann::json;

class WebSocketMessageHandler
{
  public:
    WebSocketMessageHandler(PathPlanner& path_planner) : path_planner_(path_planner) {}
    void HandleMessage(const string& message, uWS::WebSocket<uWS::SERVER>& ws);

  private:
    bool MessageHasExpectedPrefix(const string& message);
    string GetMessageContent(const string& message);
    string ProcessMessageContent(string& content);
    string CreateResponseMessage(const std::vector<CartesianPoint>& path);

    PathPlannerInput ReadPlannerInput(json data);

    void SendDefaultResponse(uWS::WebSocket<uWS::SERVER>& ws) const;

	PathPlanner& path_planner_;
};

#endif  // PF_WEBSOCKETMESSAGEHANDLER_H
