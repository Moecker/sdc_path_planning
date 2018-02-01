///
/// @file
///

#ifndef PF_WEBSOCKETMESSAGEHANDLER_H
#define PF_WEBSOCKETMESSAGEHANDLER_H

#include <uWS/uWS.h>
#include <3rdparty/json.hpp>
#include <string>

#include "PathPlanner.h"

using std::string;
using json = nlohmann::json;

class WebSocketMessageHandler
{
  public:
    WebSocketMessageHandler(PathPlanner& path_planner) : path_planner_(path_planner) {}
    void HandleMessage(const string& message, uWS::WebSocket<uWS::SERVER>& ws);

    static PathPlannerInput ReadPlannerInput(json data);
    static string CreateResponseMessage(const std::vector<CartesianPoint>& path);

  private:
    bool MessageHasExpectedPrefix(const string& message);
    string GetMessageContent(const string& message);
    string ProcessMessageContent(string& content);

    void SendDefaultResponse(uWS::WebSocket<uWS::SERVER>& ws) const;

    PathPlanner& path_planner_;
};

#endif  // PF_WEBSOCKETMESSAGEHANDLER_H
