//
// Framework Created by Stanislav Olekhnovich on 02/08/2017.
// Customized by Anthony Knight on 17/01/2018
//

#include "WebSocketMessageHandler.h"

string WebSocketMessageHandler::CreateResponseMessage(const std::vector<CartesianPoint>& path)
{
    json msgJson;

    std::vector<double> pathX;
    std::vector<double> pathY;

	//std::cout << "sending co-ordinates:" << std::endl;
    for (auto& p: path)
    {
        pathX.push_back(p.X);
        pathY.push_back(p.Y);
		//std::cout << p.X << ", " << p.Y << std::endl;
    }

    msgJson["next_x"] = pathX;
    msgJson["next_y"] = pathY;

    auto msg = "42[\"control\"," + msgJson.dump() + "]";
    return msg;
}

string WebSocketMessageHandler::ProcessMessageContent(string& content)
{
    auto jsonContent = json::parse(content);
    string eventType = jsonContent[0].get<string>();

//	std::cout << jsonContent << std::endl;

    string response;
    if (eventType == "telemetry")
    {
        auto data = jsonContent[1];
        auto pathPlannerInput = ReadPlannerInput(data);
        auto output = pathPlanner.GeneratePath(pathPlannerInput);
        response = CreateResponseMessage(output);
    }
    return response;
}

PathPlannerInput WebSocketMessageHandler::ReadPlannerInput(json data)
{
    PathPlannerInput pathPlannerInput;

	//convert to radians and move to between PI and -PI range
	double heading = data["yaw"].get<double>() * M_PI / 180.0;

	if (heading > M_PI) {
		while (heading > M_PI) {
			heading -= (2 * M_PI);
		}
	}
	if (heading < -M_PI) {
		while (heading < -M_PI) {
			heading += (2 * M_PI);
		}
	}
    pathPlannerInput.LocationCartesian= { data["x"], data["y"], heading};
    pathPlannerInput.LocationFrenet= { data["s"], data["d"] };
    pathPlannerInput.SpeedMpH = data["speed"];
	pathPlannerInput.SpeedMpS = pathPlannerInput.SpeedMpH * (1609.34 / 3600.0);
    pathPlannerInput.PreviousPathX = data["previous_path_x"].get<std::vector<double>>();
    pathPlannerInput.PreviousPathY = data["previous_path_y"].get<std::vector<double>>();

	//std::cout << "receiving Path:" << std::endl;

    assert(pathPlannerInput.PreviousPathX.size() == pathPlannerInput.PreviousPathY.size());
	
	double Theta;
    for (int i = 0; i < pathPlannerInput.PreviousPathX.size(); i++)
    {
		if (i == 0) {
			Theta = pathPlannerInput.LocationCartesian.ThetaRads;
		}
		else {
			Theta = atan2(pathPlannerInput.PreviousPathY.at(i) - pathPlannerInput.PreviousPathY.at(i - 1), pathPlannerInput.PreviousPathX.at(i) - pathPlannerInput.PreviousPathX.at(i - 1));
		}
        pathPlannerInput.Path.emplace_back(pathPlannerInput.PreviousPathX.at(i), pathPlannerInput.PreviousPathY.at(i), Theta);
//		std::cout << pathPlannerInput.Path.back().X << ", " << pathPlannerInput.Path.back().Y << ", " << pathPlannerInput.Path.back().ThetaRads << std::endl;
    }

    pathPlannerInput.PathEndpointFrenet = { data["end_path_s"], data["end_path_d"] };
    auto sensorFusionData = data["sensor_fusion"].get<std::vector<std::vector<double>>>();
    for (auto& otherCarData : sensorFusionData)
    {
        OtherCar otherCar;
		otherCar.id = otherCarData[0];
		otherCar.LocationCartesian = { otherCarData[1], otherCarData[2] };
        otherCar.XAxisSpeed = otherCarData[3];
        otherCar.YAxisSpeed = otherCarData[4];
        otherCar.LocationFrenet = { otherCarData[5], otherCarData[6] };

        pathPlannerInput.OtherCars.push_back(otherCar);
    }

    return pathPlannerInput;
}

string WebSocketMessageHandler::GetMessageContent(const string& message)
{
    //string content;
	string content = "";

    bool hasNullContent = (message.find("null") != string::npos);
    if (hasNullContent)
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
    const string prefix {"42"};
	//std::cout << "MessageHasExpectedPrefix: Message is \n" << message << "\n----" << std::endl;
    return (message.substr(0, prefix.size()) == prefix);
}

void WebSocketMessageHandler::HandleMessage(const string& message, uWS::WebSocket<uWS::SERVER>& ws)
{
	if (!MessageHasExpectedPrefix(message))
	//	std::cout << "Received Message with unexpected prefix in \n" << message << "\n - discarding." << std::endl;
       return;

    auto content = GetMessageContent(message);
    if (content.empty())
    {
		// empty telemetry events are received when simulator issues a warning - like exceeding Jerk 
		// if there is no reply then will stop system from proceeding.
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
    //std::cout << response << std::endl;
	//std::cout << "..";
    ws.send(response.data(), response.length(), uWS::TEXT);
}


