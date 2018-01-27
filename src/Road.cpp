#include "Road.h"
#include <algorithm>

Road::Road(double traffic_density, vector<int> lane_speeds)
{
    this->num_lanes_ = lane_speeds.size();
    this->lane_speeds_ = lane_speeds;
    this->density_ = traffic_density;
    this->camera_center_ = this->kUpdateWidth / 2.0;
}

Road::~Road()
{
}

Vehicle Road::get_ego()
{
    return this->vehicles_.find(this->ego_key_)->second;
}

void Road::populate_traffic()
{
    double start_s = std::max(this->camera_center_ - (this->kUpdateWidth / 2.0), 0.0);
    for (int l = 0; l < this->num_lanes_; l++)
    {
        int lane_speed = this->lane_speeds_[l];
        bool vehicle_just_added = false;
        for (double s = start_s; s < start_s + this->kUpdateWidth; s++)
        {
            if (vehicle_just_added)
            {
                vehicle_just_added = false;
            }
            if (((double)rand() / (RAND_MAX)) < this->density_)
            {

                Vehicle vehicle = Vehicle(l, s, lane_speed, 0);
                vehicle.state = "CS";
                this->vehicles_added_ += 1;
                this->vehicles_.insert(std::pair<int, Vehicle>(vehicles_added_, vehicle));
                vehicle_just_added = true;
            }
        }
    }
}

void Road::advance()
{
    map<int, vector<Vehicle> > predictions;

    map<int, Vehicle>::iterator it = this->vehicles_.begin();
    while (it != this->vehicles_.end())
    {
        int v_id = it->first;
        vector<Vehicle> preds = it->second.GeneratePredictions();
        predictions[v_id] = preds;
        it++;
    }
    it = this->vehicles_.begin();
    while (it != this->vehicles_.end())
    {
        int v_id = it->first;
        if (v_id == ego_key_)
        {
            vector<Vehicle> trajectory = it->second.ChooseNextState(predictions);
            it->second.RealizeNextState(trajectory);
        }
        else
        {
            it->second.Increment(1);
        }
        it++;
    }
}

void Road::add_ego(int lane_num, int s, vector<int> config_data)
{
    map<int, Vehicle>::iterator it = this->vehicles_.begin();
    while (it != this->vehicles_.end())
    {
        int v_id = it->first;
        Vehicle v = it->second;
        if (v.lane == lane_num && v.s == s)
        {
            this->vehicles_.erase(v_id);
        }
        it++;
    }
    Vehicle ego = Vehicle(lane_num, s, this->lane_speeds_[lane_num], 0);
    ego.Configure(config_data);
    ego.state = "KL";
    this->vehicles_.insert(std::pair<int, Vehicle>(ego_key_, ego));
}

void Road::display(int timestep)
{
    Vehicle ego = this->vehicles_.find(this->ego_key_)->second;
    double s = ego.s;
    string state = ego.state;

    this->camera_center_ = std::max(s, this->kUpdateWidth / 2.0);
    double s_min = std::max(this->camera_center_ - this->kUpdateWidth / 2.0, 0.0);
    double s_max = s_min + this->kUpdateWidth;

    vector<vector<string> > road;

    for (int i = 0; i < this->kUpdateWidth; i++)
    {
        vector<string> road_lane;
        for (int ln = 0; ln < this->num_lanes_; ln++)
        {
            road_lane.push_back("     ");
        }
        road.push_back(road_lane);
    }

    map<int, Vehicle>::iterator it = this->vehicles_.begin();
    while (it != this->vehicles_.end())
    {

        int v_id = it->first;
        Vehicle v = it->second;

        if (s_min <= v.s && v.s < s_max)
        {
            string marker = "";
            if (v_id == this->ego_key_)
            {
                marker = this->ego_rep_;
            }
            else
            {

                stringstream oss;
                stringstream buffer;
                buffer << " ";
                oss << v_id;
                for (size_t buffer_i = oss.str().length(); buffer_i < 3; buffer_i++)
                {
                    buffer << "0";
                }
                buffer << oss.str() << " ";
                marker = buffer.str();
            }
            road[int(v.s - s_min)][int(v.lane)] = marker;
        }
        it++;
    }
    ostringstream oss;
    oss << "+Meters ======================+ step: " << timestep << endl;
    int i = static_cast<int>(s_min);
    for (int lj = 0; lj < road.size(); lj++)
    {
        if (i % 20 == 0)
        {
            stringstream buffer;
            stringstream dis;
            dis << i;
            for (size_t buffer_i = dis.str().length(); buffer_i < 3; buffer_i++)
            {
                buffer << "0";
            }

            oss << buffer.str() << dis.str() << " - ";
        }
        else
        {
            oss << "      ";
        }
        i++;
        for (int li = 0; li < road[0].size(); li++)
        {
            oss << "|" << road[lj][li];
        }
        oss << "|";
        oss << "\n";
    }

    cout << oss.str();
}
