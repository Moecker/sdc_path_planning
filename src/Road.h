#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "Vehicle.h"

using namespace std;

class Road
{
  public:
    Road(double traffic_density, vector<int> lane_speeds);
    virtual ~Road();

    Vehicle GetEgo();
    void PopulateTraffic();
    void Advance();
    void Display(int timestep);
    void AddEgo(int lane_num, int s, vector<int> config_data);

    double update_width_ = 70.0;

  private:
    string ego_rep_ = " *** ";
    int ego_key_ = -1;
    size_t num_lanes_;

    vector<int> lane_speeds_;
    double density_;
    double camera_center_;

    // Maps a vehicle id to a actual vehicle including ourself
    map<int, Vehicle> vehicles_;
    int vehicles_added_ = 0;
};
