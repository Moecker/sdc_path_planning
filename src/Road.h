#include <math.h>
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
    double update_width = 70.0;
    string ego_rep = " *** ";
    int ego_key = -1;
    size_t num_lanes;

    vector<int> lane_speeds;
    int speed_limit;
    double density;
    double camera_center;

    map<int, Vehicle> vehicles;
    int vehicles_added = 0;

    Road(int speed_limit, double traffic_density, vector<int> lane_speeds);
    virtual ~Road();

    Vehicle get_ego();

    void populate_traffic();
    void advance();
    void display(int timestep);
    void add_ego(int lane_num, int s, vector<int> config_data);

    void cull();
};
