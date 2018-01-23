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
    double kUpdateWidth = 70.0;

    Road(double traffic_density, vector<int> lane_speeds);
    virtual ~Road();

    Vehicle get_ego();
    void populate_traffic();
    void advance();
    void display(int timestep);

    void add_ego(int lane_num, int s, vector<int> config_data);

  private:
    string ego_rep_ = " *** ";
    int ego_key_ = -1;
    size_t num_lanes_;

    vector<int> lane_speeds_;
    double density_;
    double camera_center_;

    map<int, Vehicle> vehicles_;
    int vehicles_added_ = 0;
};
