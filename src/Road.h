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
    Road(vector<double> lane_speeds);
    virtual ~Road();

    void Advance();
    void AddEgo(int lane_num, double s, vector<double> config_data);

    void UpdateTraffic();

    Vehicle GetEgo();

    void PopulateTraffic();
    void Display(int timestep);
    double update_width_ = 70.0;

  private:
    string ego_rep_ = " *** ";
    int ego_key_ = -1;
    size_t num_lanes_;

    vector<double> lane_speeds_;
    double density_ = 0.15;
    double camera_center_;

    // Maps a vehicle id to a actual vehicle including ourself
    map<int, Vehicle> vehicles_;
    int vehicles_added_ = 0;
};
