#ifndef COST_H
#define COST_H

#include "Vehicle.h"

using namespace std;

double CalculateCost(const Vehicle& vehicle,
                     const map<int, vector<Vehicle>>& predictions,
                     const vector<Vehicle>& trajectory);

double GoalDistanceCost(const Vehicle& vehicle,
                        const vector<Vehicle>& trajectory,
                        const map<int, vector<Vehicle>>& predictions,
                        map<string, int>& data);

double InefficiencyCost(const Vehicle& vehicle,
                        const vector<Vehicle>& trajectory,
                        const map<int, vector<Vehicle>>& predictions,
                        map<string, int>& data);

double GetLaneSpeed(const map<int, vector<Vehicle>>& predictions, int lane);

map<string, int> GetTrajectoryMetaData(const Vehicle& vehicle,
                                       const vector<Vehicle>& trajectory,
                                       const map<int, vector<Vehicle>>& predictions);

#endif
