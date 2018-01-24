#include "LaneChangePlanner.h"
#include "HighwayMap.h"

#include <algorithm>
#include <iostream>

using std::cout;
using std::endl;

void ChangeLaneCheck(int current_Lane,
                     double our_speed,
                     vector<vector<vector<double>>>& lane_lines,
                     vector<vector<int>>& close_cars,
                     vector<int>& lane_ranks)
{
    // @todo Check these variable
    double distance_increment{0.0};
    int lane_change_votes{0};
    double velocity_action{0.0};
    bool lane_change{false};
    double next_d_val{0.0};

    // This will be the number of our destination lane
    int destination_Lane = current_Lane;

    // Go through the three lanes
    for (int i = 0; i < 3; i++)
    {
        // Get lane number
        int lane_number = lane_ranks[i];

        // If best lane is current lane, we are done
        if (lane_number == current_Lane)
        {
            lane_change_votes = 0;
            break;
        }

        // Find out how mane lane shifts we need to reach the new lane and the direction
        int required_changed = lane_number - current_Lane;
        int direction = required_changed / abs(required_changed);

        // Flag to check if its feasible to change lanes
        bool change_feasible = true;

        // If we are too fast, multiple lane change causes too much jerk
        if ((our_speed >= 40.0) && (abs(required_changed) > 1))
            change_feasible = false;

        // Check if there are no cars directly in front / behind us
        for (int j = 1; j <= abs(required_changed); j++)
        {
            // Check if we can change lane
            int temp_Lane = abs(current_Lane + (j * direction));
            int carId_back = close_cars[temp_Lane][0];
            int carId_front = close_cars[temp_Lane][1];

            // Check for cars behind us
            if (carId_back != -1)
            {
                // Calculate the cars distance and velocity
                double distance = abs(lane_lines[temp_Lane][carId_back][8]);
                double velocity = lane_lines[temp_Lane][carId_back][7];

                // If we are faster, we can have distance of 15, otherwise we need 30
                if (!(((velocity < distance_increment) && (distance > 15.0)) ||
                      ((velocity > distance_increment) && (distance > 30.0))))
                {
                    change_feasible = false;
                    break;
                }
            }

            // Check for cars in front of us
            if (carId_front != -1)
            {
                // Calculate the cars distance and velocity
                double distance = abs(lane_lines[temp_Lane][carId_front][8]);
                double velocity = lane_lines[temp_Lane][carId_back][7];

                // If we are faster, we can have distance of 15, otherwise we need 30
                if (!(((velocity > distance_increment) && (distance > 15.0)) ||
                      ((velocity < distance_increment) && (distance > 30.0))))
                {
                    change_feasible = false;
                    break;
                }
            }
        }

        // Check if all lanes are fine for a change
        if (change_feasible)
        {
            // Increase number of votes to change
            lane_change_votes++;

            // If count of 20 votes is reached, do a lane change
            if (lane_change_votes > 20)
            {
                // Activate lane change
                lane_change = true;

                // Set destination
                destination_Lane = lane_number;

                // Reset votes
                lane_change_votes = 0;
                std::string lane_name;

                // Do some outprint where we go
                if (lane_number == 0)
                    lane_name = "Left";
                else if (lane_number == 1)
                    lane_name = "Middle";
                else
                    lane_name = "Right";
                cout << lane_name << " lane is faster, changing.." << endl;
            }
            break;
        }
    }

    // Update d value
    next_d_val = (destination_Lane * 4) + 2;

    // Update s value
    int car_id = close_cars[destination_Lane][1];

    // Full speed, because there is no car ahead
    if (car_id == -1)
    {
        distance_increment = 0.44;
        if (velocity_action != 0)
            cout << "Nothing in front, full speed.." << endl;
        velocity_action = 0;
    }

    // There is someone in front
    else
    {
        // Calculate distance and velocity of other car
        double distance = lane_lines[destination_Lane][car_id][8];
        double velocity = lane_lines[destination_Lane][car_id][7];

        // Distance is big enough, full speed
        if (distance > 40.0)
        {
            distance_increment = 0.44;
            if (velocity_action != 1)
                cout << "Cars far away, full speed.." << endl;
            velocity_action = 1;
        }

        // Try adapt to the speed of the car in front
        if ((distance <= 40.0) && (distance > 15.0))
        {
            if (distance_increment > velocity)
                distance_increment *= 0.7;
            else if (distance > 25.0)
                distance_increment = velocity + 0.02;
            else
                distance_increment = velocity - 0.02;
            if (velocity_action != 2)
                cout << "Adapt to front vehicle.." << endl;
            velocity_action = 2;
        }

        // If distance is too low, do brake stronger
        if ((distance <= 15.0) && (distance > 10.0))
        {
            distance_increment *= 0.7;
            if (velocity_action != 3)
                cout << "Slowing down faster.." << endl;
            velocity_action = 3;
        }
        if ((distance <= 10.0) && (distance > 5.0))
        {
            distance_increment *= 0.5;
            if (velocity_action != 4)
                cout << "Braking hard.." << endl;
            velocity_action = 4;
        }
        if (distance <= 5.0)
        {
            distance_increment *= 0.3;
            if (velocity_action != 5)
                cout << "Emergency brake.." << endl;
            velocity_action = 5;
        }
    }
}

// Identify the closest car ahead and behind of our car
void IdentClosestCars(vector<vector<vector<double>>>& lane_lines, vector<vector<int>>& close_cars)
{
    // Resize for our three lane lines
    close_cars.resize(3);

    // For each lane line
    for (int i = 0; i < 3; i++)
    {
        close_cars[i].push_back(-1);
        close_cars[i].push_back(-1);

        // Find closest car behind by maximum negative value
        for (int j = (static_cast<int>(lane_lines[i].size()) - 1); j >= 0; j--)
        {
            if (lane_lines[i][j][8] < 0)
            {
                close_cars[i][0] = j;
                break;
            }
        }

        // Find closest car ahead by minimum positive value
        for (int j = 0; j < lane_lines[i].size(); j++)
        {
            if (lane_lines[i][j][8] > 0)
            {
                close_cars[i][1] = j;
                break;
            }
        }
    }
}

// Give each lane a rank based on the score metrics / cost functions
void LaneRanking(int current_Lane,
                 vector<vector<vector<double>>>& lane_lines,
                 vector<vector<int>>& close_cars,
                 vector<int>& lane_ranks)
{
    // Store the scores for our lanes
    vector<std::pair<double, int>> lane_scores;

    // Find the closest cars over all lines
    IdentClosestCars(lane_lines, close_cars);

    // Resize for three lane lines
    lane_ranks.resize(3);

    // Compute lane scores
    for (int i = 0; i < 3; i++)
    {
        // Score metrics
        double distance_score, velocity_score, lane_change_score;

        // Calculate lane change score, we prefer small changes
        lane_change_score = 1.0 * (1.0 - (fabs(i - current_Lane) / 2.0));

        // Calculate distance to ahead car score
        if (close_cars[i][1] == -1)
            distance_score = 3.0;
        else
            distance_score = 3.0 * (1.0 - ((100.0 - lane_lines[i][close_cars[i][1]][8]) / 100.0));

        // Calculate velocity cost score
        if (close_cars[i][1] == -1)
            velocity_score = 2.0;
        else
            velocity_score = 2.0 * (1.0 - ((0.88 - lane_lines[i][close_cars[i][1]][7]) / 0.88));

        // Summ all scores together and save them
        lane_scores.push_back(std::make_pair((lane_change_score + distance_score + velocity_score), i));
    }

    // Sort the lane scores
    std::sort(lane_scores.begin(), lane_scores.end());

    // Finally, get the ranks
    for (int i = 0; i < 3; i++)
        lane_ranks[i] = lane_scores[2 - i].second;
}

// This is the behavior planner
void BehaviorPlanner(int current_lane, PathPlannerInput input)
{
    // Store information about our three lane lines
    vector<vector<vector<double>>> lane_lines(3);

    // Assign vehicles from sensor fusion to the corresponding lane
    for (int i = 0; i < input.other_cars.size(); i++)
    {
        // Get vehicle object from sensor fusion
        // @todo
        // Get this right vector<double> vehicle = sensor_fusion[i];

        vector<double> vehicle;
        vector<vector<double>> sensor_fusion;
        input.other_cars[0].frenet_location.s;

        // Add distance increment (velocity) of the car
        sensor_fusion[i].push_back(
            HighwayMap::EuclidDistance(CartesianPoint(0.0, 0.0), CartesianPoint(vehicle[3], vehicle[4] * 0.02)));

        // Add displacement from other car to our car
        sensor_fusion[i].push_back(vehicle[5] - input.frenet_location.s);

        // Add cars to the coreesponding lane
        for (int j = 0; j < 3; j++)
        {
            if ((vehicle[6] >= ((j * 4) - 0.3)) && (vehicle[6] <= (((j + 1) * 4) + 0.3)))
                lane_lines[j].push_back(sensor_fusion[i]);
        }
    }

    // Sort lanes based on distance
    for (int i = 0; i < 3; i++)
    {
        sort(lane_lines[i].begin(), lane_lines[i].end(), [](vector<double>& a, vector<double>& b) {
            return a[8] < b[8];
        });
    }

    // Rank the lanes based on the close cars
    vector<int> lane_ranks;
    vector<vector<int>> close_cars;
    LaneRanking(current_lane, lane_lines, close_cars, lane_ranks);

    // If there is a better lane, change to it
    ChangeLaneCheck(current_lane, input.speed, lane_lines, close_cars, lane_ranks);
}
