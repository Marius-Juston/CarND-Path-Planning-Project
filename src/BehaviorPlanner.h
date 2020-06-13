//
// Created by mariu on 6/12/2020.
//

#ifndef PATH_PLANNING_SRC_BEHAVIORPLANNER_H_
#define PATH_PLANNING_SRC_BEHAVIORPLANNER_H_

#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

using std::vector;
using std::cout;
using std::endl;
using std::string;

struct NewPosition {
  int lane;
  double s;
};

class BehaviorPlanner {
 public:
  explicit BehaviorPlanner(double lookAhead, double max_speed);
  NewPosition chooseNextStates(int currentLane, double currentS, const vector<vector<double>> &vehicles);
 private:
  double lookAhead;
  double maxSpeed;
  vector<double> laneSpeeds(vector<vector<double>> &vehicles);
  vector<double> inefficiencyCost(const vector<double> &laneSpeeds);
  vector<double> laneChangeCost(int currentLane);
  vector<double> calculateCosts(vector<vector<double>> costs);

  string state;
  int lane = 1;
//  const vector<double> WEIGHTS = {pow(10, 6), pow(10, 5)};
  const vector<double> WEIGHTS = {.5, 1.5, 2};
  vector<double> impossibleLaneCost(double currentS,
                                    const vector<vector<double>> &vehicles,
                                    const vector<double> &laneSpeeds);
};

#endif //PATH_PLANNING_SRC_BEHAVIORPLANNER_H_
