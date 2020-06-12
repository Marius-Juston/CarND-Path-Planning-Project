//
// Created by mariu on 6/12/2020.
//

#ifndef PATH_PLANNING_SRC_BEHAVIORPLANNER_H_
#define PATH_PLANNING_SRC_BEHAVIORPLANNER_H_

#include <string>
#include <vector>
#include <cmath>
using std::vector;

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
  vector<double> inefficiencyCost(const vector<double>& laneSpeeds);

  string state;
  int lane = 1;
};

#endif //PATH_PLANNING_SRC_BEHAVIORPLANNER_H_