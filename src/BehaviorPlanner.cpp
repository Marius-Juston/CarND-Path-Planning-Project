//
// Created by mariu on 6/12/2020.
//

#include "BehaviorPlanner.h"
BehaviorPlanner::BehaviorPlanner(double look_ahead, double max_speed) {
  lookAhead = look_ahead;
  maxSpeed = max_speed;
}

NewPosition BehaviorPlanner::chooseNextStates(int currentLane,
                                              double currentS,
                                              double carSpeed,
                                              const vector<vector<double>> &vehicles) {
  lane = currentLane;

  double vehicle_s;

  vector<vector<double>> important_vehicles;

  for (const auto &vehicle : vehicles) {
    vehicle_s = vehicle[5];
    if (vehicle_s > currentS && vehicle_s < currentS + lookAhead) {
      important_vehicles.push_back(vehicle);
    }
  }

  vector<double> speeds = laneSpeeds(important_vehicles);
  vector<double> speedCosts = inefficiencyCost(speeds);
  vector<double> laneChangeCosts = laneChangeCost(currentLane);
  vector<double> impossibleLaneCosts = impossibleLaneCost(currentS, currentLane, carSpeed, vehicles);

  vector<double> costs = calculateCosts({laneChangeCosts, speedCosts, impossibleLaneCosts});

  if (impossibleLaneCosts[0] + impossibleLaneCosts[1] + impossibleLaneCosts[2] == 3.) {
    NewPosition position{};
    position.lane = currentLane;

    return position;
  }

  auto min_lane = std::min_element(costs.begin(), costs.end()) - costs.begin();

  for (const auto cost: costs) {
    cout << cost << ' ';
  }
  cout << '\t' << min_lane;
  cout << endl;

  NewPosition position{};
  position.lane = min_lane;

  return position;
}

vector<double> BehaviorPlanner::laneSpeeds(vector<vector<double>> &vehicles) {
  double vehicle_d;
  double vehicle_speed;
  int lane;

  vector<double> speed = {maxSpeed, maxSpeed, maxSpeed};

  for (const auto &vehicle : vehicles) {
    vehicle_d = vehicle[6];
    vehicle_speed = sqrt((vehicle[3] * vehicle[3]) + (vehicle[4] * vehicle[4]));
    lane = floor(vehicle_d / 4);

    if (speed[lane] > vehicle_speed) {
      speed[lane] = vehicle_speed * 2.2369362920544;
    }
  }

  return speed;
}

vector<double> BehaviorPlanner::inefficiencyCost(const vector<double> &laneSpeeds) {
  vector<double> costs;

  costs.reserve(laneSpeeds.size());
  for (double laneSpeed : laneSpeeds) {
    costs.push_back((maxSpeed - laneSpeed) / maxSpeed);
  }

  return costs;
}
vector<double> BehaviorPlanner::laneChangeCost(int currentLane) {
  vector<double> costs(3);

  for (int lane = 0; lane < 3; ++lane) {
    costs[lane] = pow(abs(lane - currentLane) / 2., 2);
  }

  return costs;
}
vector<double> BehaviorPlanner::calculateCosts(vector<vector<double>> costs) {
  vector<double> costsC(3, 0.);

  for (int i = 0; i < WEIGHTS.size(); ++i) {
    for (int y = 0; y < 3; ++y) {
      costsC[y] += costs[i][y] * WEIGHTS[i];
    }
  }

  return costsC;
}
vector<double> BehaviorPlanner::impossibleLaneCost(double currentS,
                                                   int currentLane, double carSpeed,
                                                   const vector<vector<double>> &vehicles) {
  int lane;
  double s;
  double vehicle_speed;
  double dt = .02;

  vector<double> cost(3, 0);
  double diff;
  double nextCarPos = (currentS + carSpeed * dt);

  for (const auto &vehicle : vehicles) {
    lane = floor(vehicle[6] / 4);
    s = vehicle[5];
    vehicle_speed = sqrt((vehicle[3] * vehicle[3]) + (vehicle[4] * vehicle[4])) * 2.2369362920544;
    diff = (s + vehicle_speed * dt) - nextCarPos;

//    cout << diff << endl;

    if (diff > -5 && diff < 25) {
      cost[lane] = 1.;
    }
  }

  if (cost[1] == 1.) {
    if (currentLane == 0 || currentLane == 2)
      cost[2 - currentLane] = 1;
  }

  return cost;
}
