#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <chrono>
#include "RRT.hpp"

using namespace std;
using namespace Eigen;
using namespace chrono;

int main() {

  auto start = high_resolution_clock::now();
  //Amatrix
  RRT Test({0,13, -1,1 , -3,3 , -1,1 , -3,3 , -1,1},{1,1,1});
  //Quadcopter1 start location;
  Test.setStart({0,0,0,0,0,0});
  //Quadcopter2 start location
  Test.setStart({0,0,1,0,0,0});
  //Quadcopter1 Goals
  Test.setGoals({7,0,0,0,0,0},{12,0,0,0,0,0});
  //Quadcopter2 Goals
  Test.setGoals({7,1,0,0,0,0},{12,0,0,0,0,0});
  Test.Quad({0.2,0.2,0.2});
  Test.Obstacle({5,0,0},{1,10,1});
  Test.Obstacle({9,0,0},{1,10,1});
  //Boundaries
  Test.Obstacle({0,3,0},{20,1,20});
  Test.Obstacle({0,-3,0},{20,1,20});
  Test.Obstacle({14,0,0},{1,20,20});
  Test.Obstacle({-1.5,0,0},{1,20,20});
  Test.Obstacle({0,0,3},{20,20,1});
  Test.Obstacle({0,0,-3},{20,20,1});
  Test.findPath(15000,0.1,1);
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop-start);
  cout << "Run time for simulation: " << duration.count()*0.000001 <<" seconds" << endl;



};
