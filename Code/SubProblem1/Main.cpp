#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "RRT.hpp"

using namespace std;
using namespace Eigen;

int main() {

  double numberofIt = 0;
  double length;

  //Amatrix
  RRT Test({0,13, -1,1 , -5,5 , -1,1 , -3,3 , -1,1},{1,1,1});
  Test.setStart({0,0,0,0,0,0});
  Test.setGoals({7,0,0,0,0,0},{12,0,0,0,0,0});
  Test.Quad({0.25,0.25,0.25});
  //Obstacles
  Test.Obstacle({5,0,0},{1,10,1});
  Test.Obstacle({9,0,0},{1,10,1});

  //Boundaries
  Test.Obstacle({0,5,0},{20,1,20});
  Test.Obstacle({0,-5,0},{20,1,20});
  Test.Obstacle({14,0,0},{1,20,20});
  Test.Obstacle({0,0,3},{20,20,1});
  Test.Obstacle({0,0,-3},{20,20,1});

  Test.findPath(15000,0.1,1);



};
