#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include "RRT.hpp"

using namespace std;
using namespace Eigen;
using namespace chrono;

int main() {

  //Workspace 1
  //Initialzing the bounds of the states {x,xdot,y,ydot,z,zdot},{xdot,ydot,zdot}
  RRT Test({0,13, -1,1 , -2.5,2.5 , -1,1 , -2,2 , -1,1},{1,1,1});
  //Giving the dimensions of the quadcopter {Length,width,height}
  Test.Quad({0.2,0.2,0.2});
  //Quadcopter1 start location in terms of {x,xdot,y,ydot,z,zdot}
  Test.setStart({0,0,0,0,0,0});
  //Quadcopter2 start location
  Test.setStart({0,0,1,0,0,0});
  //Quadcopter 3 start location
  Test.setStart({0,0,-1,0,0,0});
  //Quadcopter1 Goals (intermediate goal,final goal)
  Test.setGoals({7,0,0,0,0,0},{12,0,0,0,0,0});
  //Quadcopter2 Goals
  Test.setGoals({7,0,2,0,0,0},{12,0,0,0,0,0});
  //QuadCopter3 goals
  Test.setGoals({7,0,-2,0,0,0},{12,0,0,0,0,0});
  //Obstacles (center(x,y,z),dimensions(length,width,height))
  Test.Obstacle({5,0,0},{1,10,1});
  Test.Obstacle({9,0,0},{1,10,1});

  //Boundaries constraining the map to a certain area
  Test.Obstacle({0,5,0},{20,1,20});
  Test.Obstacle({0,-5,0},{20,1,20});
  Test.Obstacle({14,0,0},{1,20,20});
  Test.Obstacle({0,0,3},{20,20,1});
  Test.Obstacle({0,0,-3},{20,20,1});

  //finding the solution takes in (Number of nodes, Goal bias, Epsilon, Quad 1
  // Path file name, Quad 2 Path file name, Quad 3 Path filename)
  Test.findPath(15000,0.1,1,"Quad1Path.txt","Quad2Path.txt","Quad3Path.txt");


  //Workspace 2
  RRT Test2({0,6, -1,1 , -6,6 , -1,1 , -2,2 , -1,1},{1,1,1});
  Test2.Quad({0.2,0.2,0.2});
  //Quadcopter1 start location and goals;
  Test2.setStart({1,0,0,0,0,0});
  Test2.setGoals({6,0,0,0,0,0},{0,0,0,0,0,0});
  //Quadcopter2 start location and goals
  Test2.setStart({0,0,1,0,0,0});
  Test2.setGoals({0,0,6,0,0,0},{0,0,0,0,0,0});
  //Quadcopter3 start location and goals
  Test2.setStart({0,0,-1,0,0,0});
  Test2.setGoals({0,0,-6,0,0,0},{0,0,0,0,0,0});

  //Obstacles
  Test2.Obstacle({4.5,-2,0},{6,1,10});
  Test2.Obstacle({4.5,2,0},{6,1,10});
  Test2.Obstacle({2,4.5,0},{1,6,10});
  Test2.Obstacle({2,-4.5,0},{1,6,10});


  //Boundaries
  Test2.Obstacle({0,7.5,0},{20,1,20});
  Test2.Obstacle({0,-7.5,0},{20,1,20});
  Test2.Obstacle({7.5,0,0},{1,20,20});
  Test2.Obstacle({-1,0,0},{1,20,20});
  Test2.Obstacle({0,0,5},{20,20,1});
  Test2.Obstacle({0,0,-1.5},{20,20,1});

  //Finding a high_resolution_clock
  Test2.findPath(15000,0.1,1.5,"QuadPath1W2.txt","QuadPath2W2.txt","QuadPath3W2.txt");

  //Workspace 3
  RRT Test3({0,9, -1,1 , -3,3 , -1,1 , -2,2 , -1,1},{1,1,1});
  Test3.Quad({0.2,0.2,0.2});
  //Quadcopter1 start location and goals;
  Test3.setStart({0,0,0,0,0,0});
  Test3.setGoals({9,0,0,0,0,0},{0,0,0,0,0,0});
  //Quadcopter2 start location and goals
  Test3.setStart({0,0,0.75,0,0,0});
  Test3.setGoals({9,0,0,0,0,0},{0,0,0,0,0,0});
  //Quadcopter3 start location and goals
  Test3.setStart({0,0,-0.75,0,0,0});
  Test3.setGoals({9,0,0,0,0,0},{0,0,0,0,0,0});

  //Obstacles
  Test3.Obstacle({6.5,-1.75,0},{10,1,10});
  Test3.Obstacle({6.5,1.75,0},{10,1,10});

  //Boundaries
  Test3.Obstacle({12,0,0},{1,20,20});
  Test3.Obstacle({-1,0,0},{1,20,20});
  Test3.Obstacle({0,0,5},{20,20,1});
  Test3.Obstacle({0,0,-1.5},{20,20,1});

  //Finding the solution
  Test3.findPath(15000,0.1,1,"QuadPath1W3.txt","QuadPath2W3.txt","QuadPath3W3.txt");

////////////////////////////////////////////////////////////////////////////////
// The code below was for the benchmarking for each map


  //Workspace 1 testing
  //Amatrix
  //double numberofIt = 0;
  //vector<double>* lengths = new vector<double>;
  //double Success1 =0;
  //double Success2 =0;
  //double Success3 =0;
  /*RRTstar Test({0,13, -1,1 , -2.5,2.5 , -1,1 , -2,2 , -1,1},{1,1,1});
  Test.Quad({0.2,0.2,0.2});
  //Quadcopter1 start location;
  Test.setStart({0,0,0,0,0,0});
  //Quadcopter2 start location
  Test.setStart({0,0,1,0,0,0});

  Test.setStart({0,0,-1,0,0,0});
  //Quadcopter1 Goals
  Test.setGoals({7,0,0,0,0,0},{12,0,0,0,0,0});
  //Quadcopter2 Goals
  Test.setGoals({7,0,2,0,0,0},{12,0,0,0,0,0});
  //QuadCopter3 goals
  Test.setGoals({7,0,-2,0,0,0},{12,0,0,0,0,0});
  //Obstacles
  Test.Obstacle({5,0,0},{1,10,1});
  Test.Obstacle({9,0,0},{1,10,1});
  //Test.Obstacle({5,0,2},{1,10,1});
  //Test.Obstacle({5,0,-2},{1,10,1});

  //Boundaries
  Test.Obstacle({0,5,0},{20,1,20});
  Test.Obstacle({0,-5,0},{20,1,20});
  Test.Obstacle({14,0,0},{1,20,20});
  Test.Obstacle({0,0,3},{20,20,1});
  Test.Obstacle({0,0,-3},{20,20,1});


  ofstream file;
  file.open("Misc.txt",ios_base::app);
  for (int i = 0; i < 1 ;i++) {
    Test.clear();
    Test.SuccessInterm1 = 0;
    Test.SuccessInterm2 = 0;
    Test.SuccessInterm3 = 0;
    Test.SuccessFinal1 = 0;
    Test.SuccessFinal2 = 0;
    Test.SuccessFinal3 = 0;
    auto start = high_resolution_clock::now();
    *(lengths) = Test.findPath(15000,0.1,1,"Path1.txt","Path2.txt","Path3.txt");
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop-start);
    cout << "Run time for simulation: " << duration.count()*0.000001 <<" seconds" << endl;
    numberofIt++;
    cout << "Iteration: " << numberofIt << endl;

    if (Test.SuccessInterm1 && Test.SuccessFinal1) {
      Success1 = 1;
    } else if (Test.SuccessInterm1 || Test.SuccessFinal1) {
      Success1 = 0.5;
    } else {
      Success1 = 0;
    }

    if (Test.SuccessInterm2 && Test.SuccessFinal2) {
      Success2 = 1;
    } else if (Test.SuccessInterm2 || Test.SuccessFinal2) {
      Success2 = 0.5;
    } else {
      Success2 = 0;
    }

    if (Test.SuccessInterm3 && Test.SuccessFinal3) {
      Success3 = 1;
    } else if (Test.SuccessInterm3 || Test.SuccessFinal3) {
      Success3 = 0.5;
    } else {
      Success3 = 0;
    }
    cout << "Here" << endl;
    file << (*(lengths))[0] << " " << (*(lengths))[1] << " " << (*(lengths))[2] << " " <<
    " " << Success1 << " " << Success2 << " " << Success3 << " " << duration.count()*0.000001 << endl;
  }
  file.close();*/
  //Workspace 2 testing


  //Amatrix
  /*RRTstar Test2({0,6, -1,1 , -6,6 , -1,1 , -2,2 , -1,1},{1,1,1});
  Test2.Quad({0.2,0.2,0.2});
  //Quadcopter1 start location and goals;
  Test2.setStart({1,0,0,0,0,0});
  Test2.setGoals({6,0,0,0,0,0},{0,0,0,0,0,0});
  //Quadcopter2 start location and goals
  Test2.setStart({0,0,1,0,0,0});
  Test2.setGoals({0,0,6,0,0,0},{0,0,0,0,0,0});
  //Quadcopter3 start location and goals
  Test2.setStart({0,0,-1,0,0,0});
  Test2.setGoals({0,0,-6,0,0,0},{0,0,0,0,0,0});

  Test2.Obstacle({4.5,-2,0},{6,1,10});
  Test2.Obstacle({4.5,2,0},{6,1,10});
  Test2.Obstacle({2,4.5,0},{1,6,10});
  Test2.Obstacle({2,-4.5,0},{1,6,10});


  //Boundaries
  Test2.Obstacle({0,7.5,0},{20,1,20});
  Test2.Obstacle({0,-7.5,0},{20,1,20});
  Test2.Obstacle({7.5,0,0},{1,20,20});
  Test2.Obstacle({-1,0,0},{1,20,20});
  Test2.Obstacle({0,0,5},{20,20,1});
  Test2.Obstacle({0,0,-1.5},{20,20,1});
  ofstream file;
  file.open("Misc.txt",ios_base::app);
  for (int i = 0; i < 1 ;i++) {
    Test2.clear();
    Test2.SuccessInterm1 = 0;
    Test2.SuccessInterm2 = 0;
    Test2.SuccessInterm3 = 0;
    Test2.SuccessFinal1 = 0;
    Test2.SuccessFinal2 = 0;
    Test2.SuccessFinal3 = 0;
    auto start = high_resolution_clock::now();
    *(lengths) = Test2.findPath(15000,0.1,1.5,"Path1W2demo2.txt","Path2W2demo2.txt","Path3W2demo2.txt");
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop-start);
    cout << "Run time for simulation: " << duration.count()*0.000001 <<" seconds" << endl;
    numberofIt++;
    cout << "Iteration: " << numberofIt << endl;

    if (Test2.SuccessInterm1 && Test2.SuccessFinal1) {
      Success1 = 1;
    } else if (Test2.SuccessInterm1 || Test2.SuccessFinal1) {
      Success1 = 0.5;
    } else {
      Success1 = 0;
    }

    if (Test2.SuccessInterm2 && Test2.SuccessFinal2) {
      Success2 = 1;
    } else if (Test2.SuccessInterm2 || Test2.SuccessFinal2) {
      Success2 = 0.5;
    } else {
      Success2 = 0;
    }

    if (Test2.SuccessInterm3 && Test2.SuccessFinal3) {
      Success3 = 1;
    } else if (Test2.SuccessInterm3 || Test2.SuccessFinal3) {
      Success3 = 0.5;
    } else {
      Success3 = 0;
    }
    cout << "Here" << endl;
    file << (*(lengths))[0] << " " << (*(lengths))[1] << " " << (*(lengths))[2] << " " <<
    " " << Success1 << " " << Success2 << " " << Success3 << " " << duration.count()*0.000001 << endl;
  }
  file.close();*/

////////////////////////////////////////////////////////////////////////////////
/*RRTstar Test2({0,9, -1,1 , -3,3 , -1,1 , -2,2 , -1,1},{1,1,1});
Test2.Quad({0.2,0.2,0.2});
//Quadcopter1 start location and goals;
Test2.setStart({0,0,0,0,0,0});
Test2.setGoals({9,0,0,0,0,0},{0,0,0,0,0,0});
//Quadcopter2 start location and goals
Test2.setStart({0,0,0.75,0,0,0});
Test2.setGoals({9,0,0,0,0,0},{0,0,0,0,0,0});
//Quadcopter3 start location and goals
Test2.setStart({0,0,-0.75,0,0,0});
Test2.setGoals({9,0,0,0,0,0},{0,0,0,0,0,0});

Test2.Obstacle({6.5,-1.75,0},{10,1,10});
Test2.Obstacle({6.5,1.75,0},{10,1,10});

//Boundaries
Test2.Obstacle({12,0,0},{1,20,20});
Test2.Obstacle({-1,0,0},{1,20,20});
Test2.Obstacle({0,0,5},{20,20,1});
Test2.Obstacle({0,0,-1.5},{20,20,1});
ofstream file;
file.open("Misc.txt",ios_base::app);
for (int i = 0; i < 62 ;i++) {
  Test2.clear();
  Test2.SuccessInterm1 = 0;
  Test2.SuccessInterm2 = 0;
  Test2.SuccessInterm3 = 0;
  Test2.SuccessFinal1 = 0;
  Test2.SuccessFinal2 = 0;
  Test2.SuccessFinal3 = 0;
  auto start = high_resolution_clock::now();
  *(lengths) = Test2.findPath(15000,0.1,1,"Path1W3demo.txt","Path2W3demo.txt","Path3W3demo.txt");
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop-start);
  cout << "Run time for simulation: " << duration.count()*0.000001 <<" seconds" << endl;
  numberofIt++;
  cout << "Iteration: " << numberofIt << endl;

  if (Test2.SuccessInterm1 && Test2.SuccessFinal1) {
    Success1 = 1;
  } else if (Test2.SuccessInterm1 || Test2.SuccessFinal1) {
    Success1 = 0.5;
  } else {
    Success1 = 0;
  }

  if (Test2.SuccessInterm2 && Test2.SuccessFinal2) {
    Success2 = 1;
  } else if (Test2.SuccessInterm2 || Test2.SuccessFinal2) {
    Success2 = 0.5;
  } else {
    Success2 = 0;
  }

  if (Test2.SuccessInterm3 && Test2.SuccessFinal3) {
    Success3 = 1;
  } else if (Test2.SuccessInterm3 || Test2.SuccessFinal3) {
    Success3 = 0.5;
  } else {
    Success3 = 0;
  }
  cout << "Here" << endl;
  file << (*(lengths))[0] << " " << (*(lengths))[1] << " " << (*(lengths))[2] << " " <<
  " " << Success1 << " " << Success2 << " " << Success3 << " " << duration.count()*0.000001 << endl;
}
file.close();*/

};





//Obstacles (Pillars)
/*Test2.Obstacle({2,0,0},{0.5,0.5,10});
Test2.Obstacle({2,3,0},{0.5,0.5,10});
Test2.Obstacle({2,-3,0},{0.5,0.5,10});

Test2.Obstacle({4,1.5,0},{0.5,0.5,10});
Test2.Obstacle({4,-1.5,0},{0.5,0.5,10});

Test2.Obstacle({6,0,0},{0.5,0.5,10});
Test2.Obstacle({6,1.5,0},{0.5,0.5,10});
Test2.Obstacle({6,-1.5,0},{0.5,0.5,10});

Test2.Obstacle({8,1.5,0},{0.5,0.5,10});
Test2.Obstacle({8,-1.5,0},{0.5,0.5,10});

Test2.Obstacle({10,0,0},{0.5,0.5,10});
Test2.Obstacle({10,3,0},{0.5,0.5,10});
Test2.Obstacle({10,-3,0},{0.5,0.5,10});*/
