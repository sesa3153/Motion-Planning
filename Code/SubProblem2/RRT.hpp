#include<iostream>
#include<vector>
#include<string>
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;

struct obsInfo {

  vector<double> center;
  double length;
  double width;
  double height;

};

struct quadCopterDim {

  double length;
  double width;
  double height;

};

struct QuadVertices{

  vector<double> vert_x;
  vector<double> vert_y;
  vector<double> vert_z;

};

struct Node {

//States
double x;
double xdot;
double y;
double ydot;
double z;
double zdot;
double time;

//NodeInfo
Node* parent = NULL;
Node* child = NULL;


};


class RRT {

    public:
      RRT(vector<double> b,vector<double> vb);
      ~RRT();
      void setStart(vector<double> Qstart);
      void setGoals(vector<double> Gfirst, vector<double> Glast);
      vector<double> SampleState();
      double GenerateProbab();
      void findPath(int n,double pgoal,double eps);
      void PropagatingGoalInterm(int n, double pgoal,double eps);
      void PropagatingGoalFinal(int n, double pgoal, double eps);
      vector<double> Extend(vector<double>* RandPtr,int option,int QuadNum);
      bool CollisionChecker(double x, double y, double z);
      bool QuadCollisionChecker(double x, double y, double z, double x2,
      double y2,double z2);
      QuadVertices locate(double x, double y, double z);
      Node* closest(vector<double>* RandPtr,int option,int QuadNum);
      vector<double> fDynamics(Node* candidate,VectorXd Input);
      VectorXd SampleControlInput();
      double dist(vector<double> arr, vector<double> arr2);
      bool WithinConstraints(vector<double> arr);
      void Obstacle(vector<double> cent, vector<double> dim);
      void Quad(vector<double> dim);
    private:
      vector<vector<double>> StartLocation;
      vector<vector<double>> GoalIntermediate;
      vector<vector<double>> Goalfinal;
      vector<Node*> Vertices1Quad1;
      vector<Node*> Vertices2Quad1;
      vector<Node*> Vertices1Quad2;
      vector<Node*> Vertices2Quad2;
      vector<double> bounds;
      vector<obsInfo> obstacles;
      vector<double> velocityConst;
      quadCopterDim QuadCopter;
};
