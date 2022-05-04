#include<iostream>
#include<vector>
#include<string>
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;

//Obstacle information (center/dimensions)
struct obsInfo {

  vector<double> center;
  double length;
  double width;
  double height;

};

//QuadCopter dimensions
struct quadCopterDim {

  double length;
  double width;
  double height;

};

//QuadCopter Vertices
struct QuadVertices{

  vector<double> vert_x;
  vector<double> vert_y;
  vector<double> vert_z;

};

//State Node
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

//Map Creater /Planner
class RRT {

    public:
      RRT(vector<double> b,vector<double> vb);
      ~RRT();
      void setStart(vector<double> Qstart);
      void setGoals(vector<double> Gfirst, vector<double> Glast);
      vector<double> SampleState(int option);
      double GenerateProbab();
      vector<double> findPath(int n,double pgoal,double eps,string name1,string name2,string name3);
      void PropagatingGoal(int n, double pgoal,double eps);
      vector<double> Extend(vector<double>* RandPtr,int option,int QuadNum);
      bool CollisionChecker(double x, double y, double z);
      bool Quad3Checker(Node* candidate,Node* tempA, Node* tempB);
      bool QuadCollisionChecker(double x, double y, double z, double x2,
      double y2,double z2);
      QuadVertices locate(double x, double y, double z);
      Node* closest(vector<double>* RandPtr,int option,int QuadNum);
      vector<double> fDynamics(Node* candidate,VectorXd Input);
      VectorXd SampleControlInput();
      double totalPathLength(Node* node);
      double dist(vector<double> arr, vector<double> arr2);
      double dist2(vector<double>arr, vector<double> arr2);
      bool WithinConstraints(vector<double> arr);
      void Obstacle(vector<double> cent, vector<double> dim);
      void Quad(vector<double> dim);
      void clear();
      double SuccessInterm1;
      double SuccessInterm2;
      double SuccessInterm3;
      double SuccessFinal1;
      double SuccessFinal2;
      double SuccessFinal3;
    private:
      vector<vector<double>> StartLocation;
      vector<vector<double>> GoalIntermediate;
      vector<vector<double>> Goalfinal;
      vector<Node*> Vertices1Quad1;
      vector<Node*> Vertices2Quad1;
      vector<Node*> Vertices1Quad2;
      vector<Node*> Vertices2Quad2;
      vector<Node*> Vertices1Quad3;
      vector<Node*> Vertices2Quad3;
      vector<double> bounds;
      vector<obsInfo> obstacles;
      vector<double> velocityConst;
      quadCopterDim QuadCopter;
};
