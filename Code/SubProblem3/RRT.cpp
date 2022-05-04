#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <cmath>
#include <time.h>
#include <fstream>
#include <Eigen/Dense>
#include "RRT.hpp"

using namespace std;
using namespace Eigen;


//Constructor takes in the bounds for the state space
RRT::RRT(vector<double> b, vector<double> vb) {

  bounds = b;
  velocityConst = vb;

};

//Destructor empties out the vertices of the graphs
RRT::~RRT() {

  for (int i = 0; i < Vertices1Quad1.size();i++) {
    Vertices1Quad1[i] = NULL;
  }
  for (int i =0; i <Vertices2Quad1.size();i++) {
    Vertices2Quad1[i] = NULL;
  }
  for (int i = 0; i < Vertices1Quad2.size();i++) {
    Vertices1Quad2[i] = NULL;
  }
  for (int i = 0; i < Vertices2Quad2.size();i++) {
    Vertices2Quad2[i] = NULL;
  }

  for (int i = 0; i < Vertices1Quad3.size();i++) {
    Vertices1Quad3[i] = NULL;
  }
  for (int i = 0; i < Vertices2Quad3.size();i++) {
    Vertices2Quad3[i] = NULL;
  }


};

//Takes in and defines the start positions for the quadcopters
void RRT::setStart(vector<double> Qstart) {
  StartLocation.push_back(Qstart);
};

//Takes in the intermediate and final goals for the 3 quadcopters
void RRT::setGoals(vector<double> Gfirst, vector<double> Glast) {
  GoalIntermediate.push_back(Gfirst);
  Goalfinal.push_back(Glast);
};

//Samples a random state from the state space
vector<double> RRT::SampleState(int option) {

  //State Vector
  double x;
  double xdot;
  double y;
  double ydot;
  double z;
  double zdot;
  double interm;

  if (option == 1) {
    interm = bounds[0];
  } else if (option==2) {
    interm = (bounds[1]-bounds[0])/2;
  }

  //Uniform Distribution generation for all states

  random_device r;
  default_random_engine generator(r());

  uniform_real_distribution<double> dit1(interm,bounds[1]);
  uniform_real_distribution<double> dit2(bounds[2],bounds[3]);
  uniform_real_distribution<double> dit3(bounds[4],bounds[5]);
  uniform_real_distribution<double> dit4(bounds[6],bounds[7]);
  uniform_real_distribution<double> dit5(bounds[8],bounds[9]);
  uniform_real_distribution<double> dit6(bounds[10],bounds[11]);

  x = dit1(generator);
  xdot = dit2(generator);
  y = dit3(generator);
  ydot = dit4(generator);
  z = dit5(generator);
  zdot = dit6(generator);

  vector<double> sampledstate{x,xdot,y,ydot,z,zdot};

  return sampledstate;

  };

//Generates a random real number between 0 and 1
double RRT::GenerateProbab() {

  random_device r;
  default_random_engine generator(r());
  uniform_real_distribution<double> dit(0,1);

  double probab = dit(generator);

  return probab;


};

//This is the main code where computations are initlized for all 3 quadcopters
vector<double> RRT::findPath(int n,double pgoal,double eps,string name1, string name2,string name3) {
  double PathLength1 = 0;
  double PathLength2 = 0;
  double PathLength3 = 0;
  Node* temp2;
  Node* temp3;
  Node* temp4;
  //Initializing Quadcopter 1 location
  Node* initial = new Node;
  initial->x = StartLocation[0][0];
  initial->xdot = StartLocation[0][1];
  initial->y = StartLocation[0][2];
  initial->ydot = StartLocation[0][3];
  initial->z = StartLocation[0][4];
  initial->zdot = StartLocation[0][5];
  initial->time = 0;
  Vertices1Quad1.push_back(initial);

  //Initialzing Quadcopter 2 location
  Node* initial2 = new Node;
  initial2->x = StartLocation[1][0];
  initial2->xdot = StartLocation[1][1];
  initial2->y = StartLocation[1][2];
  initial2->ydot = StartLocation[1][3];
  initial2->z = StartLocation[1][4];
  initial2->zdot = StartLocation[1][5];
  initial2->time = 0;
  Vertices1Quad2.push_back(initial2);

  //Initializing Quadcopter 3 location
  Node* initial3 = new Node;
  initial3->x = StartLocation[2][0];
  initial3->xdot = StartLocation[2][1];
  initial3->y = StartLocation[2][2];
  initial3->ydot = StartLocation[2][3];
  initial3->z = StartLocation[2][4];
  initial3->zdot = StartLocation[2][5];
  initial3->time = 0;
  Vertices1Quad3.push_back(initial3);

  PropagatingGoal(n,pgoal,eps);



  if (SuccessInterm1 && SuccessFinal1) {
    PathLength1 = totalPathLength(Vertices2Quad1.back());
  }

  if (SuccessInterm2 && SuccessFinal2) {
    PathLength2 = totalPathLength(Vertices2Quad2.back());
  }

  if (SuccessInterm3 && SuccessFinal3) {
    PathLength3 = totalPathLength(Vertices2Quad3.back());
  }

  vector<double> Lengths{PathLength1,PathLength2,PathLength3};

  temp2 = Vertices2Quad1.back();
  temp3 = Vertices2Quad2.back();
  temp4 = Vertices2Quad3.back();

  ofstream file;
  file.open(name1, ofstream::out | ofstream::trunc);
  file.close();
  file.open(name1, ios_base::app);
  while (temp2 != NULL) {
    file << temp2->x <<" "<< temp2->xdot << " " << temp2->y<<" "<<temp2->ydot << " " << temp2->z << " " << temp2->zdot << endl;
    temp2 = temp2->parent;
  }
  file.close();


  ofstream file2;
  file2.open(name2, ofstream::out | ofstream::trunc);
  file2.close();
  file2.open(name2, ios_base::app);
  while (temp3 != NULL) {
    file2 << temp3->x <<" "<< temp3->xdot << " " << temp3->y<<" "<<temp3->ydot << " " << temp3->z << " " << temp3->zdot << endl;
    temp3 = temp3->parent;
  }
  file2.close();

  ofstream file3;
  file3.open(name3, ofstream::out | ofstream::trunc);
  file3.close();
  file3.open(name3, ios_base::app);
  while (temp4 != NULL) {
    file3 << temp4->x <<" "<< temp4->xdot << " " << temp4->y<<" "<<temp4->ydot << " " << temp4->z << " " << temp4->zdot << endl;
    temp4 = temp4->parent;
  }
  file3.close();

  return Lengths;

  };

//This is the function that propagates the trees of each quadcopter
void RRT::PropagatingGoal(int n,double pgoal,double eps){

  vector<double>* x_rand = new vector<double>;
  vector<double>* temp = new vector<double>;
  Node* temp2;
  Node* temp3;
  double distance1 = 10;
  double distance2 = 10;
  double distance3 = 10;
  double distance4 = 10;
  double distance5 = 10;
  double distance6 = 10;
  double p;




  for (int i = 0; i < n; i++) {
      p = GenerateProbab();
      if (p > pgoal) {
        *(x_rand) = SampleState(1);
      } else {
        *(x_rand) = GoalIntermediate[0];
      }
      if (distance1 > eps) {
        *(temp) = Extend(x_rand,1,1);
        distance1 = dist(*(temp),GoalIntermediate[0]);
      }
      if (distance1 < eps) {
        cout << "QuadCopter 1 has reached the interm goal" << endl;
        //cout << "Distance: " << distance1 << endl;
        SuccessInterm1 = 1;
        break;
      }
    }
    if (distance1 > eps) {
      cout << "QuadCopter 1 has failed to reach interm goal" << endl;
    }

    Vertices2Quad1.push_back(Vertices1Quad1.back());
    for (int i = 0; i < n;i++) {
        p = GenerateProbab();
        if (p > pgoal) {
          *(x_rand) = SampleState(1);
        } else {
          *(x_rand) = Goalfinal[0];
        }
        if (distance4 > eps) {
          *(temp) = Extend(x_rand,2,1);
          distance4 = dist(*(temp),Goalfinal[0]);
        } else {
          cout << "QuadCopter 1 has reached the final goal" << endl;
          //cout << "Distance: " << distance1 << endl;
          SuccessFinal1 = 1;
          break;
        }
      }

      if (distance4 > eps) {
        cout << "QuadCopter 1 has failed to reach the final goal" << endl;
      }

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
    for (int i = 0; i < n;i++){
      p = GenerateProbab();
      if (p > pgoal) {
        *(x_rand) = SampleState(1);
      } else {
        *(x_rand) = GoalIntermediate[1];
      }
      if (distance2 > eps) {
        *(temp) = Extend(x_rand,1,2);
        distance2 = dist(*(temp),GoalIntermediate[1]);
      }
      if (distance2 < eps) {
        cout << "QuadCopter 2 has reached the interm goal" << endl;
        //cout << "Distance: " << distance2 << endl;
        //cout << " " << endl;
        SuccessInterm2 = 1;
        break;
      }
    }

    if (distance2 > eps) {
      cout << "QuadCopter 2 has failed to reach interm goal" << endl;
    }

    Vertices2Quad2.push_back(Vertices1Quad2.back());
    for (int i = 0; i<n ;i++) {
      p = GenerateProbab();
      if (p > pgoal) {
        *(x_rand) = SampleState(1);
      } else {
        *(x_rand) = Goalfinal[1];
      }
      if (distance5 > eps) {
        *(temp) = Extend(x_rand,2,2);
        distance5 = dist(*(temp),Goalfinal[1]);
      } else {
        cout << "QuadCopter 2 has reached the final goal" << endl;
        //cout << "Distance: " << distance2 << endl;
        //cout << " " << endl;
        SuccessFinal2 = 2;
        break;
      }
    }

    if (distance5 > eps) {
      cout << "QuadCopter 2 has failed to reach the final goal" << endl;
    }

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
    for (int i = 0; i < n;i++){
      p = GenerateProbab();
      if (p > pgoal) {
        *(x_rand) = SampleState(1);
      } else {
        *(x_rand) = GoalIntermediate[2];
      }
      if (distance3 > eps) {
        *(temp) = Extend(x_rand,1,3);
        distance3 = dist(*(temp),GoalIntermediate[2]);
      }
      if (distance3 < eps) {
        cout << "QuadCopter 3 has reached the interm goal" << endl;
        //cout << "Distance: " << distance3 << endl;
        SuccessInterm3 = 1;
        break;
      }
    }

    if (distance3 > eps) {
      cout << "QuadCopter 3 has failed to reach the interm goal" << endl;
    }

    Vertices2Quad3.push_back(Vertices1Quad3.back());
    for (int i = 0; i < n;i++){
      p = GenerateProbab();
      if (p > pgoal) {
        *(x_rand) = SampleState(1);
      } else {
        *(x_rand) = Goalfinal[2];
      }
      if (distance6 > eps) {
        *(temp) = Extend(x_rand,2,3);
        distance6 = dist(*(temp),Goalfinal[2]);
      }
      if (distance6 < eps) {
        cout << "QuadCopter 3 has reached the final goal" << endl;
        //cout << "Distance: " << distance3 << endl;
        //cout << " " << endl;
        SuccessFinal3 = 1;
        break;
      }
    }
    if (distance6 > eps) {
      cout << "QuadCopter 3 has failed to reach the final goal" << endl;
    }

    temp = NULL;
    x_rand = NULL;

};

//This is the function that extends a node and adds to a tree accordingly
vector<double> RRT::Extend(vector<double>* RandPtr,int option,int QuadNum ) {

  Node* closestPoint = closest(RandPtr,option, QuadNum);
  Node* addition;
  Node* temp = NULL;
  Node* temp2 = NULL;
  Node* temp3 = NULL;
  Node* temp4 = NULL;
  Node* temp5 = NULL;
  Node* temp6 = NULL;
  Node* tempA;
  Node* tempB;
  VectorXd* Input = new VectorXd;
  vector<double>* x_extend = new vector<double>;
  vector<double>* candidate;
  vector<double>* Newest = new vector<double>;
  double minimum = 1000;
  double distance;

  for (int i = 0; i <20;i++) {

    *(Input) = SampleControlInput();
    *(x_extend) = fDynamics(closestPoint,*(Input));
    distance = dist(*(x_extend),*(RandPtr));
    if (distance < minimum) {
      candidate = x_extend;
      minimum = distance;
    }
  }

  if (CollisionChecker( (*(candidate))[0],(*(candidate))[2],(*(candidate))[4] ) &&
      WithinConstraints({(*(candidate))[1],(*(candidate))[3],(*(candidate))[5]})) {

        addition = new Node;
        addition->x = (*(candidate))[0];
        addition->xdot = (*(candidate))[1];
        addition->y = (*(candidate))[2];
        addition->ydot = (*(candidate))[3];
        addition->z = (*(candidate))[4];
        addition->zdot = (*(candidate))[5];
        addition->parent = closestPoint;
        addition->time = addition->parent->time + 0.5;

        if (option == 1) {
          if (QuadNum == 1) {
            Vertices1Quad1.push_back(addition);
          } else if (QuadNum == 2) {
          //////////////////////////////////////////////////////////////////////
          tempA = Vertices2Quad1.back();
          while (tempA != NULL) {
              if (tempA->time == addition->time) {
                temp = tempA;
                break;
              }
              tempA = tempA->parent;
            }

            if (temp!= NULL) {
              if (QuadCollisionChecker(addition->x,addition->y,addition->z,
              temp->x,temp->y,temp->z)) {
                Vertices1Quad2.push_back(addition);
              } else {
                addition = NULL;
              }
            } else {
              Vertices1Quad2.push_back(addition);
            }
////////////////////////////////////////////////////////////////////////////////
          } else if (QuadNum == 3) {
            tempA = Vertices2Quad1.back();
            tempB = Vertices2Quad2.back();
            while (tempA != NULL) {
                if (tempA->time == addition->time) {
                  temp2 = tempA;
                  break;
                }
                tempA = tempA->parent;
              }
            while(tempB != NULL) {
              if (tempB->time == addition->time) {
                temp3 = tempB;
                break;
              }
              tempB = tempB->parent;
            }

            if (Quad3Checker(addition,temp2,temp3)) {
              Vertices1Quad3.push_back(addition);
            } else {
              addition = NULL;
            }


          }
      //////////////////////////////////////////////////////////////////////////
        } else {
          if (QuadNum == 1) {
            Vertices2Quad1.push_back(addition);
          } else if (QuadNum == 2) {

            tempA = Vertices2Quad1.back();
            while (tempA != NULL) {
                if (tempA->time == addition->time) {
                  temp4 = tempA;
                  break;
                }
                tempA = tempA->parent;
              }
            if (temp4 != NULL) {
              if (QuadCollisionChecker(addition->x,addition->y,addition->z,
              temp4->x,temp4->y,temp4->z)) {
                Vertices2Quad2.push_back(addition);
              } else {
                addition = NULL;
              }
            } else {
              Vertices2Quad2.push_back(addition);
            }
////////////////////////////////////////////////////////////////////////////////
          } else if (QuadNum == 3) {
            tempA = Vertices2Quad1.back();
            tempB = Vertices2Quad2.back();
            while (tempA != NULL) {
                if (tempA->time == addition->time) {
                  temp5 = tempA;
                  break;
                }
                tempA = tempA->parent;
              }
            while(tempB != NULL) {
              if (tempB->time == addition->time) {
                temp6 = tempB;
                break;
              }
              tempB = tempB->parent;
            }

            if (Quad3Checker(addition,temp5,temp6)) {
              Vertices2Quad3.push_back(addition);
            } else {
              addition = NULL;
            }
          }
        }

      }

      return *(candidate);

};

//This function propgates the dynamics for the quadcopters by a delta x
vector<double> RRT::fDynamics(Node* candidate,VectorXd Input) {

  //Amatrix
  MatrixXd A(6,6);
  A(0,0)= 0;A(0,1)= 1;A(0,2)= 0;A(0,3)= 0;A(0,4)= 0;A(0,5)= 0;
  A(1,0)= 0;A(1,1)= -0.0104;A(1,2)= 0;A(1,3)= 0;A(1,4)= 0;A(1,5)= 0;
  A(2,0)= 0;A(2,1)= 0;A(2,2)= 0;A(2,3)=1;A(2,4)=0;A(2,5)=0;
  A(3,0)= 0;A(3,1)= 0;A(3,2)= 0;A(3,3)= -0.0104;A(3,4)= 0;A(3,5)= 0;
  A(4,0)= 0;A(4,1)= 0;A(4,2)= 0;A(4,3)= 0;A(4,4)= 0;A(4,5)= 1;
  A(5,0)= 0;A(5,1)= 0;A(5,2)= 0;A(5,3)= 0;A(5,4)= 0;A(5,5) =-0.0208;

  //Bmatrix
  MatrixXd B(6,4);
  B(0,0)= 0;B(0,1)= 0;B(0,2)= 0;B(0,3)= 0;
  B(1,0)= -0.0417;B(1,1) =0;B(1,2) =0.0417;B(1,3)=0;
  B(2,0)= 0;B(2,1)= 0;B(2,2)= 0;B(2,3) = 0;
  B(3,0)= 0;B(3,1)= -0.0417;B(3,2)= 0;B(3,3)= 0.0417;
  B(4,0)= 0;B(4,1)= 0;B(4,2)= 0;B(4,3)= 0;
  B(5,0)= 0.4;B(5,1)= 0.4;B(5,2)= 0.4;B(5,3)= 0.4;


  //Exponential of A*(t_0 - t)
  MatrixXd expA(6,6);
  expA(0,0)= 1;expA(0,1)= 0;expA(0,2)= 0;expA(0,3)= 0;expA(0,4)= 0;expA(0,5)= 0;
  expA(1,0)= -5.1323;expA(1,1)= 1.0534;expA(1,2)= 0;expA(1,3)= 0;expA(1,4)= 0;expA(1,5)= 0;
  expA(2,0)= 0;expA(2,1)= 0;expA(2,2)= 1;expA(2,3)=0;expA(2,4)=0;expA(2,5)=0;
  expA(3,0)= 0;expA(3,1)= 0;expA(3,2)= -5.1323;expA(3,3)= 1.0534;expA(3,4)= 0;expA(3,5)= 0;
  expA(4,0)= 0;expA(4,1)= 0;expA(4,2)= 0;expA(4,3)= 0;expA(4,4)= 1;expA(4,5)= 0;
  expA(5,0)= 0;expA(5,1)= 0;expA(5,2)= 0;expA(5,3)= 0;expA(5,4)= -5.2693;expA(5,5) =1.1096;


  // k matrix
  /*MatrixXd k(4,6);
  k(0,0) = -303.2285;k(0,1) = -120.5148;k(0,2) = -0.5112;k(0,3) = -0.1016;k(0,4) = 15.9167;k(0,5) = 6.2950;
  k(1,0) = -0.6428;k(1,1) = -0.1277;k(1,2) = -304.5861;k(1,3) = -120.7848;k(1,4) = 16.5061;k(1,5) = 6.4120;
  k(2,0) = 303.2243;k(2,1) = 120.5140;k(2,2) = 0.4681;k(2,3) = 0.0930;k(2,4) = 15.8331;k(2,5) = 6.2786;
  k(3,0) = 0.6386;k(3,1) = 0.1269;k(3,2) = 304.5430;k(3,3) = 120.7763;k(3,4) = 15.2437;k(4,4) = 6.1616;
  */
  /*//F matrix
  MatrixXd F(4,3);
  F(0,0) = -303.2285;F(0,1) = -0.5112;F(0,2) = 15.9167;
  F(1,0) = -0.6428;F(1,1) = -304.5861;F(1,2) = 16.5061;
  F(2,0) = 303.2243;F(2,1) = 0.4681;F(2,2) = 15.8331;
  F(3,0) = 0.6386;F(3,1) = 304.5430;F(3,2) = 15.2437;
  */
  //Current Condition
  VectorXd x(6);
  x(0)=candidate->x;
  x(1)=candidate->xdot;
  x(2)=candidate->y;
  x(3)=candidate->ydot;
  x(4)=candidate->z;
  x(5)=candidate->zdot;

  VectorXd deriv1 = A*x + B*(B.transpose())*(expA)*(Input);
  VectorXd Xchange = x+deriv1;
  vector<double> Xnew{Xchange(0),Xchange(1),Xchange(2),Xchange(3),Xchange(4),Xchange(5)};

  /*for (int i =0; i <6;i++){
    cout << deriv1(i) << endl;
  }
  cout << "--"<< endl;*/

  return Xnew;





};

//This function samples a control input for fDynamics
VectorXd RRT::SampleControlInput() {


    VectorXd Input(6);
    random_device r;
    default_random_engine generator(r());
    uniform_real_distribution<double> dit1(-1,1);
    uniform_real_distribution<double> dit2(-1,1);
    uniform_real_distribution<double> dit3(-1,1);
    uniform_real_distribution<double> dit4(-1,1);
    uniform_real_distribution<double> dit5(-0.01,0.01);
    uniform_real_distribution<double> dit6(-0.01,0.01);


    Input(0) = dit1(generator);
    Input(1) = dit2(generator);
    Input(2) = dit3(generator);
    Input(3) = dit4(generator);
    Input(4) = dit5(generator);
    Input(5) = dit6(generator);


    return Input;
};

//This function computes the total path length taken by a quadcopter
double RRT::totalPathLength(Node* node) {

  double distance = 0;
  Node* temp = node;

  while (temp->parent != NULL) {
    distance = distance + dist2({temp->x,temp->y,temp->z},{temp->parent->x,
    temp->parent->y,temp->parent->z});
    temp = temp->parent;
  }
  return distance;

};

//This function collision checks a quadcopter's position against the obstacles
bool RRT::CollisionChecker(double x, double y, double z) {

    QuadVertices current = locate(x,y,z);
    double dL;
    double dW;
    double dH;
    double x_c;
    double y_c;
    double z_c;
    double v_x;
    double v_y;
    double v_z;

    for (int j = 0; j<current.vert_x.size();j++) {
      v_x = current.vert_x[j];
      v_y = current.vert_y[j];
      v_z = current.vert_z[j];
      for (int i = 0; i <obstacles.size();i++) {
        x_c = obstacles[i].center[0];
        y_c = obstacles[i].center[1];
        z_c = obstacles[i].center[2];
        dL = obstacles[i].length/2;
        dW = obstacles[i].width/2;
        dH = obstacles[i].height/2;
        if ( v_x>= (x_c-dL) && v_x<= (x_c+dL) && v_y>=(y_c-dW) && v_y<=(y_c+dW) &&
             v_z>= (z_c-dH) && v_z<=(z_c+dH)) {
               return 0;
             }
      }
    }

    return 1;

};

//This function is designed for the third quadcopter to collision check against
// the trajectories of the 1st and 2nd quadcopters
bool RRT::Quad3Checker(Node* candidate,Node* tempA, Node* tempB) {

  /*if (tempA != NULL && tempB != NULL){
    cout << "Quad1: " << tempA->x << " " << tempA->y << " " << tempA->z << endl;
    cout << "Quad2: " << tempB->x << " " << tempB->y << " " << tempB->z << endl;
    cout << "Quad3: " << candidate->x << " " << candidate->y << " " << candidate->z << endl;
    cout << "1 check: " << QuadCollisionChecker(candidate->x,candidate->y,candidate->z,
    tempA->x,tempA->y,tempA->z) << endl;
    cout << "2 check: " << QuadCollisionChecker(candidate->x,
    candidate->y,candidate->z,tempB->x,tempB->y,tempB->z);
  }*/
  if (tempA != NULL && tempB != NULL) {
    if (QuadCollisionChecker(candidate->x,candidate->y,candidate->z,
        tempA->x,tempA->y,tempA->z) && QuadCollisionChecker(candidate->x,
        candidate->y,candidate->z,tempB->x,tempB->y,tempB->z)) {
          return 1;
        }
  } else if (tempA == NULL && tempB == NULL) {
    return 1;
  } else if (tempB == NULL) {
    if (QuadCollisionChecker(candidate->x,candidate->y,candidate->z,tempA->x,
        tempA->y,tempA->z)) {
          return 1;
        }
  } else if (tempA == NULL) {
    if (QuadCollisionChecker(candidate->x,candidate->y,candidate->z,tempB->x,
        tempB->y,tempB->z)) {
          return 1;
        }
    }
  return 0;

};

//This function checks whether two quadcopters collide at given locations
bool RRT::QuadCollisionChecker(double x, double y, double z,double x2,
  double y2,double z2) {

    QuadVertices Quad1 = locate(x,y,z);
    QuadVertices Quad2 = locate(x2,y2,z2);

    for (int i = 0; i <Quad1.vert_x.size();i++) {
      if (Quad1.vert_x[i] <= Quad2.vert_x[0] && Quad1.vert_x[i] >= Quad2.vert_x[2] &&
          Quad1.vert_y[i] <= Quad2.vert_y[0] && Quad1.vert_y[i] >= Quad2.vert_y[1] &&
          Quad1.vert_z[i] <= Quad2.vert_z[7] && Quad1.vert_z[i] >= Quad2.vert_z[0]) {
            return 0;
          }
    }

    return 1;
};

//This function determines the vertices of the quadcopter based on a given location
QuadVertices RRT::locate(double x, double y, double z) {

    QuadVertices current;

////////////Bottom//////////////////////////////////////////////////////////////
    //Bottom Right front Vertex
    current.vert_x.push_back(x+QuadCopter.length/2);
    current.vert_y.push_back(y+QuadCopter.width/2);
    current.vert_z.push_back(z-QuadCopter.height/2);

    //Bottom left front vertex
    current.vert_x.push_back(x+QuadCopter.length/2);
    current.vert_y.push_back(y-QuadCopter.width/2);
    current.vert_z.push_back(z-QuadCopter.height/2);

    //Bottom left back vertex
    current.vert_x.push_back(x-QuadCopter.length/2);
    current.vert_y.push_back(y-QuadCopter.width/2);
    current.vert_z.push_back(z-QuadCopter.height/2);

    //Bottom right back vertex
    current.vert_x.push_back(x-QuadCopter.length/2);
    current.vert_y.push_back(y+QuadCopter.width/2);
    current.vert_z.push_back(z-QuadCopter.height/2);
////////////Top/////////////////////////////////////////////////////////////////
    //Top right front vertex
    current.vert_x.push_back(x+QuadCopter.length/2);
    current.vert_y.push_back(y+QuadCopter.width/2);
    current.vert_z.push_back(z+QuadCopter.height/2);

    //Top left front vertex
    current.vert_x.push_back(x+QuadCopter.length/2);
    current.vert_y.push_back(y-QuadCopter.width/2);
    current.vert_z.push_back(z+QuadCopter.height/2);

    //Top left back vertex
    current.vert_x.push_back(x-QuadCopter.length/2);
    current.vert_y.push_back(y-QuadCopter.width/2);
    current.vert_z.push_back(z+QuadCopter.height/2);

    //Top right back vertex
    current.vert_x.push_back(x-QuadCopter.length/2);
    current.vert_y.push_back(y+QuadCopter.width/2);
    current.vert_z.push_back(z+QuadCopter.height/2);

    return current;

};

//This function finds the closest node on a tree to a random state
Node* RRT::closest(vector<double>* randPtr,int option,int QuadNum) {

    double minimum = 1000;
    Node* closest;
    Node* q;

    if(option == 1) {
    ////////////////////////////////////////////////////////////////////////////
      if (QuadNum == 1) {
        for (int i = 0; i<Vertices1Quad1.size();i++) {
          q = Vertices1Quad1[i];
          if (dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot})< minimum) {

                    minimum = dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot});
                    closest = Vertices1Quad1[i];
                  }
        }
      } else if (QuadNum == 2) {
        for (int i = 0; i<Vertices1Quad2.size();i++) {
          q = Vertices1Quad2[i];
          if (dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot})< minimum) {

                    minimum = dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot});
                    closest = Vertices1Quad2[i];

          }
        }
      } else if (QuadNum == 3) {

        for (int i = 0; i<Vertices1Quad3.size();i++) {
          q = Vertices1Quad3[i];
          if (dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot})< minimum) {

                    minimum = dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot});
                    closest = Vertices1Quad3[i];

            }
        }
      }
    ////////////////////////////////////////////////////////////////////////////
    } else {
    ////////////////////////////////////////////////////////////////////////////


        if (QuadNum == 1) {
          for (int i = 0; i<Vertices2Quad1.size();i++) {
            q = Vertices2Quad1[i];
            if (dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot})< minimum) {

                      minimum = dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot});
                      closest = Vertices2Quad1[i];

          }
        }
      } else if (QuadNum == 2) {
        for (int i = 0; i<Vertices2Quad2.size();i++) {
          q = Vertices2Quad2[i];
          if (dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot})< minimum) {

                    minimum = dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot});
                    closest = Vertices2Quad2[i];

      }
    }
  } else if (QuadNum == 3) {
    for (int i = 0; i<Vertices2Quad3.size();i++) {
      q = Vertices2Quad3[i];
      if (dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot})< minimum) {

                minimum = dist( *randPtr, {q->x,q->xdot,q->y,q->ydot,q->z,q->zdot});
                closest = Vertices2Quad3[i];

          }
        }
    }
}
    return closest;


};

//This function computes the distance between two states
double RRT::dist(vector<double> arr, vector<double> arr2) {
    double distance;
    distance = sqrt( (arr[0] - arr2[0])*(arr[0] - arr2[0]) + (arr[1] - arr2[1])*(arr[1] - arr2[1])
                    +  (arr[2]-arr2[2])*(arr[2]-arr2[2]) + (arr[3]-arr2[3])*(arr[3]-arr2[3]) + (arr[4]-arr2[4])*(arr[4]-arr2[4]) +
                       (arr[5]-arr2[5])*(arr[5]-arr2[5]));
    return distance;
  };

//This function computes the distance between only the positions of two states
double RRT::dist2(vector<double> arr, vector<double> arr2) {

  double distance;
  distance = sqrt((arr[0] - arr2[0])*(arr[0] - arr2[0]) + (arr[2]-arr2[2])*(arr[2]-arr2[2])
   + (arr[4]-arr2[4])*(arr[4]-arr2[4]));
  return distance;

};

//This function checks to see whether a state satisfies the velocity constraints
bool RRT::WithinConstraints(vector<double> arr) {

    if (arr[0] >= velocityConst[0] && arr[0] <= -velocityConst[0]){
      return 0;
    }

    if(arr[1] >= velocityConst[1] && arr[1] <= -velocityConst[1]){
      return 0;
    }

    if(arr[2] >= velocityConst[2] && arr[2] <= -velocityConst[2]){
      return 0;
    }

    return 1;

};

//This function adds an obstacle
void RRT::Obstacle(vector<double> cent, vector<double> dim) {

  obsInfo obs;
  obs.center = cent;
  obs.length = dim[0];
  obs.width = dim[1];
  obs.height = dim[2];
  obstacles.push_back(obs);
};

//This function defines the dimensions of the quadcopters
void RRT::Quad(vector<double> dim) {

  QuadCopter.length = dim[0];
  QuadCopter.width = dim[1];
  QuadCopter.height = dim[2];

};

//This function clears out the vertices of a graph
void RRT::clear() {

   Vertices1Quad1.clear();
   Vertices2Quad1.clear();
   Vertices1Quad2.clear();
   Vertices2Quad2.clear();
   Vertices1Quad3.clear();
   Vertices2Quad3.clear();

};
