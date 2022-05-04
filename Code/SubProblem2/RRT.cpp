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

RRT::RRT(vector<double> b, vector<double> vb) {

  bounds = b;
  velocityConst = vb;

};


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

};


void RRT::setStart(vector<double> Qstart) {
  StartLocation.push_back(Qstart);
};

void RRT::setGoals(vector<double> Gfirst, vector<double> Glast) {
  GoalIntermediate.push_back(Gfirst);
  Goalfinal.push_back(Glast);
};

vector<double> RRT::SampleState() {

  //State Vector
  double x;
  double xdot;
  double y;
  double ydot;
  double z;
  double zdot;

  //Uniform Distribution generation for all states
  random_device r;
  default_random_engine generator(r());
  uniform_real_distribution<double> dit1(bounds[0],bounds[1]);
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

double RRT::GenerateProbab() {

  random_device r;
  default_random_engine generator(r());
  uniform_real_distribution<double> dit(0,1);

  double probab = dit(generator);

  return probab;


};

void RRT::findPath(int n,double pgoal,double eps) {

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

  PropagatingGoalInterm(n,pgoal,eps);





  };

void RRT::PropagatingGoalInterm(int n,double pgoal,double eps){

  vector<double>* x_rand = new vector<double>;
  vector<double>* temp = new vector<double>;
  Node* temp2;
  Node* temp3;
  double distance1 = 10;
  double distance2 = 10;
  double distance3 = 10;
  double distance4 = 10;
  double p;





  for (int i = 0; i < n; i++) {
      p = GenerateProbab();
      if (p > pgoal) {
        *(x_rand) = SampleState();
      } else {
        *(x_rand) = GoalIntermediate[0];
      }
      if (distance1 > eps) {
        *(temp) = Extend(x_rand,1,1);
        distance1 = dist(*(temp),GoalIntermediate[0]);
      }
      if (distance1 < eps) {
        cout << "QuadCopter 1 has reached the interm goal" << endl;
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
          *(x_rand) = SampleState();
        } else {
          *(x_rand) = Goalfinal[0];
        }
        if (distance3 > eps) {
          *(temp) = Extend(x_rand,2,1);
          distance3 = dist(*(temp),Goalfinal[0]);
        } else {
          cout << "QuadCopter 1 has reached the final goal" << endl;
          break;
        }
      }

      if (distance3 > eps) {
        cout << "QuadCopter 1 has failed to reach the final goal" << endl;
      }

////////////////////////////////////////////////////////////////////////////////

    for (int i = 0; i < n;i++){
      p = GenerateProbab();
      if (p > pgoal) {
        *(x_rand) = SampleState();
      } else {
        *(x_rand) = GoalIntermediate[1];
      }
      if (distance2 > eps) {
        *(temp) = Extend(x_rand,1,2);
        distance2 = dist(*(temp),GoalIntermediate[1]);
      }
      if (distance2 < eps) {
        cout << "QuadCopter 2 has reached the interm goal" << endl;
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
        *(x_rand) = SampleState();
      } else {
        *(x_rand) = Goalfinal[1];
      }
      if (distance4 > eps) {
        *(temp) = Extend(x_rand,2,2);
        distance4 = dist(*(temp),Goalfinal[1]);
      } else {
        cout << "QuadCopter 2 has reached the final goal" << endl;
        break;
      }
    }

    if (distance4 > eps) {
      cout << "QuadCopter 2 has failed to reach the final goal" << endl;
    }

    temp = NULL;
    x_rand = NULL;

    temp2 = Vertices2Quad1.back();
    temp3 = Vertices2Quad2.back();
    ofstream file;
    file.open("test1.txt", ofstream::out | ofstream::trunc);
    file.close();
    file.open("test1.txt", ios_base::app);
    while (temp2 != NULL) {
      file << temp2->x <<" "<< temp2->xdot << " " << temp2->y<<" "<<temp2->ydot << " " << temp2->z << " " << temp2->zdot << endl;
      temp2 = temp2->parent;
    }
    file.close();

    ofstream file2;
    file2.open("test2.txt", ofstream::out | ofstream::trunc);
    file2.close();
    file2.open("test2.txt", ios_base::app);
    while (temp3 != NULL) {
      file2 << temp3->x <<" "<< temp3->xdot << " " << temp3->y<<" "<<temp3->ydot << " " << temp3->z << " " << temp3->zdot << endl;
      temp3 = temp3->parent;
    }
    file2.close();

};

void RRT::PropagatingGoalFinal(int n,double pgoal,double eps) {

  vector<double>* x_rand = new vector<double>;
  vector<double>* temp = new vector<double>;
  Node* temp2;
  Node* temp3;
  double distance1 = 10;
  double distance2 = 10;
  double p;




  for (int i = 0; i < n;i++) {
      p = GenerateProbab();
      if (p > pgoal) {
        *(x_rand) = SampleState();
      } else {
        *(x_rand) = Goalfinal[0];
      }
      if (distance1 > eps) {
        *(temp) = Extend(x_rand,2,1);
        distance1 = dist(*(temp),Goalfinal[0]);
      } else {
        cout << "QuadCopter 1 has reached the final goal" << endl;
        break;
      }
    }

    if (distance1 > eps) {
      cout << "QuadCopter 1 has failed to reach the final goal" << endl;
    }


    for (int i = 0; i<n ;i++) {
      p = GenerateProbab();
      if (p > pgoal) {
        *(x_rand) = SampleState();
      } else {
        *(x_rand) = Goalfinal[1];
      }
      if (distance2 > eps) {
        *(temp) = Extend(x_rand,2,2);
        distance2 = dist(*(temp),Goalfinal[1]);
      } else {
        cout << "QuadCopter 2 has reached the final goal" << endl;
        break;
      }
    }

    if (distance2 > eps) {
      cout << "QuadCopter 2 has failed to reach the final goal" << endl;
    }


    temp = NULL;
    x_rand = NULL;




};

vector<double> RRT::Extend(vector<double>* RandPtr,int option,int QuadNum ) {

  Node* closestPoint = closest(RandPtr,option, QuadNum);
  Node* addition;
  Node* temp = NULL;
  Node* temp2 = NULL;
  Node* tempA;
  Node* tempB;
  VectorXd* Input = new VectorXd;
  vector<double>* x_extend = new vector<double>;
  vector<double>* candidate;
  vector<double>* Newest = new vector<double>;
  double minimum = 1000;
  double distance;

  for (int i = 0; i <10;i++) {

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
        closestPoint->child = addition;
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
            ////////////////////////////////////////////////////////////////////
          }
        } else {
          if (QuadNum == 1) {
            Vertices2Quad1.push_back(addition);
          } else if (QuadNum == 2) {

            tempB = Vertices2Quad1.back();
            while (tempB != NULL) {
                if (tempB->time == addition->time) {
                  temp2 = tempB;
                  break;
                }
                tempB = tempB->parent;
              }
            if (temp2 != NULL) {
              if (QuadCollisionChecker(addition->x,addition->y,addition->z,
              temp2->x,temp2->y,temp2->z)) {
                Vertices2Quad2.push_back(addition);
              } else {
                addition = NULL;
              }
            } else {
              Vertices2Quad2.push_back(addition);
            }

          }
        }

      }

      return *(candidate);

};

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

  //Current Condition
  VectorXd x(6);
  x(0)=candidate->x;
  x(1)=candidate->xdot;
  x(2)=candidate->y;
  x(3)=candidate->ydot;
  x(4)=candidate->z;
  x(5)=candidate->zdot;

  VectorXd deriv1 = A*x + B*(B.transpose())*(expA)*(Input);
  VectorXd Xchange = x+deriv1*0.5;
  vector<double> Xnew{Xchange(0),Xchange(1),Xchange(2),Xchange(3),Xchange(4),Xchange(5)};

  /*for (int i =0; i <6;i++){
    cout << deriv1(i) << endl;
  }
  cout << "--"<< endl;*/

  return Xnew;





};

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
  }
}
    return closest;


};

double RRT::dist(vector<double> arr, vector<double> arr2) {
    double distance;
    distance = sqrt( (arr[0] - arr2[0])*(arr[0] - arr2[0]) + (arr[1] - arr2[1])*(arr[1] - arr2[1])
                    +  (arr[2]-arr2[2])*(arr[2]-arr2[2]) + (arr[3]-arr2[3])*(arr[3]-arr2[3]) + (arr[4]-arr2[4])*(arr[4]-arr2[4]) +
                       (arr[5]-arr2[5])*(arr[5]-arr2[5]));
    return distance;

  };

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

void RRT::Obstacle(vector<double> cent, vector<double> dim) {

  obsInfo obs;
  obs.center = cent;
  obs.length = dim[0];
  obs.width = dim[1];
  obs.height = dim[2];
  obstacles.push_back(obs);
};

void RRT::Quad(vector<double> dim) {

  QuadCopter.length = dim[0];
  QuadCopter.width = dim[1];
  QuadCopter.height = dim[2];

};
