C++ was used for the computation and MATLAB
was used for the plotting in this project

There are ways 2 run the code (preferrably on the first):
NOTE: I use g++ for my c++ compiler. version 9.3
NOTE: Parameters must be specified from within the code itself
in Main3Quadcopter.cpp

Before proceeding select the proper directory for the folder.
You must also include a library called Eigen3 as it is used
in this code.

It is preferred that this is run on Ubuntu as windows might not
detect the linking of the library eigen3

NOTE: instructions on how to define parameters are in the code
explained over the workspace 1 code parameters in Main3Quadcopters.cpp

1. Ubuntu (Linux): Ensure that all the .cpp and .hpp files remain in
the same folder.
     a) Enter g++ -I(Location of eigen3 library) Main3Quadcopters.cpp RRT.cpp 
	into the Ubuntu command line terminal
     b) Enter ./a.out which will run the entire code

The c++ code will output 9 text files (3 files for each workspace),

Workspace1 ------> Quad1Path.txt Quad2Path.txt Quad3Path.txt

Workspace2 ------> Quad1PathW2.txt Quad2PathW2.txt Quad3PathW2.txt

Workspace3 ------> Quad1PathW3.txt Quad2PathW3.txt Quad3PathW3.txt

MATLAB plots the demo solutions that I used for my report and thus
the names for the loaded files above must be changed to the ones above to see them
on plot