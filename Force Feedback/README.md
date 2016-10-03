README (Force Feddback):
************************
************************

The Force Feedback foler contains the following files:
1. main.m
2. get_Transformation_Matrix.m
3. getBasicData.m
4. plot_MTM.m
5. README.md

************************
************************

1. main.m
**********
This file is the main file that calls up the functions in the other files to calculate the joint torque during the simulation and also stories the joint angles. This data is then used to simulate the MTM.

2. get_Transformation_Matrix.m
*******************************
This is a function that returns the transformation matrix for the i'th link with respect to the previous link.

3. getBasicData.m
******************
This file contains the symbolic Transformatin Matrices of:
    a. All the links wrt the previous links.
    b. All the links wrt base.
    c. All COMs wrt base.

4. plot_MTM.m
**************
This is a function that takes in some values and displays each iteration of the simulation. Based on the various situations (when F_surgeon <, >, == F_psm), the plot gives visual information to the user.

************************
************************

How to run the code:
********************
This code is compatible with MATLAB 2015.

To run the code:
1. Import all the code into MATLAB.
2. Run the main.m file.a
