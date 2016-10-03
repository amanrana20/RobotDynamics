%% Impedance Control of dVRK + Haptic feedback from PSM to MTM.
% Project for course: << RBE501 Robot Dynamics >>
% ->> Tested on MATLAB 2011, 2013 and 2015.

% >>>> ALL UNITS ARE IN millimeters (mm) <<<<

% TEAM MEMBERS: 
% 1. Aman Rana
% 2. Amit Trivedi
% 3. Kiran Mohan
% 4. Akanksha Devkar
% 5. Terence Carmichael.

% This file calculates the forward kinenatics of the Master Tool
% Manipulator to find the end effector position and then plots the
% position. The inverse kinematics of the EE position is also calculated.
% Since only the first three links are affecting the end effector position.
% Inverse kinematics relating to q1, q2, and q3 has been executed. The
% formulas used have been explained in the code in this file sand the related files.

% OUTPUT in console:
% The output in the console dispays the initial position of the end
% effector of the robot (units =  mm). Then the program asks the user for the desired
% position that they want the end effector to be (units = mm).
% After the user inputs the coordinates, the program calcutates the angles
% of the arms at the desired end effector position and displays the
% configuration of the robot at the desired end effector coordinates.


%% Setup
clc
clear all
close all


%% Initializing
syms q1 q2 q3 q4 q5 q6 q7 q8
L_arm = 279.4; % mm
L_forearm = 304.8; %mm
h = 150.6; % mm

% DH Parameters
a = [0, L_arm, L_forearm, 0, 0, 0, 0];
alpha = [90, 0, -90, 90, -90, 90, 0];
d = [0, 0, 0, h, 0, 0, 0];
theta = [-pi/2+q1, -pi/2+q2, pi/2+q3, q4, q5, -pi/2+q6, pi/2+q7];

% USERS CAN CHANGE VALUES HERE.
theta_numeric = [0, 0, 0, 0, 0, 0, 0];

% DO NOT CHANGE THIS.
theta_numeric = theta_numeric*pi/180;


%% Calculations
% Transformation Matrices
T10 = get_transformation_matrix(theta(1), d(1), a(1), alpha(1));
T10 = simplify(T10);
% display(T10);
T10_numeric = subs(T10, q1, theta_numeric(1));

T21 = get_transformation_matrix(theta(2), d(2), a(2), alpha(2));
T21 = simplify(T21);
T20 = T10 * T21;
% display(T20);
T20_numeric = subs(T20, [q1, q2], theta_numeric(1:2));

T32 = get_transformation_matrix(theta(3), d(3), a(3), alpha(3));
T32 = simplify(T32);
T30 = T20 * T32;
% display(T30);
T30_numeric = subs(T30, [q1, q2, q3], theta_numeric(1:3));

T43 = get_transformation_matrix(theta(4), d(4), a(4), alpha(4));
T43 = simplify(T43);
T40 = T30 * T43;
% display(T40);
T40_numeric = subs(T40, [q1, q2, q3, q4], theta_numeric(1:4));

T54 = get_transformation_matrix(theta(5), d(5), a(5), alpha(5));
T54 = simplify(T54);
T50 = T40 * T54;
% display(T50);
T50_numeric = subs(T50, [q1, q2, q3, q4, q5], theta_numeric(1:5));

T65 = get_transformation_matrix(theta(6), d(6), a(6), alpha(6));
T65 = simplify(T65);
T60 = T50 * T65;
% display(T60);
T60_numeric = subs(T60, [q1, q2, q3, q4, q5, q6], theta_numeric(1:6));

T76 = get_transformation_matrix(theta(7), d(7), a(7), alpha(7));
T76 = simplify(T76);
T70 = T60 * T76;
% display(T70);
T70_numeric = double(subs(T70, [q1, q2, q3, q4, q5, q6, q7], theta_numeric(1:7)));
% display(T70_numeric(1:3, 4));


%% Plotting the initial configuration of the arm
% Sending the values of Tn0 where all the algles are 0 degrees. i.e. BASE CONFIGURATION
figure;
title('Blue: initial position.  Green: final position.');
plot_da_vinci(T10_numeric(1:3, 4), T20_numeric(1:3, 4), T30_numeric(1:3, 4), T40_numeric(1:3, 4), T50_numeric(1:3, 4), T60_numeric(1:3, 4), T70_numeric(1:3, 4), 1);


%% Calculating Jacobian
J = calculate_jacobian(T30); % Pass the Tn0 matrices to calculate_jacobian function
J = simplify(J);
% J = subs(J, [q1, q2, q3, q4, q5, q6, q7], [0, 0, 0, 0, 0, 0, 0]);
% display(J);


%% Calculating inverse Jacobian
J_inv = inv(J);
J_inv = simplify(J_inv);
% display(J_inv);

display('- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -');
initial_position_of_end_effector = round(T40_numeric(1:3, 4));
display(initial_position_of_end_effector);


%% Desired end effector position input by user
display('Enter the desired position of the end effector. (units: mm)');
desired_posX = round(input('Enter desired x: '));
desired_posY = round(input('Enter desired y: '));
desired_posZ = input('Enter desired Z: ');
desired_pos = [desired_posX; desired_posY; desired_posZ];
display(' ');
display('Coordinates enter by user:');
display(' ');
disp(desired_pos);

display(' ');
display('Inputs Stored. Processing...');


%% CALCULATE CHANGE IN THETA BASED ON CHANGE IN END EFFECTOR POSITION
% THEORY
% d_theta_dot = J_inverse* d_X_dot
% (where X = position of end effector & d_X is velocity of End Effector)
% MULTIPLYING BY d_t pon both sides
% d_theta = J_inverse* d_X

%%%% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% Initializing the value of q1, q2 and q3 to be 0 each.
angles = [0, 0, 0];
step_pos = initial_position_of_end_effector;

% Scaling factor is used for fast movement of the arm at first but small
% movement of the arms as they approach the desired position.
scaling_factor = 10;

% max_dist refers to the aucledian distance between initial and final
% position.
max_dist = norm(desired_pos - initial_position_of_end_effector);

% gap refers to the distance between the current state and desired
% position. Initial position of 10 has been randomly given. DO NO GIVE
% INITIAL POSITION LESS THAN 1
gap = 10;
step = 0;

while (gap > 0.1)
    % Changing variable so that the original J_inv is preserved and can be
    % used again and again.
    J_inv1 = J_inv;
    J_inverse = double(subs(J_inv1, [q1, q2, q3], angles));
    
    %%%% TO FIND d_x, we will create small steps in the direction of the unit
    %%%% vector from the old to desired end effector position.
    
    % Calculate the unit vector and updating it at every iteration.
    unit_vector = (desired_pos - step_pos) / ( (desired_posX - step_pos(1))^2 + (desired_posY - step_pos(2))^2 + (desired_posZ - step_pos(3))^2 )^0.5;
    
    % Calculating and updating the valus of scale_factor to speed up
    % computation.
    gap = norm(desired_pos - step_pos);
    if gap < 0.7 * max_dist
        scaling_factor = 5;
        if gap < 0.2 * max_dist
            scaling_factor = 1;
            if gap < 1
                scaling_factor = ( (desired_posX - step_pos(1))^2 + (desired_posY - step_pos(2))^2 + (desired_posZ - step_pos(3))^2 )^0.5;
            end
        end
    end
    
    % The value of the small step. CAN BE SCALED UP OR DOWN.
    step = double(unit_vector * scaling_factor);

    % changing the variable name for ease of understanding and in accordance
    % with the formula.
    d_x = step;
    
    % Using formula to calculate the change in angles due to change in
    % position of end effector.
    d_q = J_inverse * d_x;
    
    angles = double(angles + d_q');
    
    % all_angles contains angles calculated above plus the rest of the
    % angles (i.e. q4, q5,q6, q7) are all zero. This is done for the purpose of substitution. 
    all_angles = [angles, 0, 0, 0, 0];
    
    % Plotting
    T10_num = double(subs(T10, q1, all_angles(1)));
    T20_num = double(subs(T20, [q1, q2], all_angles(1:2)));
    T30_num = double(subs(T30, [q1, q2, q3], all_angles(1:3)));
    T40_num = double(subs(T40, [q1, q2, q3, q4], all_angles(1:4)));
    T50_num = double(subs(T50, [q1, q2, q3, q4, q5], all_angles(1:5)));
    T60_num = double(subs(T60, [q1, q2, q3, q4, q5, q6], all_angles(1:6)));
    T70_num = double(subs(T70, [q1, q2, q3, q4, q5, q6, q7], all_angles(1:7)));
    step_pos = T70_num(1:3, 4);
    
end

plot_da_vinci(T10_num(1:3, 4), T20_num(1:3, 4), T30_num(1:3, 4), T40_num(1:3, 4), T50_num(1:3, 4), T60_num(1:3, 4), T70_num(1:3, 4), 0);

display('Displaying the final position of the End effector for comparision. (units: mm)');
display(' ');
final_position = T70_num(1:3, 4);
disp(final_position);

display('The values of q1, a2 and q3 are (in degrees): ');
q1 = angles(1)*180/pi;
display(q1);
q2 = angles(2)*180/pi;
display(q2);
q3 = angles(3)*180/pi;
display(q3);