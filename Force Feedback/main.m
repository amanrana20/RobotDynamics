%%%% This file is the main file that simulates the force feedback part of
%%%% the project for course RBE 501 Robot Dynamics.
%%%% This file forst calculates the joint angles of MTM during virtual
%%%% simulation and then in a separate for loop sends those calculated
%%%% joint angles to the plotting function for simulation in MATLAB.
%%%%
%%%% TEAM MEMBERS:
%%%% 1. Aman Rana
%%%% 2. Terence Carmichael
%%%% 3. Akanksha Devkar
%%%% 4. Amit Trivedi
%%%% 5. Kiran Mohan


%% Setup
clc
clear all


%% Symbolic Variables
q1 = sym('q1', 'real');
q2 = sym('q2', 'real');
q3 = sym('q3', 'real');
q4 = sym('q4', 'real');
q5 = sym('q5', 'real');
q6 = sym('q6', 'real');
q7 = sym('q7', 'real');


%% Parameters
% Length of 1st link (m)
L1 = 0.28826;
L2 = 0.2794;
L3 = 0.3048;

% COM pos of each link wrt itself
h1 = 0.0528;
h2 = 0.0764;
h3 = 0.183;
h4 = 0.0433;
h5 = 0.063;
h6 = 0.0319;

% Masses (Kg), 
m1 = 0.09124; 
m2 = 0.64722;
m3 = 0.89273;
m4 = 0.32541;
m5 = 0.12131;
m6 = 0.05238;
m7 = 0.020;

H = 0.1506; % m
g = 9.8;  % m / sec^2

% DH Parameters
a = [0, L2, L3, 0, 0, 0, 0];
alpha = [90, 0, -90, -90, +90, -90, 0];
d = [-0.28826, 0, 0, H, 0, 0, 0];
theta = [-pi/2+q1, -pi/2+q2, pi/2+q3, q4, q5, -pi/2+q6, -pi/2];

% DH parameters of the COM of the various links
a_com = [0, h2, h3, 0, 0, 0, 0];
alpha_com = [90, 0, -90, -90, 90, -90, 0];
d_com = [-h1, 0, 0, h4, -h5, -h6, 0];
theta_com = [-pi/2+q1, -pi/2+q2, pi/2+q3, q4, q5, -pi/2+q6, -pi/2];


%% Getting the Transformation Matrices
display('Preparing...');
[t1, t2, t3, t4, t5, t6, t7, T1, T2, T3, T4, T5, T6, T7, T1_com, T2_com, T3_com, T4_com, T5_com, T6_com, T7_com] = getBasicData();


%% Calculating Jacobian
J = jacobian(T7(1:3, 4), [q1, q2, q3, q4, q5, q6, q7]);
temp = [T1(1:3, 3), T2(1:3, 3), T3(1:3, 3), T4(1:3, 3), T5(1:3, 3), T6(1:3, 3), T7(1:3, 3)];
J = [J; temp];


%% Calculating the Potential Energies of the various links
PE1 = m1 * g * T1_com(3, 4);
PE2 = m2 * g * T2_com(3, 4);
PE3 = m3 * g * T3_com(3, 4);
PE4 = m4 * g * T4_com(3, 4);
PE5 = m5 * g * T5_com(3, 4);
PE6 = m6 * g * T6_com(3, 4);
PE7 = m7 * g * T7_com(3, 4);


%% Calculating the Lagrangian
L = -(PE1 + PE2 + PE3 + PE4 + PE5 + PE6 + PE7);


%% Calculating the joint Torques
Tau1 = vpa(-diff(L, q1));
Tau2 = vpa(-diff(L, q2));
Tau3 = vpa(-diff(L, q3));
Tau4 = vpa(-diff(L, q4));
Tau5 = vpa(-diff(L, q5));
Tau6 = vpa(-diff(L, q6));
Tau7 = vpa(-diff(L, q7));

% Making the Tau vector containing the individual joint torque values
Tau = [Tau1; Tau2; Tau3; Tau4; Tau5; Tau6; Tau7];
TAU = vpa(simplify(Tau), 2);

% joint angles that are updated in every iteration
updated_angles = [0, pi/4, -pi/4, pi/6, pi/4, pi/4, pi/6];

% Calculation of numeric end effector position
EE_pos = subs(T7(1:3, 4), [q1, q2, q3, q4, q5, q6, q7], updated_angles);


% This loop runs for 70 iterations to simulate the MTM force feedback. All
% the required values are stored and then after this for loop the
% simulation loop is run.

for i = 1:70
    
    % Calculating the numeric Jacobian and Tau matrices.
    J_num = subs(J, [q1, q2, q3, q4, q5, q6, q7], updated_angles(i, :));
    TAU = subs(TAU, [q1, q2, q3, q4, q5, q6, q7], updated_angles(i, :));
    
    % Torque on the joints due to force applied by the surgeon on the EE
    F_surgeon = [0; 0; -10; 0; 0; 0];
    Tau_surgeon = transpose(J_num) * F_surgeon;

    % << Defining the opposing Torque on the joints due to force received from PSM >>
    
    % case 1: when there is no F_psm i.e. the PSM is not in contact with
    % the tissue
    F_psm = [0; 0; 0; 0; 0; 0];
    Tau_psm = transpose(J_num) * F_psm;
    
    % case 2: when the F_psm == F_surgeon. The MTM will not move as net
    % force is zero
    F_psm2 = [0; 0; 10; 0; 0; 0];
    Tau_psm2 = transpose(J_num) * F_psm2;
    
    % case 3: Excessive Force by surgeon.
    F_psm3 = [0; 0; int8((i-50)/3); 0; 0; 0];
    Tau_psm3 = transpose(J_num) * F_psm3;
    
    % << Scaling factor for certain calculations >>
    % defining the scaling factor for futher use. This is used to scale the
    % quiver length for simulation purpose.
    scaling_factor = 70;
    
    % << This part is to apply different F_psm forces as opposing forces >>
    % << F_surgeon remains the same >>
    % Here -(TAU) is for gravity compensation, and the other torques are for
    %   feedback force from PSM and input of surgeon force.
    % Tau_d has been calculated to apply to the real system, which is the
    %   control law.
    if i <= 15
        net_F = F_surgeon + F_psm;
        % < CONTROL LAW >
        Tau_d(:, i) = double(simplify(TAU + Tau_psm - Tau_surgeon));
    end
    
    if i > 30 && i <= 50
        net_F = F_surgeon + F_psm2;
        % < CONTROL LAW >
        Tau_d(:, i) = double(simplify(TAU + Tau_psm2 - Tau_surgeon));
    end
    
    if i > 50
        net_F = F_surgeon + [0; 0; 9.999999999; 0; 0; 0];
        scaling_factor = 800;
        % < CONTROL LAW >
        Tau_d(:, i) = double(simplify(TAU + Tau_psm3 - Tau_surgeon));
    end
    
    
    % Calculating the unit vector of the direction of effective force.
    if net_F(1) == 0 && net_F(2) == 0 && net_F(3) == 0
        
        % << if both the forces(F_psm and F_surgeon) are equal, the MTM will
        % not move >>
%         display('<< FORCE APPLIED BY DOCTOR == FORCE FEEDBACK FROM PSM >>')
        updated_angles(i+1, :) =  updated_angles(i, :);
        
    else
        % << reaches here when there is a net force in some direction >>
        
        % calculation of a unit vector in the direction of the net force.
        unit_direction(:, i) = (net_F(1:3))/abs(norm(net_F(1:3)))/scaling_factor;
        
        % changing the end effector position by the unit distance and
        % calculating the change in joint angles.
        EE_pos = EE_pos + unit_direction(:, i);
        delta_angle = double( transpose(J_num) * [unit_direction(:, i); 0; 0; 0] );
        
        % updating the joins angles.
        updated_angles(i+1, :) =  updated_angles(i, :) + delta_angle';
        
    end
    
end

% displaying ready after all the virtual calculations have been done and
% the system is ready for actual simulation.
display('Ready...');
figure
hold on

% << Actual Simulation >>

% Displaying the plot as simulation
for i = 1:71
        % Plotting the MTM with updated joint values
        if i > 1 && i <= 30
            plot_MTM(updated_angles(i, :), T1(1:3, 4), T2(1:3, 4), T3(1:3, 4), T4(1:3, 4), T5(1:3, 4), T6(1:3, 4), T7(1:3, 4), unit_direction(:, i-1), i, 0);
        end
        
        if i > 30 && i <= 50
            plot_MTM(updated_angles(i, :), T1(1:3, 4), T2(1:3, 4), T3(1:3, 4), T4(1:3, 4), T5(1:3, 4), T6(1:3, 4), T7(1:3, 4), unit_direction(:, i-1), i, 2);
        end
        
        if i > 50
            plot_MTM(updated_angles(i, :), T1(1:3, 4), T2(1:3, 4), T3(1:3, 4), T4(1:3, 4), T5(1:3, 4), T6(1:3, 4), T7(1:3, 4), -unit_direction(:, i-1), i, 3);
        end
        
        if i ~= 71
            clf
        end 
end