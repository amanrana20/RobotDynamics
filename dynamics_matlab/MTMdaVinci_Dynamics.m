function MTMdaVinci_Dynamics

%% Setup
clc
clear all
close all

%% Initializing
syms q1 q2 q3 q4 q5 q6 q7 real ;
syms q1d q2d q3d q4d q5d q6d q7d real;
syms q1dd q2dd q3dd q4dd q5dd q6dd q7dd real;

syms mLa mLfa mLh mUser real;
L_arm = 279.4; % mm
L_forearm = 304.8; %mm
h = 150.6; % mm

%%
% Angular velocities, wrt the world frame
% axis, we can simply add the angular velocities
w1 = [0 0 q1d].'; %q1 is about Z.
w2 = [q2d 0 q1d].'; %q2 is about X.
w3 = [q2d+q3d 0 q1d].';%q3 is about X.
w4 = [q2d+q3d 0 q1d+q4d].'; %q4 is about Z.
w5 = [q2d+q3d+q5d 0 q1d+q4d].'; %q5 is about X.
w6 = [q2d+q3d+q5d 0 q1d+q4d+q6d].'; %q6 is about Z.
w7 = [q2d+q3d+q5d q7d q1d+q4d+q6d].'; %q7 is about Y.



%%
% We do the same for the angular accelerations
alpha1 = [0 0 q1d].'; %q1 is about Z.
alpha2 = [q2d 0 q1d].'; %q2 is about X.
alpha3 = [q2d+q3d 0 q1d].';%q3 is about X.
alpha4 = [q2d+q3d 0 q1d+q4d].'; %q4 is about Z.
alpha5 = [q2d+q3d+q5d 0 q1d+q4d].'; %q5 is about X.
alpha6 = [q2d+q3d+q5d 0 q1d+q4d+q6d].'; %q6 is about Z.
alpha7 = [q2d+q3d+q5d q7d q1d+q4d+q6d].'; %q7 is about Y.


g= 9.8;% Meters per second*second


% DH Parameters
a = [0, L_arm, L_forearm, 0, 0, 0, 0];
alpha = [90, 0, -90, -90, +90, -90, 0];
d = [-100, 0, 0, h, 0, 0, 0];
theta = [-pi/2 + q1, -pi/2+q2, pi/2+q3, q4, q5, -pi/2+q6, pi/2+q7];

%%
% Assumming that lengths of mass positions, as each mass is at the center of each link
link_lengths = [279.4, 304.8, 150.6];
center_lengths = link_lengths/2;
masses = [mLa mLfa mLh mUser];
gv=[0;-g;0]; %m/s^2

%%
% Assuming that the center of mass is at the center.
r1c1 = [center_lengths(1);0;0];
r2c1 = [-link_lengths(1)+center_lengths(1);0;0];
r12  = [link_lengths(1);0;0];

r2c2 = [center_lengths(2);0;0];
r3c2 = [-link_lengths(2)+center_lengths(2);0;0];
r23  = [link_lengths(2);0;0];

r3c3 = [center_lengths(3);0;0];
r4c3 = [-link_lengths(3)+center_lengths(3);0;0];
r34  = [link_lengths(3);0;0];



%% Calculations
% Transformation Matrices
T10 = get_transformation_matrix(theta(1), d(1), a(1), alpha(1));
T10 = simplify(T10);
% display(T10);
T10_numeric = subs(T10, q1, 0);

T21 = get_transformation_matrix(theta(2), d(2), a(2), alpha(2));
T21 = simplify(T21);
T20 = T10 * T21;
% display(T20);
T20_numeric = subs(T20, [q1, q2], [0, 0]);

T32 = get_transformation_matrix(theta(3), d(3), a(3), alpha(3));
T32 = simplify(T32);
T30 = T20 * T32;
% display(T30);
T30_numeric = subs(T30, [q1, q2, q3], [0, 0, 0]);

T43 = get_transformation_matrix(theta(4), d(4), a(4), alpha(4));
T43 = simplify(T43);
T40 = T30 * T43;
% display(T40);
T40_numeric = subs(T40, [q1, q2, q3, q4], [0, 0, 0, 0]);

T54 = get_transformation_matrix(theta(5), d(5), a(5), alpha(5));
T54 = simplify(T54);
T50 = T40 * T54;
% display(T50);
T50_numeric = subs(T50, [q1, q2, q3, q4, q5], [0, 0, 0, 0, 0]);

T65 = get_transformation_matrix(theta(6), d(6), a(6), alpha(6));
T65 = simplify(T65);
T60 = T50 * T65;
% display(T60);
T60_numeric = subs(T60, [q1, q2, q3, q4, q5, q6], [0, 0, 0, 0, 0, 0]);

T76 = get_transformation_matrix(theta(7), d(7), a(7), alpha(7));
T76 = simplify(T76);
T70 = T60 * T76;
% display(T70);
T70_numeric = double(subs(T70, [q1, q2, q3, q4, q5, q6, q7], [0, 0, 0, 0, 0, 0, 0]));
display(T70_numeric(1:3, 4));

%Position of User
P0U = T70(1:3,4);

% Individual Rotation matrices
R01 = T10(1:3,1:3);%Impacts Link Arm
R12 = T21(1:3,1:3);%Impacts Link Arm
R23 = T32(1:3,1:3);%Impacts Link FA
R34 = T43(1:3,1:3);%Impacts Link H
R45 = T54(1:3,1:3);%Impacts Link H
R56 = T65(1:3,1:3);%Impacts Link H
R67 = T76(1:3,1:3);%Impacts Link H
R7U = eye(3,3);%User to tip link.

%% Forward Recursion
% Link 1
%
% Summing the accelerations that effect particular links.
a_c1 = cross(alpha1,r1c1)+cross(w1,cross(w1,r1c1)) + ...
    cross(alpha2,r1c1)+cross(w2,cross(w2,r1c1));
a_e1 = cross(alpha1,r12)+cross(w1,cross(w1,r12)) + ...
    cross(alpha2,r12)+cross(w2,cross(w2,r12));
g_1  = R01.'*gv + R12.'*gv;


% Link 2
a_c2 = simplify(R23.'*a_e1+cross(alpha3,r2c2)+cross(w3,cross(w3,r2c2)));
a_e2 = simplify(R23.'*a_e1+cross(alpha3,r23)+cross(w3,cross(w3,r23)));
g_2  = simplify(R23.'*g_1);

%%
% Link 3
a_c3 = simplify(R34.'*a_e2+cross(alpha4,r3c3)+cross(w4,cross(w4,r3c3)))...
    + simplify(R45.'*a_e2+cross(alpha5,r3c3)+cross(w5,cross(w5,r3c3)))...
    + simplify(R56.'*a_e2+cross(alpha6,r3c3)+cross(w6,cross(w6,r3c3)))...
    + simplify(R67.'*a_e2+cross(alpha7,r3c3)+cross(w7,cross(w7,r3c3)));
a_e3 = simplify(R34.'*a_e2+cross(alpha4,r34)+cross(w4,cross(w4,r34)))...
    + simplify(R45.'*a_e2+cross(alpha5,r34)+cross(w5,cross(w5,r34)))...
    + simplify(R56.'*a_e2+cross(alpha6,r34)+cross(w6,cross(w6,r34)))...
    + simplify(R67.'*a_e2+cross(alpha7,r34)+cross(w7,cross(w7,r34)));

g_3  = simplify(R34.'*g_2)+ simplify(R45.'*g_2) + simplify(R56.'*g_2) + ...
    simplify(R67.'*g_2);
g_4  = simplify(R7U.'*g_3);

%% Backward Recursion
%Torque and force for link 3
f_3b = (masses(4))*(a_e3 - g_3);
f_3a = (masses(3))*(a_c3 - g_3);
f_3 = f_3a + f_3b;

Tau_l3_el = -cross(f_3,r3c3) + cross(-f_3b,r3c3); 

%Torque and force for link 2
R74 =  R67*R56*R45*R34; %Rotations from link h.
f_2 = R74*f_3+ (masses(2)*(a_c2 - g_2));
Tau_l2_el = R74*Tau_l3_el-cross(f_2,r2c2)+cross(R74*f_3,r3c2);

%Torque and force for link 1
f_1 = R23*f_2+(masses(1)*(a_c1 - g_1));
Tau_l1_el = R23*Tau_l2_el-cross(f_1,r1c1)+cross(R23*f_2,r2c1);

Torques = [Tau_l1_el(3) Tau_l2_el(3) Tau_l3_el(3)]'

%%
% Collecting terms for M matrix
% M11 = simplify(Torques(1) - subs(Torques(1),ddq1,0)) /ddq1;
% M12 = simplify(Torques(1) - subs(Torques(1),ddq2,0)) /ddq2;
% M13 = simplify(Torques(1) - subs(Torques(1),ddq3,0)) /ddq3;
% 
% M21 = simplify(Torques(2) - subs(Torques(2),ddq1,0)) /ddq1;
% M22 = simplify(Torques(2) - subs(Torques(2),ddq2,0)) /ddq2;
% M23 = simplify(Torques(2) - subs(Torques(2),ddq3,0)) /ddq3;
% 
% M31 = simplify(Torques(3) - subs(Torques(3),ddq1,0)) /ddq1;
% M32 = simplify(Torques(3) - subs(Torques(3),ddq2,0)) /ddq2;
% M33 = simplify(Torques(3) - subs(Torques(3),ddq3,0)) /ddq3;


%%
% The intertia matrix is as follows
% 
% M_el = [M11 M12 M13;
%         M21 M22 M23;
%         M31 M32 M33];
    
%%
% Collecting terms for G
% 
% G11 = subs(Torques(1), [dq1 dq2 dq3 ddq1 ddq2 ddq3], [0 0 0 0 0 0]);
% G21 = subs(Torques(2), [dq1 dq2 dq3 ddq1 ddq2 ddq3], [0 0 0 0 0 0]);
% G31 = subs(Torques(3), [dq1 dq2 dq3 ddq1 ddq2 ddq3], [0 0 0 0 0 0]);
% 
% G_el = simplify([G11;
%         G21;
%         G31]);

%%
% Collecting terms for C
% 
% C11 = Torques(1) - (M_el(1,:) * [ddq1 ddq2 ddq3].' + G11);
% C21 = Torques(2) - (M_el(2,:) * [ddq1 ddq2 ddq3].' + G21);
% C31 = Torques(3) - (M_el(3,:) * [ddq1 ddq2 ddq3].' + G31);
% 
% C_el = simplify([C11;
%         C21;
%         C31]);

%%
%
 
% pretty(M_el)
% pretty(C_el)
% pretty(G_el)

%%
% Moment of truth. Voila
% 
% diff_M = simplify(M - M_el)
% 
% diff_C = simplify(C - C_el)
% 
% diff_G = simplify(G - G_el)


end

