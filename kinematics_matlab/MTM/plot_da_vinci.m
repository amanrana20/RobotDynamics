%% Plotting a line diagram of the position and orientation of the robot.
%
% This file receives the position of joints with respect to the base of the
% robot. i.e. the first 3 elements of the last column of the transformation
% matrices of Tn0, where 1 < n < (DOF of the robot). The last arguement is
% isIntialPosition which can be 0 for NO and 1 for YES. Therefore to plot
% the initial position, the DaVinci file sends a 1 and to display the
% desired orientation 0 is sent.


function [] = plot_da_vinci(T10_pos, T20_pos, T30_pos, T40_pos, T50_pos, T60_pos, T70_pos, isInitialPosition)
    % Matrix X containing X cooredinates, Y containing y coordinates and Z
    % containing z coordinates.
    X = int16([0, T10_pos(1), T20_pos(1), T30_pos(1), T40_pos(1), T50_pos(1), T60_pos(1), T70_pos(1)]);
    Y = [0, T10_pos(2), T20_pos(2), T30_pos(2), T40_pos(2), T50_pos(2), T60_pos(2), T70_pos(2)];
    Z = [0, T10_pos(3), T20_pos(3), T30_pos(3), T40_pos(3), T50_pos(3), T60_pos(3), T70_pos(3)];
    
    hold on
    if isInitialPosition == 1
        line(X, Y, Z, 'linewidth', 2);
    else
        % plotting the line diagram
        line(X, Y, Z, 'Color', 'g',   'linewidth', 2);
    end
    hold on
    
    
    for i = 1:8
        plot3(X(i), Y(i), Z(i), 'r*', 'linewidth', 2);
        hold on
    end
    
    hold on
    xlabel('X axis (mm)');
    ylabel('y axis (mm)');
    zlabel('z axis (mm)');
    view(30, 30)
    hold on
    
end