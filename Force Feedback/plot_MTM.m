function [] = plot_MTM(angles, T1, T2, T3, T4, T5, T6, T7, direction, i, color_desc)
    %%%% This function is used for plotting the visual results of the
    %%%% simulation.

    syms q1 q2 q3 q4 q5 q6 q7
    
    if i <= 50
        direction = direction * 5;
        
    else
        direction = direction * (i - 50) * 5;
    end
    
    disp(direction);
    
    % Calculating the numeric position of each joint based on the passed
    % joint angles.
    T1_pos = subs(T1, [q1, q2, q3, q4, q5, q6, q7], angles);
    T2_pos = subs(T2, [q1, q2, q3, q4, q5, q6, q7], angles);
    T3_pos = subs(T3, [q1, q2, q3, q4, q5, q6, q7], angles);
    T4_pos = subs(T4, [q1, q2, q3, q4, q5, q6, q7], angles);
    T5_pos = subs(T5, [q1, q2, q3, q4, q5, q6, q7], angles);
    T6_pos = subs(T6, [q1, q2, q3, q4, q5, q6, q7], angles);
    T7_pos = subs(T7, [q1, q2, q3, q4, q5, q6, q7], angles);
    
    % Caollecting all the x, y and z coordinates in different arrays for
    % plotting in each iteration.
    X = [0, T1_pos(1), T2_pos(1), T3_pos(1), T4_pos(1), T5_pos(1), T6_pos(1), T7_pos(1)];
    Y = [0, T1_pos(2), T2_pos(2), T3_pos(2), T4_pos(2), T5_pos(2), T6_pos(2), T7_pos(2)];
    Z = [0, T1_pos(3), T2_pos(3), T3_pos(3), T4_pos(3), T5_pos(3), T6_pos(3), T7_pos(3)];
    
    % defining the axis starting and ending point.
    axis equal
    axis([-0.8 0 -0.8 0 -0.8 0]);
    
    % plotting the line diagram
    line(X, Y, Z, 'linewidth', 2);
    hold on
    
    
    % plotting based on different situiations.
    if color_desc == 0
        color = 'g';
        text1 = 'F surgeon > F psm';
        text2 = 'F surgeon: [0; 0; -10]N ; F psm : [0; 0; 0]N';
        quiver3(T7_pos(1), T7_pos(2), T7_pos(3), direction(1), direction(2), direction(3), 'Color', color);
        hold on
    end
    
    if color_desc == 2
        color = 'm';
        text1 = 'F surgeon == F psm ';
        text2 = 'F surgeon: [0; 0; -10]N ; F psm : [0; 0; 10]N';
        plot3(T7_pos(1), T7_pos(2), T7_pos(3), 'Color', color, 'linewidth', 5);
        hold on
    end
    
    if color_desc == 3
        color = 'r';
        text1 = 'Excessive Force by surgeon';
        force_above_10N = int8((i-50)/3);
        text2 = sprintf('F surgeon: [0; 0; %d]N ; F psm : [0; 0; %d]N', [-10-force_above_10N, 10+force_above_10N]);
        quiver3(T7_pos(1), T7_pos(2), T7_pos(3), direction(1), direction(2), direction(3), 'Color', color);
        hold on
    end
    
    
    legend(text1, text2,  'Location', 'NorthWest');
    
    % Labelling the axes
    xlabel('X axis (m)');
    ylabel('y axis (m)');
    zlabel('z axis (m)');
    
    view(90, 0)
    hold on
    
    drawnow;
    hold on
    
end