function plotTau(joint_angles)
    [h , ~] = size(joint_angles);
    
    for i = 1: h
        title('Joint Angles');
        
        plot(i, joint_angles(i, 1), '-b', 'linewidth', 3);
        text(10, -0.06, '\uparrow Joint 1');
        hold on
        plot(i, joint_angles(i, 2), '-g', 'linewidth', 3);
        text(25, 0.65, '\leftarrow Joint 2');
        hold on
        plot(i, joint_angles(i, 3), '-r', 'linewidth', 3);
        text(20, -0.82, '\leftarrow Joint 3');
        hold on
        plot(i, joint_angles(i, 4), '-c', 'linewidth', 7);
        text(30, 0.4, '\uparrow Joint 4');
        hold on
        plot(i, joint_angles(i, 5), '-m', 'linewidth', 7);
        text(50, 0.7, '\uparrow Joint 5');
        hold on
        plot(i, joint_angles(i, 6), '-y', 'linewidth', 3);
        text(60, 0.7, '\uparrow Joint 6');
        hold on
        plot(i, joint_angles(i, 7), '-k', 'linewidth', 3);
        text(10, 0.42, '\uparrow Joint 7');
        hold on
    end
    
end