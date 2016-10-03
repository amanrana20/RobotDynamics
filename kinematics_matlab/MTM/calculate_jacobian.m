%% This function calculates the Jacobian of the end effector in the base frame.
% Here end effector position only depends on the first 3 links, hence only
% q1, q2 and q3. Therefore the jacobian only includes those terms.

function J = calculate_jacobian(Tn0)
    syms q1 q2 q3 q4 q5 q6 q7 q7
    
    pos_EE = Tn0(1:3, 4);
    
J = [diff(pos_EE, q1), diff(pos_EE, q2), diff(pos_EE, q3)];
        
end








