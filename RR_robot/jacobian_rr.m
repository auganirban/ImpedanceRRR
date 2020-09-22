function [jac] = jacobian_rr(q)
    global l1; global l2;
    
    % Jacobian matrix of a planar RR manipulator
    jac = [-l1*sin(q(1))-l2*sin(q(1)+q(2)), -l2*sin(q(1)+q(2));
        l1*cos(q(1))+l2*cos(q(1)+q(2)), l2*cos(q(1)+q(2))];
end

