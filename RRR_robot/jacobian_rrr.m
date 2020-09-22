function [J] = jacobian_rrr(q)
    global l1; global l2; global l3;
    
    % Jacobian matrix of a planar RRR manipulator
    
    J(1,1) = -l1*sin(q(1))-l2*sin(q(1)+q(2))-l3*sin(q(1)+q(2)+q(3));
    J(1,2) = -l2*sin(q(1)+q(2))-l3*sin(q(1)+q(2)+q(3));
    J(1,3) = -l3*sin(q(1)+q(2)+q(3));
    
    J(2,1) = l1*cos(q(1))+l2*cos(q(1)+q(2))+l3*cos(q(1)+q(2)+q(3));
    J(2,2) = l2*cos(q(1)+q(2))+l3*cos(q(1)+q(2)+q(3));
    J(2,3) = l3*cos(q(1)+q(2)+q(3));
    
    J(3,1) = 1;
    J(3,2) = 1;
    J(3,3) = 1;
    
end

