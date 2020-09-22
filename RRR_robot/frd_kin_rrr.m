function [x] = frd_kin_rrr(q)
    global l1; global l2; global l3;
    
    % This function computes forward kinematics of RRR planar manipulator
    x(1,1) = l1*cos(q(1)) + l2*cos(q(1)+q(2)) + l3*cos(q(1)+q(2)+q(3));
    x(2,1) = l1*sin(q(1)) + l2*sin(q(1)+q(2)) + l3*sin(q(1)+q(2)+q(3));
    x(3,1) = q(1)+q(2)+q(3);
end

