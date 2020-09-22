function [x] = frd_kin_rr(q)
    global l1; global l2;
    
    % This function computes forward kinematics of RR planar manipulator
    x(1,1) = l1*cos(q(1)) + l2*cos(q(1)+q(2));
    x(2,1) = l1*sin(q(1)) + l2*sin(q(1)+q(2));
end

