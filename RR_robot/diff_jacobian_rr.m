function [djac] = diff_jacobian_rr(q, dq)
    global l1; global l2;
    
    % This function computes derivative of Jacobian matrix of a RR
    % manipulator.
    djac(1,1) = -l1*cos(q(1))*dq(1)-l2*cos(q(1)+q(2))*(dq(1)+dq(2));
    djac(1,2) = -l2*cos(q(1)+q(2))*(dq(1)+dq(2));
    djac(2,1) = -l1*sin(q(1))*dq(1)-l2*sin(q(1)+q(2))*(dq(1)+dq(2));
    djac(2,2) = -l2*sin(q(1)+q(2))*(dq(1)+dq(2));
end

