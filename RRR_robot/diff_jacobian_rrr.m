function [dJ] = diff_jacobian_rrr(q, dq)
    global l1; global l2; global l3;
    
    % This function computes derivative of Jacobian matrix of a RRR
    % manipulator.
    dJ(1,1) = -l1*cos(q(1))*dq(1)-l2*cos(q(1)+q(2))*(dq(1)+dq(2))-l3*cos(q(1)+q(2)+q(3))*(dq(1)+dq(2)+dq(3));
    dJ(1,2) = -l2*cos(q(1)+q(2))*(dq(1)+dq(2))-l3*cos(q(1)+q(2)+q(3))*(dq(1)+dq(2)+dq(3));
    dJ(1,3) = -l3*cos(q(1)+q(2)+q(3))*(dq(1)+dq(2)+dq(3));
    
    dJ(2,1) = -l1*sin(q(1))*dq(1)-l2*sin(q(1)+q(2))*(dq(1)+dq(2))-l3*sin(q(1)+q(2)+q(3))*(dq(1)+dq(2)+dq(3));
    dJ(2,2) = -l2*sin(q(1)+q(2))*(dq(1)+dq(2))-l3*sin(q(1)+q(2)+q(3))*(dq(1)+dq(2)+dq(3));
    dJ(2,3) = -l3*sin(q(1)+q(2)+q(3))*(dq(1)+dq(2)+dq(3));
    
    dJ(3,1) = 0;
    dJ(3,2) = 0;
    dJ(3,3) = 0;
end

