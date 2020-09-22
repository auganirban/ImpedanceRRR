function [u] = impedance_control(xe,dxe,x_des,dx_des,ddx_des,hA,q)
    global m1; global m2; global I1; global I2; global l1; global l2;
    global lc1; global lc2; global g;
    global Im1; global Im2; global mm1; global mm2; global kr1; global kr2;
    
    % Define the parameters of impedance controller
    Md = diag([100, 100]); 
    Kd = diag([500, 500]); 
    Kp = diag([2500, 2500]);

    % Mass matrix
    M(1,1) = m1*(lc1^2) + m2*((l1^2)+(lc2^2)+2*l1*lc2*cos(q(2))) + I1 + I2 + (kr1^2)*Im1 + Im2 + mm2*(l1^2);
    M(1,2) = m2*((lc2^2) + l1*lc2*cos(q(2))) + I2 + kr2*Im2;
    M(2,1) = M(1,2);
    M(2,2) = m2*(lc2^2) + I2 + (kr2^2)*Im2;

    % Coriolis matrix
    C(1,1) = -m2*l1*lc2*sin(q(2))*q(4);
    C(1,2) = -m2*l1*lc2*sin(q(2))*(q(3)+q(4));
    C(2,1) = m2*l1*lc2*sin(q(2))*q(3);
    C(2,2) = 0;
    
    % Gravity vector (according to Siciliano's book)
    G(1,1) = (m1*lc1 + m2*l1 + mm2*l1)*g*cos(q(1)) + m2*lc2*g*cos(q(1)+q(2));
    G(2,1) = m2*lc2*g*cos(q(1)+q(2));
    
    % The following impedance control law is from Siciliano's book using
    % equations 9.30 and 9.31.
    J = jacobian_rr(q(1:2));
    dJ = diff_jacobian_rr(q(1:2),q(3:4));
    y = (Md*J)\(Md*ddx_des + Kd*(dx_des - dxe) + Kp*(x_des - xe) - Md*dJ*q(3:4) - hA);
    u = M*y + C*q(3:4) + G + J'*hA;
    
% % %     % For debugging purpose (check if eq 9.32 is always satisfied)
% % %     ddq = M\(u - C*q(3:4) - G - J'*hA);
% % %     ddxe = J*ddq + dJ*q(3:4);
% % %     lhs = Md*(ddx_des-ddxe) + Kd*(dx_des-dxe) + Kp*(x_des-xe);
% % %     rhs = hA;
end
