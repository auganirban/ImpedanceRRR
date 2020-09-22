function[state, ddq] = invdyn_impedance_rr(q, tau_control, dt, hA)
    % -------------------------------------------|
    %  Solves inverse dynamics of a 2R robot     |
    %  based on the dynamic model given in       |
    %  Siciliano's book, page 268.               |
    %  Date: May 31st., 2020                     |
    %  Author: Anirban Sinha                     |
    %--------------------------------------------|
    
    global m1; global m2; global I1; global I2; global l1; global l2;
    global lc1; global lc2; global g;
    global Im1; global Im2; global mm1; global mm2; global kr1; global kr2;

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

    % The state-space dynamic model of the RR robot
    J = jacobian_rr(q(1:2));
    ddq = M\(tau_control - C*q(3:4) - G - J'*hA);
    state = q +  dt*[q(3:4); ddq]; % state is a concatenated vector of position and velocity
end
