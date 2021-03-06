function[state, ddq] = invdyn_impedance_rrr(q, tau_control, dt, hA)
    
    % -------------------------------------------|
    %  Solves inverse dynamics of a 3R robot     |
    %  based on the dynamic model given in       |
    %  Siciliano's book, page 268.               |
    %  Date: May 31st., 2020                     |
    %  Author: Anirban Sinha                     |
    %--------------------------------------------|
    
    global m1;  global m2;  global m3;
    global I1;  global I2;  global I3;
    global l1;  global l2;  global l3;
    global lc1; global lc2; global lc3;
    global Im1; global Im2; global Im3;
    global mm1; global mm2; global mm3;
    global kr1; global kr2; global kr3;
    global g;

    % Mass matrix
    M(1,1) = (m1*(lc1^2)+I1+(kr1^2)*Im1)...
        + (m2*((l1^2)+(lc2^2)+2*l1*lc2*cos(q(2)))+I2+Im2+mm2*(l1^2))...
        + (m3*(l1^2+l2^2+lc3^2+2*l1*l2*cos(q(2))+2*l1*lc3*cos(q(2)+q(3))...
        +2*l2*lc3*cos(q(3)))+I3+Im3+mm3*(l1^2+l2^2+2*l1*l2*cos(q(2))));
    M(1,2) = (m2*((lc2^2) + l1*lc2*cos(q(2))) + I2 + kr2*Im2)...
        +(m3*(l1*l2*cos(q(2))+l1*lc3*cos(q(2)+q(2))+(l2^2)+(lc3^2)+2*l2*lc3*cos(q(3)))...
        +mm3*(l1*l2*cos(q(2))+(l2^2))+I3+Im3);
    M(1,3) = m3*(lc3^3+l1*lc3*cos(q(2)+q(3))+l2*lc3*cos(q(3)))+I3+Im3*kr3;    
    M(2,1) = M(1,2);
    M(2,2) = (m2*(lc2^2) + I2 + (kr2^2)*Im2)...
        +(m3*(l2^2+lc3^2+2*l2*lc3*cos(q(3)))+I3+Im3+mm3*(l2^2));
    M(2,3) = m3*((lc3^2)+lc3*l2*cos(q(3)))+I3+Im3*kr3;
    M(3,1) = M(1,3);
    M(3,2) = M(2,3);
    M(3,3) = m3*(lc3^2)+I3+Im3*kr3^2;

    % Coriolis matrix
    % Row 1
    c111 = 0; 
    c112 = -m2*(l1*lc2*sin(q(2)))-m3*l1*(l2*sin(q(2))+lc3*sin(q(2)+q(3)))-mm3*l1*l2*sin(q(2));
    c113 = -m3*lc3*(l1*sin(q(2)+q(3))+l2*sin(q(3)));
    C(1,1) = c111+c112+c113;
    
    c121 = c112;
    c122 = -m2*l1*lc2*sin(q(2))-m3*l1*(l2*sin(q(2))+lc3*sin(q(2)+q(3)))-mm3*l1*l2*sin(q(2));
    c123 = -0.5*m3*lc3*(l1*sin(q(2)+q(3))+2*l2*sin(q(3)))-0.5*m3*l1*lc3*sin(q(2)+q(3));
    C(1,2) = c121+c122+c123;
    
    c131 = c113;
    c132 = c123;
    c133 = -m3*lc3*(l1*sin(q(2)+q(3))+l2*sin(q(3)));
    C(1,3) = c131+c132+c133;
    
    % Row 2
    c211 = -c112;
    c212 = 0;
    c213 = -0.5*m3*(l1*lc3*sin(q(2)+q(3))+2*l2*lc3*sin(q(3)))+0.5*m3*l1*lc3*sin(q(2)+q(3));
    C(2,1) = c211+c212+c213;
    
    c221 = c212;
    c222 = 0;
    c223 = -m3*l2*lc3*sin(q(3));
    C(2,2) = c221+c222+c223;
    
    c231 = c213;
    c232 = c223;
    c233 = -m3*lc3*l2*sin(q(3));
    C(2,3) = c231+c232+c233;
    
    % Row 3
    c311 = -c113;
    c312 = -0.5*m3*l1*lc3*sin(q(2)+q(3))+0.5*m3*lc3*(l1*sin(q(2)+q(3))+2*l2*sin(q(3)));
    c313 = 0;
    C(3,1) = c311+c312+c313;
    
    c321 = c312;
    c322 = m3*l2*lc3*sin(q(3));
    c323 = 0;
    C(3,2) = c321+c322+c323;
    
    c331 = c313;
    c332 = c323;
    c333 = 0;
    C(3,3) = c331+c332+c333;
    
    % Gravity vector (according to Siciliano's book)
    G(1,1) = (m1*lc1 + m2*l1 + mm2*l1)*g*cos(q(1)) + m2*lc2*g*cos(q(1)+q(2))...
        +m3*g*(l1*cos(q(1))+l2*cos(q(1)+q(2))+lc3*cos(q(1)+q(2)+q(3)))+mm3*g*(l1*cos(q(1))+l2*cos(q(1)+q(2)));
    G(2,1) = m2*lc2*g*cos(q(1)+q(2)) + m3*g*(l2*cos(q(1)+q(2))+lc3*cos(q(1)+q(2)+q(3))) + mm3*g*(l2*cos(q(1)+q(2)));
    G(3,1) = m3*g*lc3*cos(q(1)+q(2)+q(3));

    % The state-space dynamic model of the RR robot
    J = jacobian_rrr(q(1:3));
    ddq = M\(tau_control - C*q(4:6) - G - J'*[hA;0]);
    state = q +  dt*[q(4:6); ddq]; % state is a concatenated vector of position and velocity
end
