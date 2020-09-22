clc;
clear;
close all;

global l1; global l2;

% Input
q_init = [0; 0]; q_final = [4*pi/6; -pi/3]; tf = 1;
l1 = 1.0;   l2 = 1.0;

% Compute trajectory in jointspace
[q, dq, ddq] = trapezoidal_vel_js(q_init, q_final, tf);

% Map the jointspace trajecotry into the taskspace trajectories
figure(3)
[xe, dxe, ddxe] = trajectory_ts(q, dq, ddq);
for i = 1:size(q,2)
    visualize_robot_motion(q(:,i));
end
