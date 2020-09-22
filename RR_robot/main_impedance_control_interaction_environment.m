clc;
clear;
close all;

%------------------------------------------------------------------------|
% This code implements a basic impedance control scheme with a RR        |
% manipulator whose end-effector tip is touching an elastic wall.        |
% Author: Anirban Sinha. Date: June 14th., 2020                          |
%------------------------------------------------------------------------|

global m1; global m2; global I1; global I2; global l1; global l2;
global lc1; global lc2; global g;
global Im1; global Im2; global mm1; global mm2; global kr1; global kr2;

% Step-1: Define the RR Robot parameters (book Siciliano example 7.2)
m1 = 50.0; m2 = 50.0; I1 = 10.0;  I2 = 10.0;
l1 = 1.0;  l2 = 1.00; lc1 = 0.5;  lc2 = 0.5;  g = 9.81;
mm1 = 5.0; mm2 = 5.0; Im1 = 0.01; Im2 = 0.01; kr1 = 100; kr2 = 100;

% Step-2: Define the initial and desired end-effector positions, 
% velocities and accelerations with respect to the base frame.
xe = [1;0]; dxe = [0;0];
xd = [1.1;0.1]; dxd=[0;0]; ddxd=[0;0];

% Step-3: Compute the current joint state vector [q;dq] corresponding to xe
curr_state = [1.0472;-2.0944;0;0];

% Step-4: Define the initial force measurement to zeros as at time 0 there
% is no measurement available.
hA = [0;0]; hA_store = [hA];

% Step-5: Define the elastic wall.
xr = [1.0;0.0];
figure()
patch([1.0,1.3,1.3,1.0],[-2,-2,2,2],[0.5,0.5,0.5]);
hold on;

% Step-6: Define the simulation time and integration time step
tf = 3; 
dt = 0.001;
tm = 0:dt:tf;

% Step-7: Simulate and visualize
for i = 2:length(tm)
    % find the impedance control input
    u = impedance_control(xe,dxe,xd,dxd,ddxd,hA,curr_state);
    
    % apply the input to the robot and get the new state
    [new_state,ddq] = invdyn_impedance_rr(curr_state,u,dt,hA);
    
    % using new state, find end-effector states
    xe = frd_kin_rr(new_state(1:2));
    dxe = jacobian_rr(new_state(1:2))*new_state(3:4);
    % ddxe = jacobian_rr(new_state(1:2))*ddq + diff_jacobian_rr(new_state(1:2),new_state(3:4))*new_state(3:4);
    
    % find the contact force if any
    hA = get_ee_force(xe,xr);
    hA_store = [hA_store,hA];
    
    % Visualize the robot motion under impedance control
    visualize_robot_motion(new_state(1:2));
    
    % override the current state with the new state
    curr_state = new_state;
end

% Step-8: Plot the end-effector force history along x axis
figure()
plot(tm,hA_store(1,:));
xlabel("time [s]")
ylabel("F_x [N]")