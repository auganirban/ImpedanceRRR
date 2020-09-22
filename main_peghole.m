clc;
clear;
close all;

%------------------------------------------------------------------------|
% This code implements a basic impedance control scheme with a RRR       |
% manipulator trying to insert a peg into a hole.                        |
% Author: Anirban Sinha. Date: June 21st., 2020                          |
%------------------------------------------------------------------------|

global m1;  global m2;  global m3;
global I1;  global I2;  global I3;
global l1;  global l2;  global l3;
global lc1; global lc2; global lc3;
global Im1; global Im2; global Im3;
global mm1; global mm2; global mm3;
global kr1; global kr2; global kr3;
global g;
% % % global filename; global iter; 
global phase;
% % % filename = "rrr_peg_insert_impdeance_controller.gif";
phase = 1;

% Step-1: Define the RR Robot parameters (book Siciliano example 7.2)
m1 = 50.0;  m2 = 50.0;  m3 = 25;
I1 = 10.0;  I2 = 10.0;  I3 = 5;
l1 = 1.0;   l2 = 1.0;   l3 = 0.5;
lc1 = 0.5;  lc2 = 0.5;  lc3 = 0.25;
Im1 = 0.01; Im2 = 0.01; Im3 = 0.005;
kr1 = 100;  kr2 = 100;  kr3 = 100;
mm1 = 5.0;  mm2 = 5.0;  mm3 = 2.5;
g = 9.81;

% Step-2: Define the initial and desired end-effector positions, 
% velocities and accelerations with respect to the base frame.
xe = [1.2012;1.9761;0.3927]; dxe = [0;0;0];
xd = [1.7000;1.4659;0]; dxd=[0;0;0]; ddxd=[0;0;0];
store_ee_pos = [xe];

% Step-3: Compute the current joint state vector [q;dq] corresponding to xe
curr_state = [5.5*pi/12;-pi/6;-pi/6;0;0;0];

% Step-4: Define the initial force measurement to zeros as at time 0 there
% is no measurement available.
hA = [0;0]; hA_store = [hA];

% Step-5: Define the elastic wall.
obs1.height = 1.4; obs1.width = 0.2; obs1.origin = [1.3,0];
obs2.height = 0.5; obs2.width = 0.2; obs2.origin = [1.3,1.53];
xr1 = [1.3;1.53]; xr2 = [1.3;1.4];
fig_handle = figure();

% % % patch([obs1.origin(1),obs1.origin(1)+obs1.width,obs1.origin(1)+obs1.width,obs1.origin(1)],...
% % %     [obs1.origin(2),obs1.origin(2),obs1.origin(2)+obs1.height,obs1.origin(2)+obs1.height],...
% % %     [0.5,0.5,0.5]);
% % % hold on;
% % % patch([obs2.origin(1),obs2.origin(1)+obs2.width,obs2.origin(1)+obs2.width,obs2.origin(1)],...
% % %     [obs2.origin(2),obs2.origin(2),obs2.origin(2)+obs2.height,obs2.origin(2)+obs2.height],...
% % %     [0.5,0.5,0.5]);
% % % plot(xd(1),xd(2),"o","markerfacecolor","r");

patch([xr1(1),xr1(1)+0.3,xr1(1)+0.3,xr1(1)],[xr1(2),xr1(2),2.0,2.0],[0.5,0.5,0.5]);
hold on;
patch([xr2(1),xr2(1)+0.3,xr2(1)+0.3,xr2(1)],[-0.5,-0.5,xr2(2),xr2(2)],[0.5,0.5,0.5]);
plot(xd(1),xd(2),"o","markerfacecolor","r");

% Step-6: Define the simulation time and integration time step
tf = 2.5;
dt = 0.001;
tm = 0:dt:tf;
% % % rng("default");

% Step-7: Simulate and visualize
for iter = 2:length(tm)
    
    % find the impedance control input
    % Case-1: noisy force measurement
    hA_noise = normrnd(0,0.5,[2,1]);
    u = impedance_control(xe,dxe,xd,dxd,ddxd,hA+hA_noise,curr_state);
% % %     % Case-2: perfect force measurement
% % %     u = impedance_control(xe,dxe,xd,dxd,ddxd,hA,curr_state);
    
    % apply the input to the robot and get the new state
    % Case-1: noisy u input
    u_noise = normrnd(0,100,[size(u,1),1]);
    [new_state,ddq] = invdyn_impedance_rrr(curr_state,u+u_noise,dt,hA);
% % %     % Case-2: perfect u input
% % %     [new_state,ddq] = invdyn_impedance_rrr(curr_state,u,dt,hA);
    
    % using new state, find end-effector states
    % new_state = new_state + [normrnd(0,0.0005,[size(new_state,1)/2,1]);zeros(size(new_state,1)/2,1)];
    xe = frd_kin_rrr(new_state(1:3));
    dxe = jacobian_rrr(new_state(1:3))*new_state(4:6);
    % ddxe = jacobian_rr(new_state(1:3))*ddq + diff_jacobian_rr(new_state(1:3),new_state(4:6))*new_state(4:6);
    
    % find the contact force if any
    hA = get_ee_force(xe,xr1,xr2,obs1,obs2);
    hA_store = [hA_store,hA];
% % %     hA = hA + normrnd(0,0.0005,[2,1]);
    
    % Visualize the robot motion under impedance control
    visualize_motion_rrr(new_state(1:3),fig_handle);
    
    % override the current state with the new state
    curr_state = new_state;
    store_ee_pos = [store_ee_pos, xe];
    
    if mod(iter,10) == 0
        fprintf("iter: %d\n",iter);
    end
end

% Step-8: Plot the end-effector force history along x axis
figure()
plot(tm,hA_store(1,:));
xlabel("time [s]")
ylabel("F_x [N]")

figure()
plot(tm,hA_store(2,:));
xlabel("time [s]")
ylabel("F_y [N]")

figure()
plot(tm,store_ee_pos(1,:));
xlabel("time [s]")
ylabel("x [m]")

figure()
plot(tm,store_ee_pos(2,:));
xlabel("time [s]")
ylabel("y [m]")
