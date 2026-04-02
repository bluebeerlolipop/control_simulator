% main_refactored.m
clc;
clear;
close all
addpath('./lib');
addpath('./ctrl');
%% DEFINE
R2D = 180/pi;
D2R = pi/180;
%% Simulation time
simulationTime = 10;
dt = 0.01;

%% INITIAL PARAMS
% Drone Params
drone1_m = 1.25;
drone1_l = 0.265;
drone1_cg = [0, 0, 0]';
drone1_I = [0.0232, 0, 0; ...
            0, 0.0232, 0; ...
            0, 0, 0.0468];
drone1_params = containers.Map({'mass', 'armLength', 'cg', 'inertiaTensor'}, ...
    {drone1_m, drone1_l, drone1_cg, drone1_I});
drone1_initStates = [0, 0, -6, ...       % X, Y, Z
                     0, 0, 0, ...        % dX, dY, dZ
                     0, 0, 0, ...        % phi, theta, psi
                     0, 0, 0]';          % p, q, r

drone1_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';
% Payload PARAMS
payload1_m = 0.2;
payload1_cg = [0.05, 0.0, 0.07]';
payload1_I = [8.3333e-05, 0, 0; ...
              0, 8.3333e-05, 0; ...
              0, 0, 8.3333e-05];

payload1_params = containers.Map({'mass', 'cg', 'inertiaTensor'}, ...
    {payload1_m, payload1_cg, payload1_I});

% Attach payload
[sys_m, sys_l, sys_cg, sys_I] = AttachPayload(drone1_params, payload1_params);

sys_params = containers.Map({'mass', 'armLength', 'inertiaTensor', 'cg'}, ...
    {sys_m, sys_l, sys_I, sys_cg});

%% Attitude Controller Gain
%PID Gain(optional when you use PID controller)
drone1_gains = containers.Map(...
    {'P_phi', 'I_phi', 'D_phi', ...
    'P_theta', 'I_theta', 'D_theta', ...
    'P_psi', 'I_psi', 'D_psi', ...
    'P_zdot', 'I_zdot', 'D_zdot'}, ...
    {0.2, 0.0, 0.15, ...
    1.0, 0.0, 0.15, ...
    0.2, 0.0, 0.15, ...
    10.0, 0.01, 0.2});

% LQR gain(optional when you use LQR controller)
drone1_q = [1, 1, 1, 1, 1, 1000, 0.001, 0.001, 1, 1, 1, 1]; % x,y,z,xdot,ydot,zdot,phi,theta,psi,p,q,r
drone1_r = [1, 1, 1, 1]; %T,M1,M2,M3

%% Adaptive Controller(attitude control) Settings
drone1_req = containers.Map({'Gamma_r', 'Gamma_y', 'NatFreq', 'damping'}, ...
    {1.0, 1.0, 15, 0.707});
Q = 100.*[1, 0, 0, 0, 0, 0; ...
     0, 0.02, 0, 0, 0, 0; ...
     0, 0, 1, 0, 0, 0; ...
     0, 0, 0, 0.02, 0, 0; ...
     0, 0, 0, 0, 1, 0; ...
     0, 0, 0, 0, 0, 0.02];

%% Generate .mat file
numStep = simulationTime/dt;
stateHistory_test = zeros(numStep+1, length(drone1_initStates)+6);
stateHistory_test(1, :) = [drone1_initStates', zeros(1, 6)];

%% command signal
commandSig(1) = 0.0 * D2R; % phi
commandSig(2) = 0.0 * D2R; % theta
commandSig(3) = 0.0 * D2R; % psi
commandSig(4) = 0.0; % z_dot

cmd = commandSig(1:3)';

%% 객체 생성(초기화)
% 1. import drone dynamics
drone1 = Drone_State(sys_params, drone1_initStates, simulationTime, dt);

% 2. import attitude controller
controller1 = Control_PID_test(drone1_gains, sys_params, dt);
%controller2 = Control_LQR(drone1_q, drone1_r, drone1_params, commandSig);
controller3 = Control_Adapt(drone1_req, Q, dt);

%% SIMULATION LOOP
isPayloadAttached = true;

for i = 1:simulationTime/dt
    % check the payload is detached
    if i*dt >= simulationTime/2 && isPayloadAttached
        drone1.DetachPayload(drone1_params);
        isPayloadAttached = false;
    end

    drone1_state = drone1.GetState();
    drone1_dstate = drone1.GetdState();
    %u_eular = controller3.AttitudeCtrl(drone1_state, drone1_dstate, cmd);
    u_z = controller1.AttitudeCtrl(drone1_state, commandSig);
    %u = [u_z(1);u_eular];
    ym = controller3.RefState();
    drone1.UpdateState(u_z);
    drone1_state = drone1.GetState();
    stateHistory_test(i+1, :) = [drone1_state; ym];

    if (drone1_state(3) >= 0)
        msgbox('Crashed!!', 'Error', 'error');
        break;
    end
end

save('stateHistory_test.mat', 'stateHistory_test');
plot_test;