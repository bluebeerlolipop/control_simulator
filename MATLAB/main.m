% main_refactored.m
clc;
clear;
close all
addpath('./lib');
addpath('./ctrl');
addpath('./plant');
%% DEFINE
R2D = 180/pi;
D2R = pi/180;
%% Simulation time
simulationTime = 10;
dt_inner = 0.01;
dt_outer = 0.02;

ratio = round(dt_outer/dt_inner);

%% INITIAL PARAMS
% Drone Params
drone1_m = 1.25;
drone1_l = 0.265;
drone1_cg = [0, 0, 0]';
drone1_I = [0.0232, 0, 0; ...
            0, 0.0232, 0; ...
            0, 0, 0.0468];
drone1_max_angle = 50.0;
drone1_params = containers.Map({'mass', 'armLength', 'cg', 'inertiaTensor', 'max_angle'}, ...
    {drone1_m, drone1_l, drone1_cg, drone1_I, drone1_max_angle});

drone1_initStates = [0, 0, -6, ...       % X, Y, Z
                     0, 0, 0, ...        % dX, dY, dZ
                     0, 0, 0, ...        % phi, theta, psi
                     0, 0, 0]';          % p, q, r

rotor1_params = containers.Map({'tau', 'thrust_coef', 'torque_coef', 'max_rpm', 'min_rpm'}, ...
    {0.03, 1.3e-05, 1.8e-07, 8500, 1000});

% Payload PARAMS
payload1_m = 0.2;
payload1_cg = [0.05, 0.0, 0.07]';
payload1_I = [8.3333e-05, 0, 0; ...
              0, 8.3333e-05, 0; ...
              0, 0, 8.3333e-05];

payload1_params = containers.Map({'mass', 'cg', 'inertiaTensor'}, ...
    {payload1_m, payload1_cg, payload1_I});

% Attach payload
[sys_m, sys_l, sys_cg, sys_I, sys_max_angle] = AttachPayload(drone1_params, payload1_params);

sys_params = containers.Map({'mass', 'armLength', 'cg', 'inertiaTensor', 'max_angle'}, ...
    {sys_m, sys_l, sys_cg, sys_I, sys_max_angle});

%% Position Controller Gain
pos1_gains = containers.Map(...
    {'P_x', 'I_x', 'D_x', ...
    'P_y', 'I_y', 'D_y', ...
    'P_z', 'I_z', 'D_z'}, ...
    {0.1, 0.0, 0.17, ...
    0.1, 0.0, 0.17, ...
    1.0, 0.02, 2.0});

%% Attitude Controller Gain
%PID Settings
drone1_gains = containers.Map(...
    {'P_phi', 'I_phi', 'D_phi', ...
    'P_theta', 'I_theta', 'D_theta', ...
    'P_psi', 'I_psi', 'D_psi'}, ...
    {3.0, 0.0, 0.15, ...
    3.0, 0.0, 0.15, ...
    3.0, 0.0, 0.15});

% LQR Settings
% state: x,y,z,xdot,ydot,zdot,phi,theta,psi,p,q,r
drone1_q = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
drone1_r = [1, 1, 1, 1]; %T,M1,M2,M3

% MRAC(attitude control) Settings
drone1_ref = containers.Map(...
    {'P_gain', 'D_gain', 'Gamma_r', 'Gamma_y', 'Gamma_d', 'NatFreq', 'damping'}, ...
    {1.0, 0.15, 1.0, 1.0, 0.5, 10, 0.707});
Q = 200.*diag([1, 0.001, 1, 0.001, 1, 0.001]);

%% Generate .mat file
numStep = simulationTime/dt_inner;
stateHistory = zeros(numStep+1, length(drone1_initStates)+6);
stateHistory(1, :) = [drone1_initStates', zeros(1, 6)];

%% Command signal
pos_cmd = [0.0, 0.0, -6];
psi_cmd = 0.0 * D2R;
commandSig(1) = pos_cmd(1); % x
commandSig(2) = pos_cmd(2); % y
commandSig(3) = pos_cmd(3); % z
commandSig(4) = psi_cmd;    % psi

%% Initialize System & Controller
% 1. import drone dynamics
drone1 = Drone_State(sys_params, drone1_initStates, simulationTime, dt_inner);
drone1_rotor = Motor_Dynamics(sys_params, rotor1_params, dt_inner);
% 2. import position controller(use dt_outer)
controller_pos = Control_Position(pos1_gains, sys_params, dt_outer);
% 3. import attitude controller(use dt_inner)
controller1 = Control_PID(drone1_gains, dt_inner);
%controller2 = Control_LQR(drone1_q, drone1_r, drone1_params, commandSig);
controller3 = Control_Adapt(drone1_ref, Q, dt_inner);

%% SIMULATION LOOP
isPayloadAttached = true;

for i = 1:(simulationTime/dt_inner)
    % Generate scenario(droping payload)
    if i*dt_inner >= simulationTime/2 && isPayloadAttached
        drone1.DetachPayload(drone1_params);
        isPayloadAttached = false;
    end

    drone1_state = drone1.GetState();
    drone1_dstate = drone1.GetdState();

    % Position control
    if mod(i, ratio) == 1
        [u_pos, cmd_prev] = controller_pos.PositionCtrl(drone1_state, pos_cmd);
    end
    cmd = [cmd_prev;psi_cmd];

    % Attitude control
    %u_attitude = controller1.AttitudeCtrl(drone1_state, cmd);
    u_attitude = controller3.AttitudeCtrl(drone1_state, drone1_dstate, cmd);
    u = [u_pos;u_attitude];

    % Mixer
    u_actual = drone1_rotor.update(u);
    drone1.UpdateState(u_actual);

    % Update & Save state
    drone1.UpdateState(u_actual);
    drone1_state = drone1.GetState();
    ym = controller3.RefState();
    stateHistory(i+1, :) = [drone1_state; ym];

    if (drone1_state(3) >= 0)
        msgbox('Crashed!!', 'Error', 'error');
        break;
    end
end

save('stateHistory.mat', 'stateHistory');
plot_sim;
