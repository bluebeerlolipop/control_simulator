% main_refactored.m
clc;
clear;
close all
total_time_start = tic;

addpath('./lib');
addpath('./ctrl');
addpath('./plant');
%% DEFINE
R2D = 180/pi;
D2R = pi/180;
%% Simulation time
simulationTime = 5;
dt_inner = 0.001;
dt_outer = 0.02;

ratio = round(dt_outer/dt_inner);

%% INITIAL PARAMS
drone1_params = containers.Map({'mass', 'armLength', 'max_angle', 'Ixx', 'Iyy', 'Izz'}, ...
    {1.25, 0.265, 10.0, 0.0232, 0.0232, 0.0468});
rotor1_params = containers.Map({'tau', 'thrust_coef', 'torque_coef', 'max_rpm', 'min_rpm'}, ...
    {0.04, 1.19e-05, 1.8e-07, 8500, 1000});
drone1_initStates = [0, 0, -6, ...       % X, Y, Z
    0, 0, 0, ...                        % dX, dY, dZ
    0, 0, 0, ...                        % phi, theta, psi
    0, 0, 0]';                          % p, q, r

drone1_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';

%% Position Controller Gain
pos1_gains = containers.Map(...
    {'P_x', 'I_x', 'D_x', ...
    'P_y', 'I_y', 'D_y', ...
    'P_z', 'I_z', 'D_z'}, ...
    {0.1, 0.0, 0.17, ...
    0.1, 0.0, 0.17, ...
    1.0, 0.0, 2.0});

%% Attitude Controller Gain
drone1_gains = containers.Map(...
    {'P_phi', 'I_phi', 'D_phi', ...
    'P_theta', 'I_theta', 'D_theta', ...
    'P_psi', 'I_psi', 'D_psi'}, ...
    {0.2, 0.0, 0.15, ...
    0.2, 0.0, 0.15, ...
    0.2, 0.0, 0.15});
drone1_q = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]; % x,y,z,xdot,ydot,zdot,phi,theta,psi,p,q,r
drone1_r = [1, 1, 1, 1]; %T,M1,M2,M3

%% Adaptive Controller(attitude control) Settings
drone1_req = containers.Map({'Gamma_r', 'Gamma_y', 'NatFreq', 'damping'}, ...
    {1.0, 1.0, 15, 0.707});
Q = 100 .* [1, 0, 0, 0, 0, 0; ...
     0, 0.012, 0, 0, 0, 0; ...
     0, 0, 1, 0, 0, 0; ...
     0, 0, 0, 0.012, 0, 0; ...
     0, 0, 0, 0, 1, 0; ...
     0, 0, 0, 0, 0, 0.012];

%% Generate .mat file
numStep = simulationTime/dt_inner;
stateHistory = zeros(numStep+1, length(drone1_initStates));
stateHistory(1, :) = drone1_initStates';

%% command signal
pos_cmd = [0.0, 0.0, -5];
psi_cmd = 0.0 * D2R;
commandSig(1) = pos_cmd(1); % x
commandSig(2) = pos_cmd(2); % y
commandSig(3) = pos_cmd(3); % z
commandSig(4) = psi_cmd; % psi

%% 객체 생성(초기화)
% 1. import drone dynamics
drone1 = Drone_State(drone1_params, drone1_initStates, simulationTime, dt_inner);
drone1_rotor = Motor_Dynamics(drone1_params, rotor1_params, dt_inner);
% 2. import position controller
controller_pos = Control_Position(pos1_gains, drone1_params, dt_outer);
% 3. import attitude controller
%controller1 = Control_PID(drone1_gains, dt);
%controller2 = Control_LQR(drone1_q, drone1_r, drone1_params, commandSig);
controller3 = Control_Adapt(drone1_req, Q, dt_inner);

%% SIMULATION LOOP
% for i = 1:simulationTime/dt
%     %%% PID control
%     % drone1_state = drone1.GetState();
%     % [u_pos, cmd] = controller_pos.PositionCtrl(drone1_state, commandSig);
%     % u_control = controller1.AttitudeCtrl(drone1_state, cmd);
%     % u(1) = u_pos;           % thrust
%     % u(2) = u_control(1);    % M1
%     % u(3) = u_control(2);    % M2
%     % u(4) = u_control(3);    % M3
%     % u = u(:);               % column vector로 변환
%     % u_actual = drone1_rotor.update(u);
%     % drone1.UpdateState(u_actual);
%     % drone1_state = drone1.GetState();
%     % stateHistory(i+1, :) = drone1_state;
% 
%     %%% LQR control
%     % u = controller2.AttitudeCtrl(drone1_state, commandSig);
%     %u_actual = drone1_rotor.update(u);
%     % drone1.UpdateState(u_actual);
%     % drone1_state = drone1.GetState();
%     % stateHistory(i+1, :) = drone1_state;
% 
%     %%% Adaptive control
%     drone1_state = drone1.GetState();
%     drone1_dstate = drone1.GetdState();
%     [u_pos, cmd] = controller_pos.PositionCtrl(drone1_state, commandSig);
%     u_control = controller3.AttitudeCtrl(drone1_state, drone1_dstate, cmd);
%     u(1) = u_pos;           % thrust
%     u(2) = u_control(1);    % M1
%     u(3) = u_control(2);    % M2
%     u(4) = u_control(3);    % M3
%     u = u(:);               % column vector로 변환
%     %u_actual = drone1_rotor.update(u);
%     drone1.UpdateState(u);
%     drone1_state = drone1.GetState();
%     stateHistory(i+1, :) = drone1_state;
% 
% 
%     if (drone1_state(3) >= 0)
%         msgbox('Crashed!!', 'Error', 'error');
%         break;
%     end
% end

cmd = [0;0;0];
u_pos = drone1_params("mass") * 9.81;
for i = 1:(simulationTime/dt_inner)
    drone1_state = drone1.GetState();
    drone1_dstate = drone1.GetdState();

    if mod(i, ratio) == 1
        [u_pos, cmd_prev] = controller_pos.PositionCtrl(drone1_state, pos_cmd);
    end
    cmd = [cmd_prev;psi_cmd];
    u_control = controller3.AttitudeCtrl(drone1_state, drone1_dstate, cmd);

    u(1) = u_pos;
    u(2) = u_control(1);
    u(3) = u_control(2);
    u(4) = u_control(3);
    u = u(:);

    drone1.UpdateState(u);
    stateHistory(i+1, :) = drone1_state;

    if (drone1_state(3) >= 0)
        msgbox('Crashed!!', 'Error', 'error');
        break;
    end
end

save('stateHistory.mat', 'stateHistory');

elapsed_time = toc(total_time_start);
fprintf("\n계산 시간: %.3f초\n", elapsed_time);
plot_sim;