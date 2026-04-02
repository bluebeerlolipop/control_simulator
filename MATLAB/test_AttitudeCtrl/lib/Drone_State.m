classdef Drone_State < handle

%% MEMBERS
    properties
        g
        t
        dt
        tf

        m
        l
        I
        cg

        x           % [X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r]'
        r           % [X, Y, Z]'
        dr          % [dX, dY, dZ]'
        euler       % [pihi, theta, psi]'
        w           % [p, q, r]'

        dx

        T           % T_sum
        M           % [M1, M2, M3]'
    end

%% METHODS
    methods
        %% CONSTRUCTOR
        function obj = Drone_State(params, initStates, simTime, dt)
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = dt;
            obj.tf = simTime;

            obj.m = params('mass');
            obj.l = params('armLength');
            obj.I = params('inertiaTensor');
            obj.cg = params('cg');

            obj.x = initStates;
            obj.r = obj.x(1:3);     % x,y,z
            obj.dr = obj.x(4:6);    % x,y,z(dot)
            obj.euler = obj.x(7:9); % phi, theta, psi
            obj.w = obj.x(10:12);   % p, q, r

            obj.dx = zeros(12,1);   % state_dot term
            obj.T = 0;
            obj.M = [0;0;0]; % initial M 

        end

        function state = GetState(obj)
            state = obj.x;
        end

        function dstate = GetdState(obj)
            dstate = obj.dx;
        end

        function obj = EvalEOM(obj)
            bRi = RPY2Rot(obj.euler);
            R = bRi';

            obj.dx(1:3) = obj.dr; %x(4:6)
            obj.dx(4:6) = 1 / obj.m * ([0; 0; obj.m*obj.g] + R * obj.T * [0; 0; -1]);

            phi = obj.euler(1); theta = obj.euler(2);
            obj.dx(7:9) = [ 1 sin(phi)*tan(theta) cos(phi)*tan(theta);
                            0 cos(phi)            -sin(phi);
                            0 sin(phi)*sec(theta) cos(phi)*sec(theta)] * obj.w;

            M_cg = cross(-obj.cg, [0; 0; -obj.T]);
            obj.dx(10:12) = (obj.I)\((obj.M + M_cg) - cross(obj.w, obj.I * obj.w));

        end

        function obj = UpdateState(obj, u)
            obj.T = u(1);
            obj.M = u(2:4);
            
            obj.t = obj.t + obj.dt;

            obj.EvalEOM();
            obj.x = obj.x + obj.dx.*obj.dt; % euler method

            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12); % sensor disturbance 추가시 rand 사용해 sensor 구성
        end

        function obj = DetachPayload(obj, drone_params)
            obj.m = drone_params('mass');
            obj.l = drone_params('armLength');
            obj.I = drone_params('inertiaTensor');
            obj.cg = drone_params('cg');
        end

    end
end