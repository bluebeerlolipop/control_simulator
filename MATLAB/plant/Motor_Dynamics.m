classdef Motor_Dynamics < handle

    %% MEMBERS
    properties
        l       % armlength
        dt      % time step

        Ct      % thrust coefficient
        Cq      % torque coefficient
        Gamma   % Mixer matrix
        tau     % time constant
        max_rpm % maximum rpm
        min_rpm % minimum rpm
        max_omega
        min_omega

        omega
        omega_prev
    end
    
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = Motor_Dynamics(DroneParams, MotorParams, dt)
            obj.l = DroneParams('armLength');
            obj.dt = dt;
            obj.Ct = MotorParams('thrust_coef');
            obj.Cq = MotorParams('torque_coef');
            obj.tau = MotorParams('tau');
            obj.max_rpm = MotorParams('max_rpm');
            obj.min_rpm = MotorParams('min_rpm');
            obj.max_omega = (pi/30) * obj.max_rpm;
            obj.min_omega = (pi/30) * obj.min_rpm;

            d = obj.l * sqrt(2) / 2;

            obj.Gamma = [obj.Ct         obj.Ct          obj.Ct          obj.Ct;
                        d*obj.Ct       -d*obj.Ct        -d*obj.Ct       d*obj.Ct;
                        d*obj.Ct       d*obj.Ct         -d*obj.Ct       -d*obj.Ct;
                        obj.Cq         -obj.Cq          obj.Cq          -obj.Cq];

            obj.omega = [0; 0; 0; 0];
            obj.omega_prev = [0; 0; 0; 0];
        end

        function actual_u = update(obj, u)
            omega_squared = obj.Gamma \ u;
            omega_squared_max = max(omega_squared, 0);
            omega_target = sqrt(omega_squared_max);

            % target rpm after saturation
            omega_target_sat = max(min(omega_target, obj.max_omega), obj.min_omega);

            % add time delay using time constant tau
            alpha = obj.dt / (obj.tau + obj.dt);
            obj.omega = (1 - alpha) * obj.omega_prev + alpha * omega_target_sat;
            
            % actual values after 1st order delay
            actual_u = obj.Gamma * (obj.omega.^2);

            % update previous value
            obj.omega_prev = obj.omega;
        end
    end
end