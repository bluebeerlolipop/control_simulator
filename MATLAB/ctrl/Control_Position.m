% [x_err, y_err]' -> PID -> [theta_des, phi_des]'

classdef Control_Position < handle
    properties
        dt
        m
        g
        max_angle

        % x_des
        x_err
        x_err_prev
        x_err_sum

        % y_des
        y_err
        y_err_prev
        y_err_sum

        % z_des(altitude)
        z_err
        z_err_prev
        z_err_sum

        % PID gain
        kP_x
        kI_x
        kD_x

        kP_y
        kI_y
        kD_y

        kP_z
        kI_z
        kD_z
    end
    %% METHODS
    methods
        function obj=Control_Position(gains, droneParams, dt)
            obj.g = 9.81;
            obj.dt = dt;
            obj.m = droneParams('mass');
            obj.max_angle = droneParams("max_angle")*(pi/180);

            obj.x_err = 0;
            obj.x_err_prev = 0;
            obj.x_err_sum = 0;

            obj.y_err = 0;
            obj.y_err_prev = 0;
            obj.y_err_sum = 0;

            obj.z_err = 0;
            obj.z_err_prev = 0;
            obj.z_err_sum = 0;

            obj.kP_x = gains('P_x');
            obj.kI_x = gains('I_x');
            obj.kD_x = gains('D_x');

            obj.kP_y = gains('P_y');
            obj.kI_y = gains('I_y');
            obj.kD_y = gains('D_y');

            obj.kP_z = gains('P_z');
            obj.kI_z = gains('I_z');
            obj.kD_z = gains('D_z');

        end

        %% Position Controller
        function [u, cmd] =PositionCtrl(obj, currentState, refSig)

            current_r = currentState(1:3);
            %current_dr = currentState(4:6);

            x_des = refSig(1);
            y_des = refSig(2);
            z_des = refSig(3);
            %psi_cmd = refSig(4);

            obj.x_err = x_des - current_r(1);
            obj.y_err = y_des - current_r(2);
            obj.z_err = z_des - current_r(3);

            u = 0;
            %cmd = zeros(3,1);
            cmd = zeros(2,1);

            u(1) = obj.m * obj.g - (obj.kP_z * obj.z_err + ...
                                    obj.kI_z * obj.z_err_sum + ...
                                    obj.kD_z * (obj.z_err - obj.z_err_prev)/obj.dt);
            
            obj.z_err_sum = obj.z_err_sum + obj.z_err*obj.dt;
            obj.z_err_prev = obj.z_err;

            % phi
            phi_raw =   (obj.kP_y * obj.y_err + ...
                        obj.kI_y * obj.y_err_sum + ...
                        obj.kD_y * (obj.y_err - obj.y_err_prev)/obj.dt);

            cmd(1) = max(min(phi_raw, obj.max_angle), -obj.max_angle);

            obj.y_err_sum = obj.y_err_sum + obj.y_err*obj.dt;
            obj.y_err_prev = obj.y_err;

            % theta
            theta_raw = - (obj.kP_x * obj.x_err + ...
                        obj.kI_x * obj.x_err_sum + ...
                        obj.kD_x * (obj.x_err - obj.x_err_prev)/obj.dt);

            cmd(2) = max(min(theta_raw, obj.max_angle), -obj.max_angle);

            obj.x_err_sum = obj.x_err_sum + obj.x_err*obj.dt;
            obj.x_err_prev = obj.x_err;

            % psi
            %cmd(3) = psi_cmd;


        end
    end
end
            

            