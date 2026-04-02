classdef Control_Adapt < handle
    properties
        dt

        P_gain
        D_gain

        Ky
        Kr

        Gamma_r
        Gamma_y

        err

        ym

        r
        y

        Am
        Bm

        P
        J
    end

    methods
        function obj = Control_Adapt(sys_req, Q, dt)

            obj.P_gain = 0.2;
            obj.D_gain = 0.15;

            obj.Ky = [-obj.P_gain, -obj.D_gain, 0, 0, 0, 0; ...
                0, 0, -obj.P_gain, -obj.D_gain, 0, 0; ...
                0, 0, 0, 0, -obj.P_gain, -obj.D_gain];
            obj.Kr = diag([obj.P_gain, obj.P_gain, obj.P_gain]);

            obj.err = zeros(6,1);

            obj.ym = zeros(6,1);

            obj.r = zeros(3,1);
            obj.y = zeros(6,1);

            [obj.Am, obj.Bm] = SetRefModel(sys_req);

            obj.Gamma_r = sys_req("Gamma_r");
            obj.Gamma_y = sys_req("Gamma_y");


            obj.dt = dt;
            
            obj.P = lyap(obj.Am', Q);
        end

        function u = AttitudeCtrl(obj, currentState, currentdState, refSig)
            eular = currentState(7:9);
            deular = currentdState(7:9);

            x = [eular(1); deular(1); eular(2); deular(2); eular(3); deular(3)];

            obj.r = refSig;                     % [roll; pitch; yaw]
            dym = obj.Am*obj.ym + obj.Bm*obj.r;
            obj.ym = obj.ym + dym * obj.dt;
            obj.y = x;

            obj.err = obj.y - obj.ym;

            dKr = -obj.Gamma_r * obj.Bm' * obj.P * obj.err * obj.r';
            dKy = -obj.Gamma_y * obj.Bm' * obj.P * obj.err * obj.y';

            obj.Kr = obj.Kr + dKr * obj.dt;
            obj.Ky = obj.Ky + dKy * obj.dt;

            u = obj.Kr*obj.r + obj.Ky*obj.y;

        end

        function ym = RefState(obj)
            ym = obj.ym;
        end

    end
end
