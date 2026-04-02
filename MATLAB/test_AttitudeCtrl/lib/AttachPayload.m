function [new_m, new_l, new_cg, new_I] = AttachPayload(drone_params, payload_params)
    %calculate new mass
    new_m = drone_params('mass') + payload_params('mass');

    % calculate new length
    new_l = drone_params('armLength');

    % calculate new cg
    new_cg = CalCG(drone_params, payload_params);

    % calculate new inertiaTensor
    drone_I = CalInertiaTensor(drone_params, new_cg);
    payload_I = CalInertiaTensor(payload_params, new_cg);
    new_I = drone_I + payload_I;
end
