function x = CalCG(droneParam, payloadParam)
    drone_m = droneParam('mass');
    drone_cg = droneParam('cg');
    payload_m = payloadParam('mass');
    payload_cg = payloadParam('cg');


    x = (drone_m.*drone_cg + payload_m.*payload_cg)/(drone_m + payload_m);
end