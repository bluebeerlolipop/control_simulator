function I = CalInertiaTensor(params, original_CG, new_CG)
    m = params('mass');
    I_original = [params('Ixx'), 0, 0; ...
         0, params('Iyy'), 0; ...
         0, 0, params('Izz')];
    dx = original_CG(1) - new_CG(1);
    dy = original_CG(2) - new_CG(2);
    dz = original_CG(3) - new_CG(3);

    I_shift = m.*[(dy^2+dz^2), -dx*dy, -dx*dz; ...
                  -dy*dx, (dx^2+dz^2), -dy*dz; ...
                  -dz*dx, -dz*dy, (dx^2+dy^2)];
    I = I_original + I_shift;
end
