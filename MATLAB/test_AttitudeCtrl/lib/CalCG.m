function x = CalCG(droneParam, x_drone, payloadParam, x_payload)
    m_drone = droneParam('mass');
    m_payload = payloadParam('mass');

    x = (m_drone.*x_drone + m_payload.*x_payload)/(m_drone + m_payload);
end