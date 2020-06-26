function R_EB = angle2dcm1(yaw,pitch,roll)
% A function used to formulate the rotation matrix from body frame to earth
% inertial frame
theta = pitch;
phi = roll;
psi = yaw;



R_EB = [cos(theta)*cos(psi) sin(theta)*sin(phi)*cos(psi)-cos(phi)*sin(psi) cos(psi)*sin(theta)*cos(phi) + sin(phi)*sin(psi);
        sin(psi)*cos(theta) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
        -sin(theta)         cos(theta)*sin(phi)                            cos(phi)*cos(theta)                             ];
    
R_EB = R_EB';   
    
end