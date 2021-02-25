
% Create the homogeneous transformation matrix given the four D.H
% parameters.
% Alpha and Theta [radians]
function tform = create_homogeneous_transformation(a, d, alpha, theta)
    
    alpha_degree = rad2deg(alpha);
    theta_degree = rad2deg(theta);
    
    % Rotation + translation around z
    rot_z = rotz(theta_degree);
    tform_z = rotm2tform(rot_z);
    tform_z(3,4) = d;
    %tform_z
    % Rotation + translation around x
    rot_x = rotx(alpha_degree);
    tform_x = rotm2tform(rot_x);
    tform_x(1, 4) = a;
    %tform_x
    tform = tform_z * tform_x;

end
