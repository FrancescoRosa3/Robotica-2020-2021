% Perform transformation.
% A: matrix that contains for each row a homogenous transformation matrix
% start_frame;
% end_frame;
function [tform, quaternion, axang, r, p, y] = perfom_transformation(A, start_frame, end_frame)
    
    % Compute the Homogeneous transformation matrix from start frame to
    % end frame
    tform = A((start_frame*4)-3:(start_frame*4), :);
    for i = start_frame+1:end_frame
        tform = tform * A((i*4)-3:(i*4), :);
    end
    
    % Extract the rotation matrix
    rotm = tform2rotm(tform);
    
    % Compute the quaternion
    quaternion = rotm2quat(rotm); % w x y z
    
    % Compute RPY angles from quaternion
    %eul = rotm2eul(rotm, "XYZ");
    [y, p, r] = quat2angle(quaternion, 'ZYX'); % ZYX
    % Compute the axis angle rotation from rotation matrix
    axang = rotm2axang(rotm); % x y z Theta
    
end
