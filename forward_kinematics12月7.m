function [x, y] = forward_kinematics(r1, r2, t1, t2)
    % SCARA Robot Forward Kinematics
    % Calculates the end effector position (x, y) based on joint angles (t1, t2)
    % and arm lengths (r1, r2).
    %
    % Inputs:
    %   r1 - Reararm length [mm]
    %   r2 - Forearm length [mm]
    %   t1 - Reararm angle [deg]
    %   t2 - Forearm angle [deg]
    %
    % Outputs:
    %   x - X-coordinate of the end effector [mm]
    %   y - Y-coordinate of the end effector [mm]

    % Homogeneous transformation matrices
    A01 = [cosd(t1) -sind(t1) 0 r1 * cosd(t1);
           sind(t1) cosd(t1)  0 r1 * sind(t1);
           0        0         1 0;
           0        0         0 1];

    A12 = [cosd(t2) -sind(t2) 0 r2 * cosd(t2);
           sind(t2) cosd(t2)  0 r2 * sind(t2);
           0        0         1 0;
           0        0         0 1];

    % Final transformation matrix
    T = A01 * A12;

    % Extract x and y positions
    x = T(1, 4); % X-coordinate
    y = T(2, 4); % Y-coordinate
end
