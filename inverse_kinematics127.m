function [t1, t2] = inverse_kinematics(r1, r2, x, y)
    % SCARA Robot Inverse Kinematics
    % Calculates the reararm angle (t1) and forearm angle (t2) in degrees
    % based on the desired position (x, y) and arm lengths (r1, r2).
    %
    % Inputs:
    %   r1 - Reararm length [mm]
    %   r2 - Forearm length [mm]
    %   x  - Desired x-coordinate of the end effector [mm]
    %   y  - Desired y-coordinate of the end effector [mm]
    %
    % Outputs:
    %   t1 - Reararm angle [deg]
    %   t2 - Forearm angle [deg]
    
    % Ensure (x, y) is within the reachable workspace
    if y < 0 || y > 156 || x < -78 || x > 78
        error('Target position is outside the reachable workspace.');
    end

    % Calculate forearm angle t2 [deg]
    D = (x^2 + y^2 - r1^2 - r2^2) / (2 * r1 * r2);
    if abs(D) > 1
        error('Target position is not reachable.');
    end
    t2 = acosd(D);
    t2 = max(-180, min(t2, 180)); % Ensure t2 is within [-180, 180]

    % Adjust forearm angle for workspace quadrants
    if x >= 0
        t2 = -abs(t2); % First quadrant uses negative t2
    else
        t2 = abs(t2);  % Second quadrant uses positive t2
    end

    % Calculate reararm angle t1 [deg]
    if t2 < 0
        % Adjustments for negative t2
        k1 = y / x;
        k2 = abs(r2 * sind(t2)) / (r1 + abs(r2 * cosd(t2)));
        t1 = atand(k1) + atand(k2);
    else
        % Adjustments for positive t2
        k1 = y / x;
        k2 = abs(r2 * sind(t2)) / (r1 + abs(r2 * cosd(t2)));
        t1 = atand(k1) + 180 - atand(k2);
    end
    t1 = max(0, min(t1, 180)); % Ensure t1 is within [0, 180]
end
