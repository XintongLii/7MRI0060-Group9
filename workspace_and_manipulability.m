function workspace_and_manipulability()
    % Parameters for the workspace visualization
    l1 = 107; % Length of the first arm
    l2 = 87;  % Length of the second arm
    theta1 = 0:0.05:pi; % All possible theta1 values
    theta2 = -pi:0.05:pi; % All possible theta2 values
    [THETA1, THETA2] = meshgrid(theta1, theta2);
    X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2); % x coordinates
    Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2); % y coordinates

    % Plot the workspace
    figure;
    plot(X(:), Y(:), 'r.');
    axis equal;
    xlabel('X (mm)', 'fontsize', 10);
    ylabel('Y (mm)', 'fontsize', 10);
    title('Workspace of the Robot Arm', 'fontsize', 10);

    % Parameters for manipulability simulation
    r1 = 107; % Length of first link
    r2 = 87;  % Length of second link
    t1 = 0;   % Initial angle of joint 1
    t2 = 0;   % Initial angle of joint 2
    x_d = [150; 100]; % Desired end-effector position
    damping_factor = 0.01; % Damping factor for DLS
    step_limit = 0.1; % Maximum step size for target clamping
    gain = 0.3; % Gain for Jacobian transpose method
    tolerance = 1e-3; % Position error tolerance
    max_iterations = 1000; % Maximum iterations
    dt = 0.01; % Time step for plotting manipulability
    ellipsoid_scale = 20; % Scaling factor for ellipsoids
    manipulability_values = zeros(max_iterations, 1);

    % Figure setup for manipulability
    figure;
    hold on;
    axis equal;
    xlim([-250, 250]);
    ylim([-250, 250]);
    xlabel('X (mm)');
    ylabel('Y (mm)');
    title('Robot Arm Motion and Manipulability Ellipsoids');

    % Simulation loop
    for i = 1:max_iterations
        % Forward kinematics
        T = forward_kinematics(r1, r2, t1, t2);
        x = T(1, 4);
        y = T(2, 4);
        current_position = [x; y];
        error = x_d - current_position;

        % Target clamping
        if norm(error) > step_limit
            error = step_limit * (error / norm(error));
        end

        % Compute Jacobian
        J = ik_jacobian(r1, r2, t1, t2);
        manipulability = sqrt(det(J * J'));
        manipulability_values(i) = manipulability;

        % Calculate joint updates
        inv_J_damped = (J' * J + damping_factor^2 * eye(size(J, 2))) \ J';
        delta_q = inv_J_damped * error;

        % Update joint angles
        t1 = t1 + delta_q(1);
        t2 = t2 + delta_q(2);

        % Plot the robot arm position
        plot([0, r1*cos(t1), r1*cos(t1) + r2*cos(t1 + t2)], ...
             [0, r1*sin(t1), r1*sin(t1) + r2*sin(t1 + t2)], '-o');

        % Plot manipulability ellipsoid
        [U, S, ~] = svd(J);
        theta = linspace(0, 2*pi, 100);
        ellipse_x = ellipsoid_scale * S(1, 1) * cos(theta);
        ellipse_y = ellipsoid_scale * S(2, 2) * sin(theta);
        ellipse = U * [ellipse_x; ellipse_y];
        plot(ellipse(1, :) + x, ellipse(2, :) + y, 'm');

        pause(0.01); % Visualization pause

        % Check if position reached
        if norm(error) < tolerance
            disp('Desired position reached!');
            break;
        end
    end

    % Plot manipulability trajectory
    figure;
    plot(manipulability_values(1:i));
    xlabel('Iteration');
    ylabel('Manipulability');
    title('Manipulability across the trajectory');
end

% Forward kinematics function
function T = forward_kinematics(r1, r2, t1, t2)
    T1 = [cos(t1), -sin(t1), 0, r1*cos(t1);
          sin(t1),  cos(t1), 0, r1*sin(t1);
          0,        0,       1, 0;
          0,        0,       0, 1];
    T2 = [cos(t2), -sin(t2), 0, r2*cos(t2);
          sin(t2),  cos(t2), 0, r2*sin(t2);
          0,        0,       1, 0;
          0,        0,       0, 1];
    T = T1 * T2;
end

% Jacobian function
function J = ik_jacobian(r1, r2, t1, t2)
    J11 = -r1*sin(t1) - r2*sin(t1 + t2);
    J12 = -r2*sin(t1 + t2);
    J21 =  r1*cos(t1) + r2*cos(t1 + t2);
    J22 =  r2*cos(t1 + t2);
    J = [J11, J12; J21, J22];
end
