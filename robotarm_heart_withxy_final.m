close all;
clear;
clc;

% Parameters for robot arm
r1 = 107; % Length of first link (in mm)
r2 = 87;  % Length of second link (in mm)
t1 = 0;   % Initial angle of joint 1 (in radians)
t2 = -pi/4; % Initial angle of joint 2 (in radians)

% Define rectangle dimensions
rect_width = 156;  % Width of the rectangle
rect_height = 156; % Height of the rectangle
rect_x = -78;      % Left edge at -78
rect_y = 20;       % Bottom edge at 20

% Adjust rectangle to new coordinates
rect_x_right = 78; % Right edge at 78
rect_y_top = 176;  % Top edge at 176
rect_width = rect_x_right - rect_x; % Recalculate width
rect_height = rect_y_top - rect_y; % Recalculate height

% Heart shape parameters
t = linspace(0, 2 * pi, 1000); % Parameter for heart shape
heart_x = 6 * sin(t).^3; % X-coordinates of the heart
heart_y = 5 * cos(t) - 2 * cos(2 * t) - cos(3 * t) - cos(4 * t); % Y-coordinates of the heart
heart_x = heart_x * 7; % Scale the heart
heart_y = heart_y * 7;

% Center the heart shape in the rectangle
heart_x = heart_x + (rect_x + rect_width / 2);
heart_y = heart_y + (rect_y + rect_height / 2);

% Control parameters
damping_factor = 0.01; % Damping factor for DLS
step_limit = 5; % Maximum step size for target clamping (in mm)
gain = 0.5; % Gain for Jacobian transpose method
tolerance = 1; % Position error tolerance (in mm)
max_iterations = 1000; % Maximum number of iterations
dt = 0.01; % Time step for visualization

% Initialize figure for visualization
figure;
hold on;
axis equal;
xlim([-200, 200]);
ylim([-200, 200]);
xlabel('X (mm)');
ylabel('Y (mm)');
title('Robot Arm Drawing Heart Shape in Workspace');

% Plot the complete workspace of the robot arm
[theta1, theta2] = meshgrid(0:0.05:pi, -pi:0.05:pi);
workspace_x = r1 * cos(theta1) + r2 * cos(theta1 + theta2);
workspace_y = r1 * sin(theta1) + r2 * sin(theta1 + theta2);
plot(workspace_x(:), workspace_y(:), 'r.', 'MarkerSize', 1, 'DisplayName', 'Workspace');

% Draw checkered rectangle
square_size = 12; % Size of each square in the checkered pattern
for i = 0:(rect_width / square_size - 1)
    for j = 0:(rect_height / square_size - 1)
        square_x = rect_x + i * square_size;
        square_y = rect_y + j * square_size;
        if mod(i + j, 2) == 0
            fill([square_x, square_x + square_size, square_x + square_size, square_x], ...
                 [square_y, square_y, square_y + square_size, square_y + square_size], 'b', 'EdgeColor', 'none');
        else
            fill([square_x, square_x + square_size, square_x + square_size, square_x], ...
                 [square_y, square_y, square_y + square_size, square_y + square_size], 'w', 'EdgeColor', 'none');
        end
    end
end
rectangle('Position', [rect_x, rect_y, rect_width, rect_height], 'EdgeColor', 'k', 'LineWidth', 1.5);

% Initialize arrays to store end-effector trajectory and joint angles
trajectory_x = [];
trajectory_y = [];
angles_t1 = [];
angles_t2 = [];
x_positions = [];
y_positions = [];

% Main loop to draw the heart shape
for point_idx = 1:length(heart_x)
    % Desired position for this step
    x_d = [heart_x(point_idx); heart_y(point_idx)];

    % Initialize iteration variables
    for i = 1:max_iterations
        % Forward kinematics to calculate current end-effector position
        T = forward_kinematics(r1, r2, t1, t2);
        x = T(1, 4);
        y = T(2, 4);
        current_position = [x; y];

        % Calculate position error
        error = x_d - current_position;

        % Target clamping: Limit the step size to avoid large jumps
        if norm(error) > step_limit
            error = step_limit * (error / norm(error));
        end

        % Compute Jacobian for the current joint angles
        J = ik_jacobian(r1, r2, t1, t2);

        % Damped Least Squares (DLS) method for stable inverse calculation
        inv_J_damped = (J' * J + damping_factor^2 * eye(size(J, 2))) \ J';
        delta_q_dls = inv_J_damped * error;

        % Update joint angles using calculated change
        t1 = t1 + delta_q_dls(1);
        t2 = t2 + delta_q_dls(2);

        % Check if the end-effector is within the desired tolerance
        if norm(error) < tolerance
            break;
        end
    end

    % Store the current end-effector position
    trajectory_x = [trajectory_x, x];
    trajectory_y = [trajectory_y, y];
    angles_t1 = [angles_t1, rad2deg(t1)]; % Convert to degrees
    angles_t2 = [angles_t2, rad2deg(t2)]; % Convert to degrees
    x_positions = [x_positions, x];
    y_positions = [y_positions, y];

    % Plot the current position of the robot arm
    plot([0, r1*cos(t1), r1*cos(t1) + r2*cos(t1 + t2)], ...
         [0, r1*sin(t1), r1*sin(t1) + r2*sin(t1 + t2)], '-o', 'LineWidth', 1.5);

    % Plot the trajectory of the end-effector
    plot(trajectory_x, trajectory_y, 'y-', 'LineWidth', 1);

    % Display current angles and end-effector position in the figure
    text(-190, -150, sprintf('End-Effector: (%.2f, %.2f) mm\nt1: %.2f°\nt2: %.2f°', x, y, rad2deg(t1), rad2deg(t2)), ...
         'FontSize', 10, 'BackgroundColor', 'w');

    pause(dt); % Pause for visualization
end

% Add legend to the plot
legend('Workspace', '156x156 Region', 'End-Effector Trajectory', 'Location', 'southeast');

% Plot joint angles and (x, y) coordinates over time
figure;

% Joint 1 Angle Variation
subplot(4, 1, 1);
plot(angles_t1, 'b-', 'LineWidth', 1.5);
xlabel('Step');
ylabel('Joint 1 Angle (°)');
title('Joint 1 Angle Variation');
grid on;

% Joint 2 Angle Variation
subplot(4, 1, 2);
plot(angles_t2, 'g-', 'LineWidth', 1.5);
xlabel('Step');
ylabel('Joint 2 Angle (°)');
title('Joint 2 Angle Variation');
grid on;

% X Coordinate Variation
subplot(4, 1, 3);
plot(x_positions, 'r-', 'LineWidth', 1.5);
xlabel('Step');
ylabel('X Position (mm)');
title('X Coordinate Variation');
grid on;

% Y Coordinate Variation
subplot(4, 1, 4);
plot(y_positions, 'm-', 'LineWidth', 1.5);
xlabel('Step');
ylabel('Y Position (mm)');
title('Y Coordinate Variation');
grid on;

%% Function Definitions

% Forward kinematics function
function T = forward_kinematics(r1, r2, t1, t2)
    % Compute transformation matrices
    T1 = [cos(t1), -sin(t1), 0, r1*cos(t1);
          sin(t1),  cos(t1), 0, r1*sin(t1);
          0,        0,       1, 0;
          0,        0,       0, 1];
    T2 = [cos(t2), -sin(t2), 0, r2*cos(t2);
          sin(t2),  cos(t2), 0, r2*sin(t2);
          0,        0,       1, 0;
          0,        0,       0, 1];
    T = T1 * T2; % Combined transformation
end

% Jacobian calculation function
function J = ik_jacobian(r1, r2, t1, t2)
    % Compute the Jacobian for a 2-link planar manipulator
    J11 = -r1*sin(t1) - r2*sin(t1 + t2);
    J12 = -r2*sin(t1 + t2);
    J21 =  r1*cos(t1) + r2*cos(t1 + t2);
    J22 =  r2*cos(t1 + t2);
    J = [J11, J12; J21, J22];
end
