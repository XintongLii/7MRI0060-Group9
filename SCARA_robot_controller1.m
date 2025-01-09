function SCARA_robot_controller()
    % SCARA Robot Controller
    % Combines Forward Kinematics, Inverse Kinematics, and MATLAB-Arduino Communication
    % to control and visualize the SCARA robot's motion in real-time.
    % @author         Alejandro Granados
    % @organisation   King's College London
    % @module         Medical Robotics Hardware Development
    % @year           2023

    close all
    clear all

    % Global variables for GUI, serial communication, and real-time plotting
    global s hInput1 hInput2 hPlot hFig hTimer c y1 y2
    r1 = 107; % Length of the first arm (in mm)
    r2 = 87;  % Length of the second arm (in mm)

    %% Set up serial communication
    serialports = seriallist(); % List available serial ports
    disp('Available Serial Ports:');
    disp(serialports);

    s = serialport(serialports(1), 115200); % Connect to the first serial port
    configureTerminator(s, "CR/LF");
    s.UserData = struct("Data", [], "Count", 1);

    %% Create GUI
    hFig = figure('Name', 'SCARA Robot Controller', 'NumberTitle', 'off');

    % Input fields for end-effector position
    hInput1 = uicontrol('Style', 'edit', 'Position', [20, 20, 100, 25], 'String', 'Enter X');
    hInput2 = uicontrol('Style', 'edit', 'Position', [120, 20, 100, 25], 'String', 'Enter Y');

    % Button to send command to Arduino
    uicontrol('Style', 'pushbutton', 'String', 'Send', ...
        'Position', [20, 50, 100, 25], 'Callback', @sendCommand);

    % Plot area for real-time visualization
    hPlot = axes('Position', [0.2, 0.6, 0.6, 0.3]);
    xlabel(hPlot, 'X (mm)');
    ylabel(hPlot, 'Y (mm)');
    title(hPlot, 'End-Effector Trajectory');
    grid on;

    % Variables for real-time plotting
    c = [];
    y1 = [];
    y2 = [];

    % Timer for continuously reading data
    hTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, 'TimerFcn', @readDataTimer);
    start(hTimer);

    % Handle GUI closure
    hFig.CloseRequestFcn = @closeGUI;

    %% Forward Kinematics function
    function [x, y] = forward(theta1_deg, theta2_deg)
        % SCARA Robot Forward Kinematics
        % Calculates the end effector position (x, y) based on joint angles
        L1 = r1; % Length of the first arm
        L2 = r2; % Length of the second arm

        % Convert angles to radians
        theta1 = deg2rad(theta1_deg);
        theta2 = deg2rad(theta2_deg);

        % Calculate x and y positions
        x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
        y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    end

    %% Inverse Kinematics function
    function [theta1_deg, theta2_deg] = inverse(r1, r2, px, py)
        % SCARA Robot Inverse Kinematics
        % Calculates joint angles theta1 and theta2 for a given end-effector position
        distance = sqrt(px^2 + py^2); % Distance to target
        if distance > (r1 + r2)
            error('Target position is outside the reachable workspace.');
        end

        % Calculate theta2 using cosine law
        cos_theta2 = (px^2 + py^2 - r1^2 - r2^2) / (2 * r1 * r2);
        theta2 = acos(cos_theta2);

        % Calculate theta1
        theta1 = atan2(py, px) - atan2(r2 * sin(theta2), r1 + r2 * cos(theta2));

        % Convert to degrees
        theta1_deg = rad2deg(theta1);
        theta2_deg = rad2deg(theta2);
    end

    %% Callback for sending commands
    function sendCommand(~, ~)
        % Send Inverse Kinematics result to Arduino
        input1 = str2double(get(hInput1, 'String')); % Get X coordinate
        input2 = str2double(get(hInput2, 'String')); % Get Y coordinate

        if isnan(input1) || isnan(input2)
            disp('Invalid input. Enter numeric values.');
            return;
        end

        % Compute joint angles using IK
        [theta1_deg, theta2_deg] = inverse(r1, r2, input1, input2);

        % Format command and send to Arduino
        cmdStr = sprintf("C%.2f,%.2f;", theta1_deg, theta2_deg);
        write(s, cmdStr, "string");
    end

    %% Timer callback for reading data
    function readDataTimer(~, ~)
        % Read real-time data from Arduino and plot trajectory
        try
            dataStr = readline(s); % Read data from serial
            if isempty(dataStr)
                return;
            end

            % Parse data string
            data = sscanf(dataStr, "%c%f,%f");
            if numel(data) < 3
                return;
            end

            % Extract angles and compute end-effector position
            current_theta1 = data(2);
            current_theta2 = data(3);
            [current_x, current_y] = forward(current_theta1, current_theta2);

            % Update trajectory data
            y1 = [y1, current_x];
            y2 = [y2, current_y];

            % Plot real-time trajectory
            plot(hPlot, y1, y2, 'r-');
            hold on;
        catch
            disp('Error reading data.');
        end
    end

    %% GUI closure callback
    function closeGUI(~, ~)
        % Stop timer and close resources
        stop(hTimer);
        delete(hTimer);
        delete(s);
        delete(hFig);
    end
end
