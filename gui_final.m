% Communication between Arduino and MATLAB
% Enhanced GUI for Motor Control with Real-time Plotting
close all;
clear all;

% Declare global variables
global s hInputX hInputY hMotor1Angle hMotor2Angle hStatusText hPlot hTimer c y1 y2;

% Robot arm parameters (link lengths)
global r1 r2;
r1 = 107; % Length of link 1
r2 = 87; % Length of link 2

%% Setup serial communication
try
    s = serialport("COM3", 115200); % Ensure "COM3" is correct
    configureTerminator(s, "CR/LF");
catch
    errordlg('Failed to connect to Arduino. Check COM port.', 'Connection Error');
    return;
end

% Initialize variables for real-time plotting
c = [];
y1 = [];
y2 = [];

%% Create GUI
hFig = figure('Name', 'Motor Control GUI', 'Position', [300, 300, 600, 500], 'Resize', 'off');

% X coordinate input
uicontrol('Style', 'text', 'Position', [20, 440, 120, 20], 'String', 'X Coordinate (cm):', ...
    'HorizontalAlignment', 'left', 'FontSize', 10);
hInputX = uicontrol('Style', 'edit', 'Position', [150, 440, 100, 25], 'FontSize', 10, 'String', '0');

% Y coordinate input
uicontrol('Style', 'text', 'Position', [20, 400, 120, 20], 'String', 'Y Coordinate (cm):', ...
    'HorizontalAlignment', 'left', 'FontSize', 10);
hInputY = uicontrol('Style', 'edit', 'Position', [150, 400, 100, 25], 'FontSize', 10, 'String', '0');

% Motor 1 angle input
uicontrol('Style', 'text', 'Position', [20, 360, 120, 20], 'String', 'Motor 1 Angle (°):', ...
    'HorizontalAlignment', 'left', 'FontSize', 10);
hMotor1Angle = uicontrol('Style', 'edit', 'Position', [150, 360, 100, 25], 'FontSize', 10, 'String', '0');

% Motor 2 angle input
uicontrol('Style', 'text', 'Position', [20, 320, 120, 20], 'String', 'Motor 2 Angle (°):', ...
    'HorizontalAlignment', 'left', 'FontSize', 10);
hMotor2Angle = uicontrol('Style', 'edit', 'Position', [150, 320, 100, 25], 'FontSize', 10, 'String', '0');

% Drive motor by X/Y button
uicontrol('Style', 'pushbutton', 'String', 'Drive by XY', ...
    'Position', [300, 440, 100, 30], 'FontSize', 10, 'Callback', @sendXYCommand);

% Drive motor by angles button
uicontrol('Style', 'pushbutton', 'String', 'Drive by Angles', ...
    'Position', [300, 360, 100, 30], 'FontSize', 10, 'Callback', @sendAngleCommand);

% Status/Feedback display
uicontrol('Style', 'text', 'Position', [20, 280, 560, 20], 'String', 'Status:', ...
    'HorizontalAlignment', 'left', 'FontSize', 10);
hStatusText = uicontrol('Style', 'text', 'Position', [20, 260, 560, 20], 'String', 'Ready...', ...
    'HorizontalAlignment', 'left', 'FontSize', 10, 'ForegroundColor', 'blue');

% Real-time plot
hPlot = axes('Position', [0.1, 0.05, 0.8, 0.4]);
title(hPlot, 'Real-Time Data');
xlabel(hPlot, 'Time (s)');
ylabel(hPlot, 'Values');
legend(hPlot, {'Motor 1', 'Motor 2'}, 'Location', 'northwest');
grid on;

% Assign close function
hFig.CloseRequestFcn = @closeGUI;

% Setup timer for continuously receiving data from Arduino
hTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, 'TimerFcn', @readDataTimer);
start(hTimer);

%% Function to send command using XY coordinates
function sendXYCommand(~, ~)
    global s hInputX hInputY hStatusText r1 r2;

    % Read inputs
    x = str2double(get(hInputX, 'String'));
    y = str2double(get(hInputY, 'String'));

    % Validate inputs
    if isnan(x) || isnan(y)
        set(hStatusText, 'String', 'Error: Invalid input. Please enter numeric values.', 'ForegroundColor', 'red');
        return;
    end

    % Check if the target is within the reachable workspace
    if sqrt(x^2 + y^2) > (r1 + r2) || sqrt(x^2 + y^2) < abs(r1 - r2)
        set(hStatusText, 'String', 'Error: Target out of reach.', 'ForegroundColor', 'red');
        return;
    end

    % Perform inverse kinematics
    [theta1, theta2] = inverseKinematics(r1, r2, x, y);

    % Send command to Arduino
    command = sprintf("C%.2f,%.2f;", rad2deg(theta1), rad2deg(theta2));
    try
        write(s, command, "string");
        set(hStatusText, 'String', ['Command sent: ', command], 'ForegroundColor', 'blue');
        disp(['Command sent: ', command]);
    catch
        set(hStatusText, 'String', 'Error: Failed to send command.', 'ForegroundColor', 'red');
    end
end

%% Function to send command using motor angles
function sendAngleCommand(~, ~)
    global s hMotor1Angle hMotor2Angle hStatusText;

    % Read inputs
    angle1 = str2double(get(hMotor1Angle, 'String'));
    angle2 = str2double(get(hMotor2Angle, 'String'));

    % Validate inputs
    if isnan(angle1) || isnan(angle2)
        set(hStatusText, 'String', 'Error: Invalid input. Please enter numeric values.', 'ForegroundColor', 'red');
        return;
    end

    if angle1 < -360 || angle1 > 360 || angle2 < -360 || angle2 > 360
        set(hStatusText, 'String', 'Error: Angles must be between -360° and 360°.', 'ForegroundColor', 'red');
        return;
    end

    % Send command to Arduino
    command = sprintf("C%.2f,%.2f;", angle1, angle2);
    try
        write(s, command, "string");
        set(hStatusText, 'String', ['Command sent: ', command], 'ForegroundColor', 'blue');
        disp(['Command sent: ', command]);
    catch
        set(hStatusText, 'String', 'Error: Failed to send command.', 'ForegroundColor', 'red');
    end
end

%% Inverse Kinematics Function
function [theta1, theta2] = inverseKinematics(r1, r2, Px, Py)
    % Calculate theta2
    D = (Px^2 + Py^2 - r1^2 - r2^2) / (2 * r1 * r2);
    theta2 = atan2(sqrt(1 - D^2), D);

    % Calculate theta1
    theta1 = atan2(Py, Px) - atan2(r2 * sin(theta2), r1 + r2 * cos(theta2));
end

%% Callback function for reading data from Arduino
function readDataTimer(~, ~)
    global s hPlot c y1 y2;

    % Read data from serial port
    try
        dataStr = readline(s);
        if isempty(dataStr) || dataStr == ""
            return;
        end

        % Parse data values (expected format: c1,100,200)
        data = sscanf(dataStr, "c%f,%f");
        if isempty(data)
            return;
        end

        % Update data series
        y1 = [y1, data(1)];
        y2 = [y2, data(2)];
        if length(y1) > 100
            y1 = y1(end-99:end);
            y2 = y2(end-99:end);
        end

        % Update plot
        plot(hPlot, 1:length(y1), y1, 'r-', 1:length(y2), y2, 'b-');
        legend(hPlot, {'Motor 1', 'Motor 2'}, 'Location', 'northwest');
    catch
        disp('Error reading data from Arduino.');
    end
end

%% Cleanup function
function closeGUI(~, ~)
    global s hTimer;
    try
        stop(hTimer);
        delete(hTimer);
        delete(s); % Close serial port
    catch
        % Ignore errors during cleanup
    end
    delete(gcf);% Close GUI
end