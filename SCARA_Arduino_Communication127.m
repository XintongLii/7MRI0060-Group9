% MATLAB-Arduino Communication for SCARA Robot
% @author         Alejandro Granados
% @organisation   King's College London
% @module         Medical Robotics Hardware Development
% @year           2024

function SCARA_Arduino_Communication()
    close all;
    clear all;

    % Global variables
    global s hInput1 hInput2 hPlot hFig hTimer c y1 y2
    r1 = 107; % Reararm length [mm]
    r2 = 87;  % Forearm length [mm]

    % Setup serial communication
    serialports = seriallist(); % List available serial ports
    disp('Available Serial Ports:');
    disp(serialports);
    s = serialport(serialports(1), 115200);
    configureTerminator(s, "CR/LF");
    s.UserData = struct("Data", [], "Count", 1);

    % Create GUI
    hFig = figure('Name', 'SCARA Robot Controller', 'NumberTitle', 'off');
    hInput1 = uicontrol('Style', 'edit', 'Position', [20, 20, 100, 25], 'String', 'Enter X');
    hInput2 = uicontrol('Style', 'edit', 'Position', [120, 20, 100, 25], 'String', 'Enter Y');
    uicontrol('Style', 'pushbutton', 'String', 'Send', 'Position', [20, 50, 100, 25], 'Callback', @sendCommand);
    hPlot = axes('Position', [0.2, 0.6, 0.6, 0.3]);
    xlabel(hPlot, 'X (mm)');
    ylabel(hPlot, 'Y (mm)');
    title(hPlot, 'End-Effector Trajectory');
    grid on;

    % Initialize variables for plotting
    c = [];
    y1 = [];
    y2 = [];

    % Timer for data updates
    hTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, 'TimerFcn', @readDataTimer);
    start(hTimer);
    hFig.CloseRequestFcn = @closeGUI;

    % Send command to Arduino
    function sendCommand(~, ~)
        input1 = str2double(get(hInput1, 'String'));
        input2 = str2double(get(hInput2, 'String'));

        if isnan(input1) || isnan(input2)
            disp('Invalid input.');
            return;
        end

        % Call inverse kinematics to calculate joint angles
        [t1, t2] = inverse_kinematics(r1, r2, input1, input2);

        % Send formatted command to Arduino
        cmdStr = sprintf("C%.2f,%.2f;", t1, t2);
        write(s, cmdStr, "string");
    end

    % Timer callback for reading data
    function readDataTimer(~, ~)
        try
            dataStr = readline(s);
            if isempty(dataStr)
                return;
            end
            data = sscanf(dataStr, "%c%f,%f");
            current_t1 = data(2);
            current_t2 = data(3);
            [current_x, current_y] = forward_kinematics(r1, r2, current_t1, current_t2);
            y1 = [y1, current_x];
            y2 = [y2, current_y];
            plot(hPlot, y1, y2, 'r-');
            hold on;
        catch
            disp('Error reading data.');
        end
    end

    % Cleanup on GUI closure
    function closeGUI(~, ~)
        stop(hTimer);
        delete(hTimer);
        delete(s);
        delete(hFig);
    end
end
