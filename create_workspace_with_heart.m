function create_workspace_with_checkered_rectangle()
    % Lengths of the arms
    l1 = 107; % length of first arm
    l2 = 87;  % length of second arm

    % Possible values of theta1 and theta2
    theta1 = 0:0.05:pi; % all possible theta1 values
    theta2 = -pi:0.05:pi; % all possible theta2 values

    % Generate grid of angle values
    [THETA1, THETA2] = meshgrid(theta1, theta2);

    % Compute x and y coordinates based on the arm lengths and angles
    X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2); % x coordinates
    Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2); % y coordinates

    % Plot the complete workspace
    figure;
    plot(X(:), Y(:), 'r.', 'MarkerSize', 1);
    hold on;

    % Define rectangle dimensions
    rect_width = 156;  % Width of the rectangle
    rect_height = 156; % Height of the rectangle

    % Calculate rectangle's bottom-left corner
    rect_x = -78; % Left edge at -78
    rect_y = 20;  % Bottom edge at 20

    % Generate checkered pattern inside the rectangle
    square_size = 12; % Size of each square in the checkered pattern
    for i = 0:(rect_width / square_size - 1)
        for j = 0:(rect_height / square_size - 1)
            % Determine the bottom-left corner of the current square
            square_x = rect_x + i * square_size;
            square_y = rect_y + j * square_size;

            % Alternate colors for the checkered pattern
            if mod(i + j, 2) == 0
                % Blue squares
                fill([square_x, square_x + square_size, square_x + square_size, square_x], ...
                     [square_y, square_y, square_y + square_size, square_y + square_size], 'b', 'EdgeColor', 'none');
            else
                % White squares
                fill([square_x, square_x + square_size, square_x + square_size, square_x], ...
                     [square_y, square_y, square_y + square_size, square_y + square_size], 'w', 'EdgeColor', 'none');
            end
        end
    end

    % Draw rectangle boundary
    rectangle('Position', [rect_x, rect_y, rect_width, rect_height], 'EdgeColor', 'k', 'LineWidth', 1.5);

    % Annotate the rectangle corners
    text(rect_x, rect_y, sprintf('(%d, %d)', round(rect_x), round(rect_y)), ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'fontsize', 8);
    text(rect_x, rect_y + rect_height, sprintf('(%d, %d)', round(rect_x), round(rect_y + rect_height)), ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'fontsize', 8);
    text(rect_x + rect_width, rect_y + rect_height, sprintf('(%d, %d)', round(rect_x + rect_width), round(rect_y + rect_height)), ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'fontsize', 8);
    text(rect_x + rect_width, rect_y, sprintf('(%d, %d)', round(rect_x + rect_width), round(rect_y)), ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'fontsize', 8);

    % Draw a yellow heart inside the blue squares region
    t = linspace(0, 2 * pi, 1000); % Parameter for heart shape
    heart_x = 6 * sin(t).^3; % X-coordinates of the heart
    heart_y = 5 * cos(t) - 2 * cos(2 * t) - cos(3 * t) - cos(4 * t); % Y-coordinates of the heart
    heart_x = heart_x * 7; % Scale the heart
    heart_y = heart_y * 7;

    % Position the heart at the center of the blue checkered region
    heart_center_x = rect_x + rect_width / 2;
    heart_center_y = rect_y + rect_height / 2;
    heart_x = heart_x + heart_center_x;
    heart_y = heart_y + heart_center_y;

    % Fill the heart with yellow color
    fill(heart_x, heart_y, 'y', 'EdgeColor', 'none');

    % Adjust axis to visualize the full workspace and rectangle
    axis equal;
    xlim([-200, 200]); % Extend axis limits to show full workspace
    ylim([-50, 250]);
    grid on;

    % Label the axes and add title
    xlabel('X (mm)', 'fontsize', 10);
    ylabel('Y (mm)', 'fontsize', 10);
    title('Workspace with Checkered Pattern Rectangle and Yellow Heart', 'fontsize', 10);

    hold off;
end
