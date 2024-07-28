function visualize_grid(grid, path, cost, visited, start, goal)
    % Visualize the grid and the path

    % Check if the correct number of arguments are provided
    if nargin < 6
        error('Not enough input arguments. Expected grid, path, and cost.');
    end
    
    % Get the size of the grid
    [rows, cols] = size(grid);
    
    % Create a white background image
    img = ones(rows, cols, 3); 
    
    % Set obstacles to black
    obstacle_mask = (grid == 1);
    img(repmat(obstacle_mask, [1, 1, 3])) = 0; % Set all channels to 0 for obstacles
    
    % Ensure path contains valid indices
    if isempty(path)
        error('Path is empty or invalid.');
    end
    
    % Check if path indices are within grid bounds
    if any(path(:,1) < 1 | path(:,1) > rows) || any(path(:,2) < 1 | path(:,2) > cols)
        error('Path indices are out of bounds.');
    end
    
    % Display the grid
    figure;
    imshow(img, 'InitialMagnification', 'fit');
    hold on;

    % Fill visited nodes with grey color
    for i = 1:size(visited, 1)
        rectangle('Position', [visited(i, 2) - 0.5, visited(i, 1) - 0.5, 1, 1], 'FaceColor', [0.5, 0.5, 0.5, 0.5], 'EdgeColor', 'none');
    end
    
    % Plot the start and goal nodes
    scatter(start(2), start(1), 100, 'g', 'filled'); % Start node in green
    scatter(goal(2), goal(1), 100, 'b', 'filled'); % Goal node in blue
    
    % Draw the path
    plot(path(:, 2), path(:, 1), 'r', 'LineWidth', 2);
    % for i = 2:length(path)
    %     line([path(i-1, 2), path(i, 2)], [path(i-1, 1), path(i, 1)], 'Color', 'r', 'LineWidth', 2);
    % end

    % Add title
    text(25, -3, 'Dijkstra', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(25, -1, ['Cost: ', num2str(cost)], 'FontSize', 12, 'HorizontalAlignment', 'center');
    
    saveas(gcf, 'dijkstra.png');
    hold off;
end

