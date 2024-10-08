% Create a 50x30 grid
rows = 30;
cols = 50;
grid = zeros(rows, cols);

% Add obstacles to the grid
obstacles = {
    1:30, 1;
    1:30, 50;
    1, 1:50;
    30, 1:50;
    2:16, 21;
    16, 11:20;
    16:30, 31;
    2:15, 41;
};

% Convert the obstacles to linear indices and set them in the grid
for i = 1:size(obstacles, 1)
    row = obstacles{i, 1};
    cols_range = obstacles{i, 2};
    for col = cols_range
        grid(row, col) = 1;
    end
end

% Start and goal nodes
start = [6, 6];
goal = [26, 46];

% Find the shortest path using Dijkstra's algorithm
[cost, path, visited] = dijkstra(grid, start, goal);

% Visualize the result
visualize_grid(grid, path, cost, visited, start, goal);




