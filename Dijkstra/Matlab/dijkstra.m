function [cost, path, visit_order] = dijkstra(grid, start_node, goal_node)
    % Dijkstra's algorithm implementation in MATLAB
    % Input: 
    %   grid - The grid map (0 = free space, 1 = obstacle)
    %   start_node - The start node [row, col]
    %   goal_node - The goal node [row, col]
    % Output:
    %   cost - The cost to reach the goal node
    %   path - The path from start_node to goal_node
    %   visit_order - The order in which nodes were visited

    % Initialize the priority queue with the start node
    priority_queue = [];
    priority_queue = [priority_queue; 0, start_node];
    costs = containers.Map('KeyType', 'char', 'ValueType', 'double');
    previous_nodes = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    % Encode the start node as a string
    start_key = sprintf('%d,%d', start_node);
    costs(start_key) = 0;
    previous_nodes(start_key) = [];

    visited = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    visit_order = [];
    
    while ~isempty(priority_queue)
        % Pop the node with the lowest cost
        [~, idx] = min(priority_queue(:, 1));
        current_cost = priority_queue(idx, 1);
        current_node = priority_queue(idx, 2:3);
        priority_queue(idx, :) = [];
        
        % Encode the current node as a string
        current_key = sprintf('%d,%d', current_node);
        
        % Skip if already visited
        if isKey(visited, current_key)
            continue;
        end
        
        % Mark the node as visited
        visited(current_key) = true;
        visit_order = [visit_order; current_node];
        
        % Check if we reached the goal
        if isequal(current_node, goal_node)
            break;
        end
        
        % Get the neighbors of the current node
        [neighbors, distances] = get_neighbors(grid, current_node);
        
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            distance = distances(i);
            
            % Skip if the neighbor is invalid
            if neighbor(1) == -1
                continue;
            end
            
            % Encode the neighbor as a string
            neighbor_key = sprintf('%d,%d', neighbor);
            
            % Compute the cost to reach the neighbor
            cost = current_cost + distance;
            
            % If the neighbor has not been visited or a cheaper cost is found
            % if ~isKey(costs, neighbor_key) || cost < costs(neighbor_key)
            if ~isKey(costs, neighbor_key)
                costs(neighbor_key) = cost;
                previous_nodes(neighbor_key) = current_node;
                priority_queue = [priority_queue; cost, neighbor];
            end
        end
    end
    
    % Reconstruct the path from goal to start
    path = [];
    current_node = goal_node;
    while ~isempty(current_node)
        path = [current_node; path];
        current_key = sprintf('%d,%d', current_node);
        current_node = previous_nodes(current_key);
    end
    
    % Return the cost to reach the goal
    cost = costs(sprintf('%d,%d', goal_node));
end

function [neighbors, distances] = get_neighbors(grid, node)
    % Get the valid neighbors of the current node in the grid
    [rows, cols] = size(grid);
    moves = [-1, 0; 1, 0; 0, -1; 0, 1; -1, -1; -1, 1; 1, -1; 1, 1];  % 8 directions
    neighbors = [];
    distances = [];
    
    for i = 1:size(moves, 1)
        neighbor = node + moves(i, :);
        
        % Check if the neighbor is within bounds
        if neighbor(1) >= 1 && neighbor(1) <= rows && neighbor(2) >= 1 && neighbor(2) <= cols
            if grid(neighbor(1), neighbor(2)) == 0  % Check if the neighbor is not an obstacle
                if moves(i,1) ~= 0 && moves(i,2) ~= 0
                    if grid(node(1),neighbor(2)) == 1 || grid(neighbor(1),node(2)) == 1
                        continue
                    end
                end
                neighbors = [neighbors; neighbor];
                if i <= 4
                    distances = [distances; 1];  % Distance for orthogonal moves
                else
                    distances = [distances; sqrt(2)];  % Distance for diagonal moves
                end
            end
        end
    end
    
    % If no valid neighbors, return a default invalid neighbor
    if isempty(neighbors)
        neighbors = [-1, -1];
        distances = [Inf];
    end
end
