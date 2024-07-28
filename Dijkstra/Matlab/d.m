function [path, cost, visited] = dijkstra(grid, start_node, goal_node)
    % Dijkstra algorithm to find the shortest path in a grid
    % grid: 2D grid map (0: free, 1: obstacle)
    % start_node: [row, col] of the start node
    % goal_node: [row, col] of the goal node
    
    [rows, cols] = size(grid);
    max_cost = inf;
    
    % Initialize cost and predecessor matrices
    cost_map = max_cost * ones(rows, cols);
    cost_map(start_node(1), start_node(2)) = 0;
    
    % Initialize the visited list
    visited = [];
    % Priority queue for nodes to visit
    pq = PriorityQueue();
    pq.insert(start_node, 0);
    
    % Initialize the predecessors map to reconstruct the path
    predecessors = cell(rows, cols);
    
    % Possible moves (8 directions)
    moves = [-1, 0; -1, 1; 0, 1; 1, 1; 1, 0; 1, -1; 0, -1; -1, -1];
    move_costs = [1, sqrt(2), 1, sqrt(2), 1, sqrt(2), 1, sqrt(2)];
    
    while ~pq.isEmpty()
        [current, current_cost] = pq.extractMin();
        if isempty(visited) || ~ismember(current, visited,"rows")
            visited = [visited; current];
        end
        
        if isequal(current, goal_node)
            break;
        end
        
        for i = 1:size(moves, 1)
            neighbor = current + moves(i, :);
            if neighbor(1) < 1 || neighbor(1) > rows || neighbor(2) < 1 || neighbor(2) > cols
                continue;
            end
            if grid(neighbor(1), neighbor(2)) == 1
                continue;
            end

            if moves(i,1) ~= 0 && moves(i,2) ~= 0
                if grid(current(1),neighbor(2)) == 1 || grid(neighbor(1),current(2)) == 1
                    continue
                end
            end

            new_cost = current_cost + move_costs(i);

            if new_cost < cost_map(neighbor(1), neighbor(2))
                cost_map(neighbor(1), neighbor(2)) = new_cost;
                pq.insert(neighbor, new_cost);
                predecessors{neighbor(1), neighbor(2)} = current;
            end
        end
    end
    
    % Construct Path
    path = [];
    if ~isnan(predecessors{goal_node(1), goal_node(2)})
        current = goal_node;
        while ~isequal(current, start_node)
            % Easy to track back
            path = [current; path];
            current = predecessors{current(1), current(2)};
        end
        path = [start_node; path];
        cost = cost_map(goal_node(1), goal_node(2));
    else
        cost = max_cost;
    end
end




