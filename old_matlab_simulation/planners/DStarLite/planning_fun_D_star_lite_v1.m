function [final_path] = planning_fun_D_star_lite_v1(state_robot, dt, limit,...
    goal, image, resolution, maxIter, cost, range)

    moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];
    grid_search = D_star_lite_v1(state_robot, dt, limit, goal, image,...
        resolution, maxIter, moves, cost, range);
    
    grid_search.computeShortestPath();
    
    final_path = grid_search.run();
end


