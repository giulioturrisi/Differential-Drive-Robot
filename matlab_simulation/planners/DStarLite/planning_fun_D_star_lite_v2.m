function [final_path] = planning_fun_D_star_lite_v2(state_robot, dt, limit,...
    goal, image, resolution, maxIter,  moves, cost, range)
    grid_search = D_star_lite_v2(state_robot, dt, limit, goal, image,...
        resolution, maxIter,  moves, cost, range);
    
    grid_search.computeShortestPath();
    
    final_path = grid_search.run();
end


