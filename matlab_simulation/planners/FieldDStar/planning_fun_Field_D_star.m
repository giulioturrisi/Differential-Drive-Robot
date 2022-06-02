function [final_path] = planning_fun_Field_D_star(state_robot, dt, limit,...
    goal, image, resolution, maxIter,  moves, cost, range)
    grid_search = Field_D_star(state_robot, dt, limit, goal, image,...
        resolution, maxIter, moves, cost, range);
    
    grid_search.computeShortestPath();
    
    final_path = grid_search.run();
end



