%% Main
% 
% moves = [[1; 0], [0; 1], [-1; 0], [0; -1]];
% 
% obsts = [[4; 3], [4; 4], [4; 5], [4; 6], [5; 3], [6; 3], [7; 3]];
% m = Map(20, 20, obsts);
% 
% start = m.map(2, 3);
% start.state = Map.MAP_START;
% goal = m.map(17, 11);
% goal.state = Map.MAP_GOAL;
% 
% disp("Initial Map!")
% m.print_map();
% 
% d = D_Star(moves, m, goal);
% 
% d.run(start, goal);
% m.print_map();

function [final_path] = planning_fun_D_star(state_robot,dt,limit,goal,image,resolution,maxIter)
    grid_search = D_Star(state_robot,dt,limit,goal,image,resolution,maxIter);
    
    final_path = grid_search.run();
end

