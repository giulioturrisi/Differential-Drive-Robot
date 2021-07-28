function [final_path] = planning_fun_A_star(state_robot,dt,limit,goal,image,resolution,maxIter)

graph_search = A_star(state_robot,dt,limit,goal,image,resolution,maxIter);
finish = 0;
while(finish == 0)
   next_cell = graph_search.find_next_cell();
   if(next_cell == -1)
       finish = 1;
       break;
   end
   finish = graph_search.check_goal(next_cell);
   if(finish == 1)
       break;
   end
   graph_search.open_sons_cell(next_cell);
end
final_path = graph_search.take_path();
end

