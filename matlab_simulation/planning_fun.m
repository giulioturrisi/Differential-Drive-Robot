function [final_path] = planning_fun(state_robot,dt,limit,goal,image,resolution,maxIter)

RRT = RRT_input_output_deltaInput(state_robot,dt,limit,goal,image,resolution,maxIter);

path = state_robot;
size_path = 1;
%RRT loop
for j = 1:maxIter
    j;
    desired_node = RRT.sample();
    near_index = RRT.find_nearest(desired_node);
    new_node = RRT.choose_primitives(near_index,desired_node);
    %%check collision
    good = RRT.check_collision(new_node);
    if(good == 1)
        RRT.add_nodes(new_node);
    end
    finish = RRT.check_goal(new_node);
    
    if(finish == 1)
        [path,size_path] = RRT.take_path(new_node(4));
        break;
    end
end

final_path = path(1:size_path+1,:);
end

