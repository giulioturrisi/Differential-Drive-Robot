function [final_path] = planning_fun_star(state_robot,dt,limit,goal,image,resolution,maxIter)

state_robot = [state_robot 0];
RRT = RRT_star_input_output(state_robot,dt,limit,goal,image,resolution,maxIter);

path = state_robot;
size_path = 1;

%RRT loop
for j = 1:maxIter
    j
    desired_node = RRT.sample();
    near_index = RRT.find_nearest(desired_node);
    new_node = RRT.choose_primitives(near_index,desired_node);
    
    %near_index = RRT.choose_parent(new_node);
    %new_node = RRT.choose_primitives(near_index,desired_node);
    
    %%check collision
    good = RRT.check_collision(new_node);
    if(good == 1)
        RRT.add_nodes(new_node);
        distance_max = 0.06;
        RRT.find_nearest_ball(new_node,distance_max);
    end
    

end

near_index = RRT.find_nearest_minimum_cost([goal 0])
[path,size_path] = RRT.take_path(near_index);
final_path = path(1:size_path+1,:);
end

