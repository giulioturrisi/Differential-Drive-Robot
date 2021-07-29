function [final_path] = planning_fun_RRT_primitives(state_robot,dt,limit,goal,image,resolution,maxIter)

RRT = RRT_primitives(state_robot,dt,limit,goal,image,resolution,maxIter);

path = state_robot;
size_path = 1;
%RRT loop
finish = 0;
for j = 1:maxIter
    j
    desired_node = RRT.sample();
    near_index = RRT.find_nearest(desired_node);
    [new_node,maneuvers] = RRT.choose_primitives(near_index,desired_node);
    %%check collision
    good = RRT.check_collision(new_node);
    if(good == 1)
        RRT.add_nodes(new_node);
    end
    finish = RRT.check_goal(new_node);
       
    %plot
    plot(goal(1),goal(2),'-o','Color','r'); hold on; plot(state_robot(1),state_robot(2),'-o','Color','b'); axis([0 3 0 3]);
    hold on;
    plot([RRT.nodes(near_index,1),maneuvers(1,1),maneuvers(2,1),maneuvers(3,1)],[RRT.nodes(near_index,2),maneuvers(1,2),maneuvers(2,2),maneuvers(3,2)]);

    
    if(finish == 1)
        [path,size_path] = RRT.take_path(new_node(4));
        break;
    end

end
%if not finish, i should take the nearest point
if(finish == 0)
    near_index = RRT.find_nearest([goal 0 0 0 0])
    [path,size_path] = RRT.take_path(near_index);
end
final_path = path(1:size_path+1,:);
end

