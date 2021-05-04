classdef RRT_input_output_deltaInput < handle
    %RRT_PRIMITIVES Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        nodes
        dt
        map_limit
        goal
        map
        k
     
    end
    
    methods
        function obj = RRT_input_output_deltaInput(initial_state,sampling_time,limit,goal,map)
            %RRT_PRIMITIVES Construct an instance of this class
            %   Detailed explanation goes here
            obj.nodes = [initial_state(1) initial_state(2) initial_state(3) 0 0 0];
            obj.dt = sampling_time;
            obj.map_limit = limit;
            obj.goal = goal;
            obj.map = map;
            obj.k = [0.4770    0.5449];
            
 
        end
        
        function add_nodes(obj,new_node)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.nodes = vertcat(obj.nodes,new_node);
        end
        
        function finish = check_goal(obj,new_node)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            x = new_node(1);
            y = new_node(2);
            if((x - obj.goal(1))^2 < 0.2 & (y - obj.goal(2))^2 < 0.2)
                finish = 1;
            else
                finish = 0;
            end
        end
        
        function near_index = find_nearest(obj,new_node)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            number_of_nodes = size(obj.nodes);
            number_of_nodes = number_of_nodes(1);
            near_index = 1;
            best_distance = 10000;
            best_node = [0 0 0];
            
            for k = 1:number_of_nodes
                node = obj.nodes(k,:);
                x = node(1);
                y = node(2);
                theta = node(3); 
                distance = sqrt((x-new_node(1))^2 + (y-new_node(2))^2 + 1*(theta-new_node(3))^2);
                if(distance < best_distance)
                    best_distance = distance;
                    near_index = k;
                    best_node = node;
                end
            end
            
        end
        
        function desired_node = sample(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            sample_goal_prob = rand();
            if(sample_goal_prob > 0.8)
                desired_node = [obj.goal(1) obj.goal(2) 0];
            else
                rand_x = rand()*obj.map_limit(1);
                rand_y = rand()*obj.map_limit(2);
                rand_z = (rand() - 0.5)*pi;
                desired_node = [rand_x rand_y rand_z];
            end
        end
        
        
        function new_node = choose_primitives(obj,near_index, desired_node)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            near_node = obj.nodes(near_index,:);
            best_index = 1;
            best_distance = 10000;
            best_node = [0 0 0];
            %best_control = [obj.k obj.k];
            %best_control = [0 0];
            
            %to add dynamic
            b = 0.05;
            u1 = obj.k(1)*(desired_node(1) - near_node(1)) + obj.k(2)*(-near_node(5));
            u2 = obj.k(2)*(desired_node(2) - near_node(2)) + obj.k(2)*(-near_node(6));
            
            u1 = u1 + near_node(5);
            u2 = u2 + near_node(6);
            best_control = [u1 u2];
            
            x_new = near_node(1) + u1*obj.dt;
            y_new = near_node(2) + u2*obj.dt;
            theta_new = near_node(3) + (u2*cos(near_node(3)) - u1*sin(near_node(3)))*obj.dt/b;% + w*obj.dt;
            
            best_node = [x_new y_new theta_new]
       
            
            new_node = [best_node near_index best_control];
        end
        
        
        function path = take_path(obj,index)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            final_node = obj.nodes(index,:);
            path = [final_node];
            for i = 1:(index)
                if(final_node(4) == 0)
                    break;
                end
                parent = obj.nodes(final_node(4),:);
                path = vertcat(path,parent);
                final_node = parent;
            end
        end
        
        function good = check_collision(obj,node_to_check)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            x = node_to_check(1);
            y = node_to_check(2);
            theta = node_to_check(3);
            
            radius = 0.3;
            %height = 0.2;
            %width = 0.2;
            scale = 20;
            
            top = [int16(x*scale), int16((y + radius)*scale)];
            bottom = [int16(x*scale), int16((y - radius)*scale)];
            left = [int16((x - radius)*scale), int16(y*scale)];
            right = [int16((x + radius)*scale), int16(y*scale)];
            if(x < 0 | y < 0)
                good = 0;
            elseif(abs(x) > 18 | abs(y) > 18)
                good = 0;
            elseif(obj.map(top(1),top(2)) < 250 | obj.map(bottom(1),bottom(2)) < 250 | obj.map(left(1),left(2)) < 250 | obj.map(right(1),right(2)) < 250)
                good = 0;
            else
                good = 1;
            end
            
            
        end
        
    %end methods    
    end
%end class
end

