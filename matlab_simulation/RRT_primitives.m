classdef RRT_primitives < handle
    %RRT_PRIMITIVES Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        nodes
        dt
        map_limit
        primitives
        goal
        map
     
    end
    
    methods
        function obj = RRT_primitives(initial_state,sampling_time,limit,goal,map)
            %RRT_PRIMITIVES Construct an instance of this class
            %   Detailed explanation goes here
            obj.nodes = [initial_state(1) initial_state(2) initial_state(3) 0 0 0];
            obj.dt = sampling_time;
            obj.map_limit = limit;
            obj.goal = goal;
            obj.map = map;
            
            %primitives v, w
            forward = [1,0]
            %backward = [-1*0,0]
            turn_left = [0, -1]*0.3
            turn_right = [0, 1]*0.3
            arc_left_forward = [1, -1*0.3]
            arc_right_forward = [1, 1*0.3]
            arc_left_min_forward = [1, -0.5*0.3]
            arc_right_min_forward = [1, 0.5*0.3]
            arc_left_forward_min = [0.5, -0.5*0.3]
            arc_right_forward_min = [0.5, 0.5*0.3]
            %arc_left_backward = [-1*0, -1*0.3]
            %arc_right_backward = [-1*0, 1*0.3]
            %arc_left_backward_min = [-1*0, -0.5*0.3]
            %arc_right_backward_min = [-1*0, 0.5*0.3]

            obj.primitives = [forward;turn_left;turn_right;arc_left_forward;arc_right_forward;
                          arc_left_forward_min;arc_right_forward_min;
                          arc_left_min_forward;arc_right_min_forward]
            
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
            if((x - obj.goal(1))^2 < 0.5 & (y - obj.goal(2))^2 < 0.5)
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
                distance = sqrt((x-new_node(1))^2 + (y-new_node(2))^2 + 2*(theta-new_node(3))^2);
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
            best_control = [];
            v_w = obj.primitives();
            for i = 1:length(obj.primitives)
                    for d = 1:5
                        %if(d < 5)
                            v = v_w(i,1)*d*0.1;
                            w = v_w(i,2)*d*0.1;
                        %else
                        %    v = v_w(i,1)*rand();
                        %    w = v_w(i,2)*rand();
                        %end
                        

                        x_new = near_node(1) + v*cos(near_node(3))*obj.dt;
                        y_new = near_node(2) + v*sin(near_node(3))*obj.dt;
                        theta_new = near_node(3) + w*obj.dt;

                        distance = sqrt((x_new-desired_node(1))^2 + (y_new-desired_node(2))^2 + 2*(theta_new-desired_node(3))^2);
                        if(distance < best_distance & obj.check_collision([x_new y_new theta_new]))
                        %if(distance < best_distance)    
                            best_distance = distance;
                            best_index = i;
                            best_node = [x_new y_new theta_new];
                            best_control = [v w];
                        end
                    end
            end
            
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
            
            radius = 0.15;
            %height = 0.2;
            %width = 0.2;
            scale = 20;
            
            top = [int16(x*scale), int16((y + radius)*scale)];
            bottom = [int16(x*scale), int16((y - radius)*scale)];
            left = [int16((x - radius)*scale), int16(y*scale)];
            right = [int16((x + radius)*scale), int16(y*scale)];
            if(abs(x) > 18 | abs(y) > 18)
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

