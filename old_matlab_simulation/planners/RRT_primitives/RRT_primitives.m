classdef RRT_primitives < handle
    %RRT_PRIMITIVES Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        nodes
        dt
        map_limit
        goal
        map
        resolution
        maxIter
        numberIter
        primitives
     
    end
    
    methods
        function obj = RRT_primitives(initial_state,sampling_time,limit,goal,map,resolution,maxIter)
            %RRT_PRIMITIVES Construct an instance of this class
            %   Detailed explanation goes here
            %obj.nodes = [initial_state(1) initial_state(2) initial_state(3) 0 0 0];
            obj.dt = sampling_time;
            obj.map_limit = limit;
            obj.goal = goal;
            obj.map = map;
            obj.resolution = resolution;
            obj.maxIter = maxIter;
            
            obj.numberIter = 1;
            size_state = size(initial_state);
            obj.nodes = zeros(maxIter,size_state(2));
            obj.nodes(1,:) = [initial_state(1) initial_state(2) initial_state(3) 0 0 0];
            
            %primitives v, w
            forward = [1,0];
            backward = [-1,0];
            turn_left = [0, -1];
            turn_right = [0, 1];
            arc_left_forward = [1, -1];
            arc_right_forward = [1, 1];
            arc_left_min_forward = [1, -0.5];
            arc_right_min_forward = [1, 0.5];
            %arc_left_forward_min = [0.5, -0.5];
            %arc_right_forward_min = [0.5, 0.5];
            arc_left_backward = [-1, -1];
            arc_right_backward = [-1, 1];
            arc_left_min_backward = [-1, -0.5];
            arc_right_min_backward = [-1, 0.5];

            %obj.primitives = [forward;turn_left;turn_right;arc_left_forward;arc_right_forward;
            %              arc_left_forward_min;arc_right_forward_min;
            %              arc_left_min_forward;arc_right_min_forward];
                      
            obj.primitives = [forward;arc_left_forward;arc_right_forward;
                          arc_left_min_forward;arc_right_min_forward;
                          arc_left_backward;arc_right_backward;arc_left_min_backward;arc_right_min_backward;
                          backward;turn_left;turn_right;];
            
 
        end
        
        function add_nodes(obj,new_node)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            %obj.nodes = vertcat(obj.nodes,new_node);
            %coder.varsize('obj.nodes');
            obj.numberIter = obj.numberIter + 1;
            obj.nodes(obj.numberIter,:) = new_node;
        end
        
        function finish = check_goal(obj,new_node)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            x = new_node(1);
            y = new_node(2);
            if((x - obj.goal(1))^2 < 0.05 & (y - obj.goal(2))^2 < 0.05)
                finish = 1;
            else
                finish = 0;
            end
        end
        
        function near_index = find_nearest(obj,new_node)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            number_of_nodes = obj.numberIter;
            %number_of_nodes = size(obj.nodes);
            %number_of_nodes = number_of_nodes(1);
            
            near_index = 1;
            best_distance = 10000;
            best_node = [0 0 0];
            
            for k = 1:number_of_nodes
                node = obj.nodes(k,:);
                x = node(1);
                y = node(2);
                theta = node(3); 
                distance = sqrt((x-new_node(1))^2 + (y-new_node(2))^2 + 1*0*(theta-new_node(3))^2);
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
        
        
        function [new_node,maneuvers] = choose_primitives(obj,near_index, desired_node)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            near_node = obj.nodes(near_index,:);
            best_index = 1;
            best_distance = 10000;
            best_node = [0 0 0];
            maneuvers = zeros(3,3);
            
           best_control = [];
           v_w = obj.primitives();
           for i = 1:length(obj.primitives)
               %d = 10;
               for d = 1:10
               %    if(d < 40)
                       v = v_w(i,1)*d;
                       w = v_w(i,2)*d;
               %    else
               %        v = v_w(i,1)*rand();
               %        w = v_w(i,2)*rand();
               %    end
                    x_new1 = near_node(1) + v*cos(near_node(3))*obj.dt/3.;
                    y_new1 = near_node(2) + v*sin(near_node(3))*obj.dt/3.;
                    theta_new1 = near_node(3) + w*obj.dt/3.;
                    collision_1 = obj.check_collision([x_new1 y_new1 theta_new1]);
    
                    x_new2 = x_new1 + v*cos(theta_new1)*obj.dt/3.;
                    y_new2 = y_new1 + v*sin(theta_new1)*obj.dt/3.;
                    theta_new2 = theta_new1 + w*obj.dt/3.;
                    collision_2 = obj.check_collision([x_new2 y_new2 theta_new2]);

                    
                    x_new = x_new2 + v*cos(theta_new2)*obj.dt/3.;
                    y_new = y_new2 + v*sin(theta_new2)*obj.dt/3.;
                    theta_new = theta_new2 + w*obj.dt/3.;
                    collision = obj.check_collision([x_new y_new theta_new]);
                    
                    
                                       
                    distance = sqrt((x_new-desired_node(1))^2 + (y_new-desired_node(2))^2 + 0.*(theta_new-desired_node(3))^2);
                    
                    %fprintf('The primitives %d generate a distance of ',i); distance
                    if(distance < best_distance & collision_1 & collision_2 & collision)
                        best_distance = distance;
                        best_index = i;
                        best_node = [x_new y_new theta_new];
                        best_control = [v w];
                        maneuvers(1,:) = [x_new1,y_new1,theta_new1];
                        maneuvers(2,:) = [x_new2,y_new2,theta_new2];
                        maneuvers(3,:) = [x_new,y_new,theta_new];
                    end
               end
           end
           best_distance
           new_node = [best_node near_index best_control];
        end
        
        
        function [path,size_path] = take_path(obj,index)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            final_node = obj.nodes(index,:);
            
            size_node = size(final_node);
            path = ones(obj.maxIter,size_node(2));
            path(1,:) = final_node;
            
            size_path = 0;
            %path = [final_node];
            for i = 1:(index)
                if(final_node(4) == 0)
                    break;
                end
                parent = obj.nodes(final_node(4),:);
                %path = vertcat(path,parent);
                path(i+1,:) = parent;
                final_node = parent;
                size_path = size_path+1;
            end
        end
        
        function good = check_collision(obj,node_to_check)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            x = node_to_check(1);
            y = node_to_check(2);
            theta = node_to_check(3);
            
          
            scale = 1/obj.resolution;

            if(x < 0 | y < 0)
                good = 0;
                return;
            end
            if(int16(x*scale)+1 > size(obj.map,1))
                good = 0;
                return;
            end
            if(int16(y*scale)+1 > size(obj.map,2))
                good = 0;
                return;
            end       
     
            if(obj.map(int16(x*scale)+1,int16(y*scale)+1) < 250)
                good = 0;
            else
                good = 1;
            end
            
            
        end
        
    %end methods    
    end
%end class
end

