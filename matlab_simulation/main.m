clear all;
%x y theta x_dot y_dot theta_dot
%state_robot = [4 10 0 0 0 0];
state_robot = [1 1 0 0 0 0];
dt = 0.1;

%image = imread('map.pgm');
image = imread('coppeliasim_simple.pgm');
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);
resolution = 0.05;
scale = 1/resolution;
%N.B 0.0039 è free, sopra tutto occupato o incerto

%LQR for input-output planning (if used)
%A = ;
%B = [1];
%Q = [10];
%R = 100;
%[K,S,e] = lqr(A,B,Q,R);

A = [1 dt; 0 1];
B = [dt;1];
Q = eye(2)*5;
R = 10;
[K,S,e] = dlqr(A,B,Q,R);

%RRT choice
%RRT = RRT_input_output(state_robot,dt,[20,20],[9,5],image);
%RRT = RRT_input_output_deltaInput(state_robot,dt,[20,20],[9,5],image);
%RRT = RRT_primitives(state_robot,dt,[20,20],[10,4],image);


% RRT = RRT_input_output_deltaInput(state_robot,dt,[3,3],[2,2],image,resolution);
% 
% %RRT loop
% for j = 1:100000
%     j
%     desired_node = RRT.sample();
%     near_index = RRT.find_nearest(desired_node);
%     new_node = RRT.choose_primitives(near_index,desired_node);
%     %%check collision
%     good = RRT.check_collision(new_node);
%     if(good == 1)
%         RRT.add_nodes(new_node);
%     end
%     finish = RRT.check_goal(new_node);
%     
%     if(finish == 1)
%         path = RRT.take_path(new_node(4));
%         break;
%     end
% end
goal = [3.291794,0];
goal = [1.5,0.5];
map_limit = [3,3];
max_iteration = 1000;
path = planning_fun(state_robot,dt,[3,3],goal,image,resolution,max_iteration)



%parameters needed for control loop
k1 = 10;
k2 = 10;
k3 = 5;
state_robot(1) = state_robot(1);% + 0.1;
state_robot(2) = state_robot(2);% - 0.1;
state_robot(3) = state_robot(3);% + 0.8;
real_robot = [state_robot(1),state_robot(2),state_robot(3)];

%augment path for bigger control_dt
size_path = size(path);
control_dt = 0.01;
xq = 0:control_dt:size_path(1);
path_x = fliplr(interp1(path(:,1),xq,'spline'));
path_y = fliplr(interp1(path(:,2),xq,'spline'));
path_theta = fliplr(interp1(path(:,3),xq,'spline'));
path_v = fliplr(interp1(path(:,5),xq,'linear'));
path_w = fliplr(interp1(path(:,6),xq,'linear'));
path = [path_x' path_y' path_theta' path_theta' path_v' path_w'];
size_path = size(path);


%grey to rgb mab
rgbImage = cat(3, image, image, image);

%START
rgbImage(int16(state_robot(1)*scale),int16(state_robot(2)*scale),1) = 0;
rgbImage(int16(state_robot(1)*scale),int16(state_robot(2)*scale),2) = 255;
rgbImage(int16(state_robot(1)*scale),int16(state_robot(2)*scale),3) = 0;

%GOAL
rgbImage(int16(goal(1)*scale),int16(goal(2)*scale),1) = 255;
rgbImage(int16(goal(1)*scale),int16(goal(2)*scale),2) = 0;
rgbImage(int16(goal(1)*scale),int16(goal(2)*scale),3) = 0;

%main loop control
for d = 2:size_path(1)-1
    if(d == size_path(1))
        break;
    end
    near_node = path(d,:);

    
    
    %nonlinear control
    e1 = cos(state_robot(3))*(near_node(1) - state_robot(1)) + sin(state_robot(3))*(near_node(2) - state_robot(2));
    e2 = -sin(state_robot(3))*(near_node(1) - state_robot(1)) + cos(state_robot(3))*(near_node(2) - state_robot(2));
    e3 = near_node(3) - state_robot(3);
    
    u1 = -k1*e1;
    if(e3 == 0)
        u2 = -k2*near_node(5)*(sin(e3)/(e3 + 0.001))*e2 - k3*e3;
    else
        u2 = -k2*near_node(5)*(sin(e3)/(e3 + 0.001))*e2 - k3*e3;
    end
    %se lo voglio lineare
    u2 = -k2*e2 - k3*e3;
    
    v = near_node(5)*cos(e3) - u1;
    w = near_node(6) - u2;
    
    %if(isnan(v))
    %    disp("stop")
    %end
    

    
    %input-output lin
    x_vel = (near_node(1) - path(d - 1,1))/dt;
    y_vel = (near_node(2) - path(d - 1,2))/dt;
    u1_io = k1*0.5*(near_node(1) - state_robot(1)) + x_vel;
    u2_io = k1*0.5*(near_node(2) - state_robot(2)) + y_vel;
    v = cos(state_robot(3))*u1_io + sin(state_robot(3))*u2_io;
    w = -sin(state_robot(3))*u1_io/0.01 + cos(state_robot(3))*u2_io/0.01;
    

    
    
    %integration
    state_robot(1) = state_robot(1) + v*cos(state_robot(3))*dt;
    state_robot(2) = state_robot(2) + v*sin(state_robot(3))*dt; 
    state_robot(3) = state_robot(3) + w*dt;
    
    
    %draw path and inflated robot
    x = near_node(1);
    y = near_node(2);
    %x = state_robot(1);
    %y = state_robot(2);
    radius = 0.15;
    
    %rgbImage = insertShape(rgbImage,'circle',[int16(near_node(2)*scale) int16(near_node(1)*scale) radius],'LineWidth',1, 'Color', 'blue');
    rgbImage(int16(x*scale),int16(y*scale),1) = 0;
    rgbImage(int16(x*scale),int16(y*scale),2) = 0;
    rgbImage(int16(x*scale),int16(y*scale),3) = 255;
    
    
    
    real_robot = vertcat(real_robot,[state_robot(1),state_robot(2),state_robot(3)]);
end

%plotting
plot(path(:,1),path(:,2));
hold on;
plot(real_robot(:,1),real_robot(:,2))

figure();
J = imrotate(rgbImage,90);
imshow(J);
