clear all; close all;
addpath(genpath('./planners'))
addpath(genpath('./controllers'))
addpath(genpath('./maps'))
addpath('./planners/A_star')

image = imread('simple_walls_map.pgm');
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);
resolution = 0.05;
scale = 1/resolution;


%x y theta x_dot y_dot theta_dot
state_robot = [1 1 0 0 0 0];
dt = 0.1;
goal = [2,1];
map_limit = [3,3];
max_iteration = 1000;

%LQR if needed
A = [1 dt; 0 1];
B = [dt;1];
Q = eye(2)*5;
R = 10;
[K,S,e] = dlqr(A,B,Q,R);

%RRT choice
%path = planning_fun_RRT_lqr(state_robot,dt,[3,3],goal,image,resolution,max_iteration)
%path = planning_fun_RRT_line(state_robot,dt,[3,3],goal,image,resolution,max_iteration)
%path = planning_fun_RRT_primitives(state_robot,dt,[3,3],goal,image,resolution,max_iteration*5)
%path = planning_fun_RRT_star_line(state_robot,dt,[3,3],goal,image,resolution,max_iteration)

%Grid search choice
path = planning_fun_A_star(state_robot,dt,[3,3],goal,image,resolution,max_iteration);



%initial state control
state_robot(1) = state_robot(1);% + 0.1;
state_robot(2) = state_robot(2);% - 0.1;
state_robot(3) = state_robot(3);% + 0.8;


%save path before spline
path_x = fliplr(path(:,1)');
path_y = fliplr(path(:,2)');
path_theta = fliplr(path(:,3)');
path_v = fliplr(path(:,5)');
path_w = fliplr(path(:,6)');
old_path = [path_x' path_y' path_theta' path_theta' path_v' path_w'];
%old_path = path;
size_path = size(old_path);

%interpolation (linear, makima, spline, etc)
size_path = size(path);
interpolation_dt = 0.1; %it will generate 10 new points every 1 original point
xq = 0:interpolation_dt:size_path(1);
path_x = fliplr(interp1(path(:,1),xq,'makima'));
path_y = fliplr(interp1(path(:,2),xq,'makima'));
path_theta = fliplr(interp1(path(:,3),xq,'makima'));
path_v = fliplr(interp1(path(:,5),xq,'makima'));
path_w = fliplr(interp1(path(:,6),xq,'makima'));
path = [path_x' path_y' path_theta' path_theta' path_v' path_w'];
size_path = size(path);



%controller choice - you can also use as sampling time dt*interpolation_dt
[rgbImage,real_robot] = input_output_linearization(image,state_robot,path,scale,goal,dt);
%[rgbImage,real_robot] = approximate_linearization (image,state_robot,path,scale,goal,dt)
%[rgbImage,real_robot] = nonlinear_lyapunov(image,state_robot,path,scale,goal,dt)

%plotting
plot(path(:,1),path(:,2)); hold on; plot(real_robot(:,1),real_robot(:,2))
figure(); J = imrotate(rgbImage,90); imshow(J);
