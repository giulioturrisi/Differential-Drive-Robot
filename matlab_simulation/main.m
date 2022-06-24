clear all; close all; clc;

%% load path
addpath(genpath('./planners'))
addpath(genpath('./controllers'))
addpath(genpath('./controllers/nonlinear_mpc'))
addpath(genpath('./maps'))
%addpath('./planners/A_star')

%% load map and set resolution
image = imread('map.pgm');
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);
resolution = 0.05;
scale = 1/resolution;

%% initial state of the robot, goal position, hyperparameters
state_robot = [1 1 0 0 0 0]; %x y theta x_dot y_dot theta_dot
dt = 0.1; %sampling time
%goal = [2,1];
goal = [0.5,2.5]; %goal
map_limit = [3,3]; %map dimension
max_iteration = 1000; %max iterations planner
v_max = 10; %saturate maximum linear velocity
w_max = 10; %saturate maximum angular velocity

%% LQR if needed
A = [1 dt; 0 1];
B = [dt;1];
Q = eye(2)*5;
R = 10;
[K,S,e] = dlqr(A,B,Q,R);


%% planner choice
disp("Select planning algorithm"+newline+...
     "    1: A*"+newline+...
     "    2: Greedy best first"+newline+...
     "    3: Dijkstra"+newline+...
     "    4: D*"+newline+...
     "    5: D*Lite v1"+newline+...
     "    6: D*Lite v2"+newline+...
     "    7: Field D*"+newline+...
     "    8: RRT LQR"+newline+...
     "    9: RRT line"+newline+...
     "    10: RRT* line"+newline+...
     "    11: RRT primitives"+newline)
algorithm = input('planning algorithm: ');

switch algorithm
    case 1
        path = planning_fun_A_star(state_robot,dt,[3,3],goal,image,resolution,max_iteration);
    case 2
        path = planning_fun_Greedy_best_first(state_robot,dt,[3,3],goal,image,resolution,max_iteration);
    case 3
        path = planning_fun_Dijkstra(state_robot,dt,[3,3],goal,image,resolution,max_iteration);
    case 4
        path = planning_fun_D_star(state_robot, dt, map_limit, goal, image, resolution, max_iteration);
    case 5
        cost = 1;
        range = 2;
        path = planning_fun_D_star_lite_v1(state_robot, dt, map_limit, goal, image,...
            resolution,max_iteration, range, cost);
    case 6
        path = 0
    case 7
        path = 0
    case 8
        path = planning_fun_RRT_lqr(state_robot,dt,[3,3],goal,image,resolution,max_iteration);
    case 9
        path = planning_fun_RRT_line(state_robot,dt,[3,3],goal,image,resolution,max_iteration);
    case 10
        path = planning_fun_RRT_star_line(state_robot,dt,[3,3],goal,image,resolution,max_iteration);
    case 11
        path = planning_fun_RRT_primitives(state_robot,dt,[3,3],goal,image,resolution,max_iteration);
end    

%initial state control
%state_robot(1) = state_robot(1);% + 0.1;
%state_robot(2) = state_robot(2);% - 0.1;
%state_robot(3) = state_robot(3);% + 0.8;

%% controller choice
disp("Select controller"+newline+...
     "    1: nonlinear_mpc"+newline+...
     "    2: input_output_linearization"+newline+...
     "    3: approximate_linearization"+newline+...
     "    4: nonlinear_lyapunov"+newline)
controller = input('controller: ');


%% path saving and interpolation with splines
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


%% simulation
%you can also use as sampling time dt*interpolation_dt
switch controller
    case 1
        [rgbImage,real_robot] = nonlinear_mpc(image,state_robot,path,scale,goal,dt);
    case 2
        [rgbImage,real_robot] = input_output_linearization(image,state_robot,path,scale,goal,dt);
    case 3
        [rgbImage,real_robot] = approximate_linearization(image,state_robot,path,scale,goal,dt);
    case 4
        [rgbImage,real_robot] = nonlinear_lyapunov(image,state_robot,path,scale,goal,dt)
end 

%% plotting
figure(); plot(path(:,1),path(:,2)); hold on; plot(real_robot(:,1),real_robot(:,2))
%figure(); J = imrotate(rgbImage,90);  J = imresize( J , 5); imshow(J);
