clear all; close all; clc;

%% load path
addpath(genpath('./../../maps'))
%addpath('./planners/A_star')

%% load map and set resolution
image = imread('simple_walls_map.pgm');
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

path = planning_fun_A_star(state_robot,dt,[3,3],goal,image,resolution,max_iteration);
