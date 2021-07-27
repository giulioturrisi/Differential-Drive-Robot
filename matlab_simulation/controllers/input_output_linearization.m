function [outputArg1,outputArg2] = input_output_linearization(image,state_robot,path,scale,goal,dt)
%grey to rgb mab
rgbImage = cat(3, image, image, image);
%START
rgbImage(int16(state_robot(1)*scale)+1,int16(state_robot(2)*scale+1),1) = 0;
rgbImage(int16(state_robot(1)*scale)+1,int16(state_robot(2)*scale+1),2) = 255;
rgbImage(int16(state_robot(1)*scale)+1,int16(state_robot(2)*scale+1),3) = 0;
%GOAL
rgbImage(int16(goal(1)*scale)+1,int16(goal(2)*scale+1),1) = 255;
rgbImage(int16(goal(1)*scale)+1,int16(goal(2)*scale+1),2) = 0;
rgbImage(int16(goal(1)*scale)+1,int16(goal(2)*scale+1),3) = 0;


real_robot = [state_robot(1),state_robot(2),state_robot(3)];
k1 = 10;

size_path = size(path);
%main loop control
for d = 2:size_path(1)-1
    if(d == size_path(1))
        break;
    end
    near_node = path(d,:);

        
    %input-output lin
    x_vel = (near_node(1) - path(d - 1,1))/dt;
    y_vel = (near_node(2) - path(d - 1,2))/dt;
    u1_io = k1*0.5*(near_node(1) - state_robot(1)) + x_vel;
    u2_io = k1*0.5*(near_node(2) - state_robot(2)) + y_vel;
    v = cos(state_robot(3))*u1_io + sin(state_robot(3))*u2_io;
    w = -sin(state_robot(3))*u1_io/0.02 + cos(state_robot(3))*u2_io/0.02;
    
    
    %integration
    state_robot(1) = state_robot(1) + v*cos(state_robot(3))*dt;
    state_robot(2) = state_robot(2) + v*sin(state_robot(3))*dt; 
    state_robot(3) = state_robot(3) + w*dt;
    
    if(isnan(real_robot(end,1)))
        disp("nan")
    end
    
    %draw path and inflated robot
    x = near_node(1);
    y = near_node(2);

    
    rgbImage(int16(x*scale)+1,int16(y*scale)+1,1) = 0;
    rgbImage(int16(x*scale)+1,int16(y*scale)+1,2) = 0;
    rgbImage(int16(x*scale)+1,int16(y*scale)+1,3) = 255;
    
    real_robot = vertcat(real_robot,[state_robot(1),state_robot(2),state_robot(3)]);

outputArg1 = rgbImage;
outputArg2 = real_robot;
end
