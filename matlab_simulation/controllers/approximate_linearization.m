function [outputArg1,outputArg2] = approximate_linearization(image,state_robot,path,scale,goal,dt)
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


real_robot = [state_robot(1),state_robot(2),state_robot(3),0,0];
k1 = 10;
k2 = 10;
k3 = 5;
size_path = size(path);
%main loop control
for d = 2:size_path(1)-1
    if(d == size_path(1))
        break;
    end
    near_node = path(d,:);


    e1 = cos(state_robot(3))*(near_node(1) - state_robot(1)) + sin(state_robot(3))*(near_node(2) - state_robot(2));
    e2 = -sin(state_robot(3))*(near_node(1) - state_robot(1)) + cos(state_robot(3))*(near_node(2) - state_robot(2));
    e3 = near_node(3) - state_robot(3);
    
    u1 = -k1*e1;
    u2 = -k2*e2 - k3*e3;
    
    v = near_node(5)*cos(e3) - u1;
    w = near_node(6) - u2;
    

    %integration
    state_robot(1) = state_robot(1) + v*cos(state_robot(3))*dt;
    state_robot(2) = state_robot(2) + v*sin(state_robot(3))*dt; 
    state_robot(3) = state_robot(3) + w*dt;
    
    
    %draw path and inflated robot
    x = near_node(1);
    y = near_node(2);

    
  
    rgbImage(int16(x*scale)+1,int16(y*scale)+1,1) = 0;
    rgbImage(int16(x*scale)+1,int16(y*scale)+1,2) = 0;
    rgbImage(int16(x*scale)+1,int16(y*scale)+1,3) = 255;
    
    real_robot = vertcat(real_robot,[state_robot(1),state_robot(2),state_robot(3),v,w]);

outputArg1 = rgbImage;
outputArg2 = real_robot;
end

