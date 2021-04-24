%x y theta x_dot y_dot theta_dot
state_robot = [4 10 0 0 0 0]
dt = 0.5;

image = imread('map.pgm');
%imshow(image)
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);
%show(map)
%%N.B 0.0039 è free, sopra tutto occupato o incerto

RRT = RRT_primitives(state_robot,dt,[20,20],[10,8],image)
%main for RRT
for j = 1:100000
    j
    %RRT.nodes
    desired_node = RRT.sample();
    near_index = RRT.find_nearest(desired_node);
    new_node = RRT.choose_primitives(near_index,desired_node);
    %%check collision
    good = RRT.check_collision(new_node);
    %good = 1;
    if(good == 1)
        RRT.add_nodes(new_node);
    end
    finish = RRT.check_goal(new_node);
    
    if(finish == 1)
        path = RRT.take_path(new_node(4));
        break;
    end;
end




k1 = 15;
k2 = 15;
k3 = 1;
state_robot(1) = state_robot(1);% + 0.1;
state_robot(2) = state_robot(2);% - 0.1;
state_robot(3) = state_robot(3);% + 0.8;
real_robot = [state_robot(1),state_robot(2),state_robot(3)];

size_path = size(path);
xq = 0:0.01:size_path(1);
path_x = interp1(path(:,1),xq,'spline');
path_y = interp1(path(:,2),xq,'spline');
path_theta = interp1(path(:,3),xq,'spline');
path_v = interp1(path(:,5),xq,'spline');
path_w = interp1(path(:,6),xq,'spline');


path = [path_x' path_y' path_theta' path_theta' path_v' path_w'];
size_path = size(path);

dt = 0.01;
for d = 1:size_path(1)
    if(size_path(1) - d == 0)
        break;
    end
    near_node = path(size_path(1) - d,:);
    %state_robot(1) = state_robot(1) + near_node(5)*cos(state_robot(3))*dt;
    %state_robot(2) = state_robot(2) + near_node(5)*sin(state_robot(3))*dt; 
    %state_robot(3) = state_robot(3) + near_node(6)*dt;
    
    e1 = cos(state_robot(3))*(near_node(1) - state_robot(1)) + sin(state_robot(3))*(near_node(2) - state_robot(2));
    e2 = -sin(state_robot(3))*(near_node(1) - state_robot(1)) + cos(state_robot(3))*(near_node(2) - state_robot(2));
    e3 = near_node(3) - state_robot(3);
    
    u1 = -k1*e1;
    u2 = -k2*near_node(5)*(sin(e3)/e3)*e2 - k3*e3;
    
    v = near_node(5)*cos(e3) - u1;
    w = near_node(6) - u2;
    
    state_robot(1) = state_robot(1) + v*cos(state_robot(3))*dt;
    state_robot(2) = state_robot(2) + v*sin(state_robot(3))*dt; 
    state_robot(3) = state_robot(3) + w*dt;
    
    image(int16(state_robot(1)*20),int16(state_robot(2)*20)) = 0;
    real_robot = vertcat(real_robot,[state_robot(1),state_robot(2),state_robot(3)]);
end

plot(path(:,1),path(:,2));
hold on;
plot(real_robot(:,1),real_robot(:,2))

imshow(image);

%xq = 0:0.01:size_path(1);
%path_x = interp1(path(:,1),xq,'spline');
%path_y = interp1(path(:,2),xq,'spline');