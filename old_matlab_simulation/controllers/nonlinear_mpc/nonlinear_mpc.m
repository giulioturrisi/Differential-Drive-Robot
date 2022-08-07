function [outputArg1,outputArg2] = nonlinear_mpc(image,state_robot,path,scale,goal,dt)

%% define NMPC
N = 50;
T = 5;
nlp_solver = 'sqp'; % sqp, sqp_rti
qp_solver = 'partial_condensing_hpipm';
qp_solver_cond_N = 5;
% integrator model
model_sim_method = 'erk';
model_sim_method_num_stages = 1;
model_sim_method_num_steps = 2;
model = differential_drive_model;
nx = model.nx;
nu = model.nu;
%% model to create the solver
ocp_model = acados_ocp_model();
model_name = 'differential_drive';

%% acados ocp model
ocp_model.set('name', model_name);
ocp_model.set('T', T);
% symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_xdot', model.sym_xdot);

% nonlinear-least squares cost
ocp_model.set('cost_type', 'nonlinear_ls');
ocp_model.set('cost_type_e', 'nonlinear_ls');

ocp_model.set('cost_expr_y', model.cost_expr_y);
ocp_model.set('cost_expr_y_e', model.cost_expr_y_e);

W_x = diag([1e3, 1e3, 1e-1]);
W_u = diag([1e0,1e0]);
W = blkdiag(W_x, W_u);
ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', model.W_e);

% dynamics
ocp_model.set('dyn_type', 'explicit');
ocp_model.set('dyn_expr_f', model.expr_f_expl);

% constraints
ocp_model.set('constr_type', 'auto');
ocp_model.set('constr_expr_h', model.expr_h);
U_max = [10,3];
ocp_model.set('constr_lh', -U_max); % lower bound on h
ocp_model.set('constr_uh', U_max);  % upper bound on h
ocp_model.set('constr_x0', state_robot(1:3));

%% acados ocp set opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
%ocp_opts.set('shooting_nodes', shooting_nodes);

ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', model_sim_method);
ocp_opts.set('sim_method_num_stages', model_sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', model_sim_method_num_steps);

ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
% ... see ocp_opts.opts_struct to see what other fields can be set

%% create ocp solver
ocp = acados_ocp(ocp_model, ocp_opts);









%% grey to rgb mab
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

size_path = size(path);
%main loop control
for d = 2:size_path(1)-1
    if(d == size_path(1))
        break;
    end
    near_node = path(d,:);

        

    
    ocp.set('constr_x0', state_robot(1:3));
    for k=0:N-1
        if(d+k >= size_path(1))
            d+k
            yref = path(size_path(1),1:3);
        else
            yref = path(d+k,1:3);
        end
        yref = [yref,0,0];
        ocp.set('cost_y_ref', yref, k);
    end
    %yref_e(1) = p_ref(k+1);
    ocp.set('cost_y_ref_e', yref(1:3), N);
    
    % solve
    ocp.solve();
    % get solution
    u0 = ocp.get('u', 0);
    v = u0(1);
    w = u0(2);
    

    
    
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
    
    real_robot = vertcat(real_robot,[state_robot(1),state_robot(2),state_robot(3),v,w]);
    
    %to check evolution linearization point
    %real_robot = vertcat(real_robot,[y1,y2,state_robot(3)]);

outputArg1 = rgbImage;
outputArg2 = real_robot;

end

