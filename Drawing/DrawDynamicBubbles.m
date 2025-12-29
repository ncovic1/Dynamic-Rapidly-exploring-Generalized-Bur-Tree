% close all;
% clear;

set(groot,'defaulttextinterpreter','latex'); 
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

q_init = [-2; -2];        % [-pi/4; 0] [1; 2] [2; -1] [-2; -2] ('b', 'r', 'g', 'magenta')
q_dot = [2; 0];
q_ddot = [0; 0];
t_f = 1;  % Usvojimo
r_remote = 2;
v_obs = 1;
max_num_layers = 5;
color = 'magenta';

figure(2);
global robot obstacles;
robot.dim       = 2;        % Robot dimensionality in workspace
robot.model     = 1;        % Model of robotic manipulator (See details in 'Initialization')
obstacles.model = 2;        % Model of obstacles (1: spherical, 2: boxed)
obstacles.type  = 2;        % See details bellow
Initialization;             % Initialization of the robot and obstacles
% DrawObstacles(0.02, 4);
DrawRobot(q_init, color, 0.3, 2, 15);

[d_c, planes] = GetDistance(q_init);
R = computeEnclosingRadii(q_init);
q_border = [];

for fi = linspace(-pi, pi, 50)
    q_f = q_init + [r_remote * cos(fi); r_remote * sin(fi)];
    q_f_dot = [0; 0];
    q_f_ddot = [0; 0];

    f = q_init;
    e = q_dot;
    d = q_ddot / 2;
    c = -((3*d - q_f_ddot/2)*t_f^2 + (6*e + 4*q_f_dot)*t_f + 10*(f - q_f)) / t_f^3;
    b = -(3*c*t_f^2 + (3*d + q_f_ddot/2)*t_f + 2*(e - q_f_dot)) / (2*t_f^3);
    a = -(6*b*t_f^2 + 3*c*t_f + d - q_f_ddot/2) / (10*t_f^3);
    
    %% Style 1: Plot all points inside of dynamic bubble
    q_free = [];
    q_occ = [];
    var = true;
    d_c0 = d_c;
    R0 = R;
    q0 = q_init;
    t0 = 0;
    num_layers = 0;
    for t = 0:0.0001:t_f
        q = a*t.^5 + b*t.^4 + c*t.^3 + d*t.^2 + e*t + f;
        rho_robot = R0(:,end)' * abs(q - q0);        
        if rho_robot < d_c0 - v_obs * (t - t0) && var
            q_free = [q_free, q];
        else
            if num_layers < max_num_layers
                q0 = q;
                t0 = t;
                d_c0 = GetDistanceUnderestimation(q, planes) - v_obs * t;
                R0 = computeEnclosingRadii(q);
                num_layers = num_layers + 1;
                plot(q(1), q(2), 'k.', 'MarkerSize', 5); hold on; drawnow;
            end
            
            if num_layers == max_num_layers || d_c0 < 0
                q_occ = [q_occ, q];
                var = false;
            else
                q_free = [q_free, q];
            end
        end
    end
    
    plot(q_free(1,:), q_free(2,:), 'Color', color, 'LineWidth', 1); hold on; %drawnow;
%     if ~isempty(q_occ)
%         plot(q_occ(1,:), q_occ(2,:), 'r-', 'LineWidth', 1); hold on; %drawnow; 
%     end

    %% Style 2: Plot only the border of dynamic bubble
%     t = 0:0.01:t_f;
%     q = a*t.^5 + b*t.^4 + c*t.^3 + d*t.^2 + e*t + f;
%     plot(q(1,:), q(2,:), 'r-', 'LineWidth', 1); hold on;
    
%     for t = 0:0.0001:t_f
%         q = a*t.^5 + b*t.^4 + c*t.^3 + d*t.^2 + e*t + f;
%         rho_robot = R(:,end)' * abs(q - q_init);        
%         if rho_robot > d_c - v_obs * t
%             q_border = [q_border, q];
% %             plot(q(1), q(2), 'k.', 'MarkerSize', 5); hold on; %drawnow;
%             break;
%         end
%     end
end

% plot(q_border(1,:), q_border(2,:), 'Color', color, 'LineWidth', 1); hold on; %drawnow;
% plot(q_init(1), q_init(2), 'k.', 'MarkerSize', 15); hold on; %drawnow;


function R = computeEnclosingRadii(q)
    global robot;
    
    [xyz, ~] = DirectKinematics(robot, q);
    R = zeros(robot.N_DOF, robot.N_DOF+1);
    for i = 1 : robot.N_DOF
		for j = i+1 : robot.N_DOF+1
			R(i, j) = norm(xyz(:,j) - xyz(:,i)) + robot.radii(j-1);
        end
    end
end

