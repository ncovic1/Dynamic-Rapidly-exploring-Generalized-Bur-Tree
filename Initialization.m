function Initialization
global robot obstacles;

eval(['InitRobot_',num2str(robot.dim),num2str(robot.model)]);
eval(['InitObs_',num2str(robot.dim),num2str(robot.model),num2str(obstacles.model),num2str(obstacles.type)]); 


function InitRobot_21()
% Two-segment planar robotic manipulator

    robot.N_DOF = 2;        % Number of robot degrees of freedom
    robot.N_links = 2;      % Number of robot links  
    robot.DH_table = [      % DH table: d, a, alpha
        0 1 0
        0 1 0];             
    robot.offsets = [0; 0]; % Offset of angle theta
    robot.range = [-pi, pi; -pi, pi];   % Range/limit for each joint
    robot.radii = [0; 0];   % Radii of all enclosing cylinder  
    robot.weights = [2; 1]; % Weights in criterial function for optimal planning
end

function InitRobot_22()
% Four-segment planar robotic manipulator

    robot.N_DOF = 4;        % Number of robot degrees of freedom
    robot.N_links = 4;      % Number of robot links
    robot.DH_table = [      % DH table: d, a, alpha
        0 1 0
        0 1 0
        0 1 0
        0 1 0];
    robot.offsets = [0; 0; 0; 0];   % Offsets of angle theta
    robot.range = [-pi*ones(robot.N_DOF,1), pi*ones(robot.N_DOF,1)];   % Range for each segment
    robot.radii = [0; 0; 0; 0];     % Radii of all enclosing cylinder  
    robot.weights = [4; 3; 2; 1];   % Weights in criterial function for optimal planning
end

function InitRobot_23()
% Ten-segment planar robotic manipulator
    
    robot.N_DOF = 10;        
    robot.N_links = 10;    
    robot.DH_table = [zeros(robot.N_DOF,1), ones(robot.N_DOF,1), zeros(robot.N_DOF,1)];             
    robot.offsets = zeros(robot.N_DOF,1);
    robot.range = [-pi*ones(robot.N_DOF,1), pi*ones(robot.N_DOF,1)];
    robot.radii = zeros(robot.N_DOF,1);    
    robot.weights = [robot.N_links:-1:1]';
end

function InitRobot_31()
% Anthropomorphic arm with spherical wrist
    
    robot.DH_table = [      % d, a, alpha
        1  0  pi/2
        0  1  0
        0  0  pi/2
        1  0  -pi/2
        0  0  pi/2
        1  0  0];             
    robot.offsets = zeros(6,1);
    robot.N_DOF = 6;     % Number of robot degrees of freedom
    robot.N_links = 4;   % Number of robot links 
    robot.range = [-pi*ones(robot.N_DOF,1), pi*ones(robot.N_DOF,1)];
    robot.radii = zeros(4,1);    
end

function InitRobot_32()
% xArm6
    
    robot.DH_table = [      % d, a, alpha
        0.2670       0   -pi/2
             0  0.2895       0
             0  0.0775   -pi/2
        0.3425       0    pi/2
             0  0.0760   -pi/2
        0.0970       0       0];             
    robot.offsets = [0; -1.3849179; 1.3849179; 0; 0; 0];
    robot.N_DOF = 6;     % Number of robot degrees of freedom
    robot.N_links = 6;   % Number of robot links
    robot.range = [-pi*ones(robot.N_DOF,1), pi*ones(robot.N_DOF,1)];
    robot.radii = [0.1552; 0.1881; 0.1623; 0.1623; 0.106; 0.076]/2; % Radii of all enclosing cylinders 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function InitObs_2111()
    obstacles.loc = [1.5 0.7 0 0.4]';    % [x_c y_c z_c r]
    robot.q_init = [-2;-2.5]; 
    robot.q_goal = [2; 2.5];
end

function InitObs_2112()
    obstacles.loc = [1.5 0.7 0 0.4; 1.5 -0.7 0 0.4]';
    robot.q_init = [-2;-2.5]; 
    robot.q_goal = [2; 2.5];
end

function InitObs_2113()
    obstacles.loc = [1.5 0.7 0 0.4; 1.5 -0.7 0 0.4; -1.5 -0.7 0 0.4; -1.5 0.7 0 0.4]';
    robot.q_init = [-2;-2.5]; 
    robot.q_goal = [2; 2.5];
end

function InitObs_2114()
    obstacles.loc = [1.5 0.7 0 0.4; 1.5 -0.7 0 0.4; -1.5 -0.7 0 0.4; -1.5 0.7 0 0.4;  
                     0.6 1.5 0 0.4; 0.6 -1.5 0 0.4; -0.6 1.5 0 0.4;  -0.6 -1.5 0 0.4]';
    robot.q_init = [-2;-2.5]; 
    robot.q_goal = [2; 2.5];
end

function InitObs_2115()
    obstacles.loc= [0.65 0.7 0 0.2; -0.65 0.7 0 0.2; 0.65 1.3 0 0.1; -0.65 1.3 0 0.1]';
    robot.q_init = [1.8; 1.25]; 
    robot.q_goal = [pi-1.8; -1.25];
end

function InitObs_2116()
    obstacles.loc = [-3 0 0 1; 8 8 0 3; 7 -7 0 2; -7 -8 0 2]'; 
    robot.q_init = [-2;-2.5]; 
    robot.q_goal = [2; 2.5];
end

function InitObs_2117()
    B = 5; r = rand(B,1)*sum(L)./(sum(L)+0.5)+0.5; fi = rand(B,1)*2*pi; 
    obstacles.loc = [r.*cos(fi), r.*sin(fi), rand(B,1)*0.2+0.1]';
    robot.q_init = [-2;-2.5]; 
    robot.q_goal = [2; 2.5];
end

function InitObs_2121()
    obstacles.loc = [1 0.5 0 1.5 1 0]';   % [x_min y_min z_min x_max y_max z_max]
    robot.q_init = [-2;-2.5]; 
    robot.q_goal = [2; 2.5];
end

function InitObs_2122()
    obstacles.loc = [1 0.5 0 1.5 1 0; 1 -1 0 1.5 -0.5 0]'; 
    robot.q_init = [-2;-2.5]; 
    robot.q_goal = [2; 2.5];
end

function InitObs_2123()
    obstacles.loc = [1 0.5 0 1.5 1 0; 1 -1 0 1.5 -0.5 0; -1.5 0.5 0 -1 1 0; -1.5 -1 0 -1 -0.5 0]';
    robot.q_init = [-2;-2.5]; 
    robot.q_goal = [2; 2.5];
end

function InitObs_2124()
    obstacles.loc = [];
    dim= [0.5, 0.5, 0];
    trans= [1.5, 0.7, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.5, 0.5, 0];
    trans= [1.5, -0.7, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];      
    dim= [0.5, 0.5, 0];
    trans= [-1.5, -0.7, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.5, 0.5, 0];
    trans= [-1.5, 0.7, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.5, 0.5, 0];
    trans= [0.6, 1.5, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.5, 0.5, 0];
    trans= [0.6, -1.5, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.5, 0.5, 0];
    trans= [-0.6, 1.5, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.5, 0.5, 0];
    trans= [-0.6, -1.5, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
      
    robot.q_init = [-2; -2.5]; 
    robot.q_goal = [2; 2.5];
    
end

function InitObs_2125()
    obstacles.loc = [];
    dim= [0.3, 0.3, 0];
    trans= [0.65, 0.7, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.3, 0.3, 0];
    trans= [-0.65, 0.7, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];      
    dim= [0.1, 0.1, 0];
    trans= [0.65, 1.3, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.1, 0.1, 0];
    trans= [-0.65, 1.3, 0];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    
    robot.q_init = [1.8; 1.25]; 
    robot.q_goal = [pi-1.8; -1.25];
end

function InitObs_2221()
    obstacles.loc = [-1.2 0.5 0 -0.8 2 0; -1.2 -2 0 -0.8 -0.5 0; -2.5 1.5 0 -1.2 2 0; 
        -2.5 -2 0 -1.2 -1.5 0; -4 -2 0 -2.5 2 0]';   % [x_min y_min z_min x_max y_max z_max]
    robot.q_init = [-2;-2.5; pi/2; pi/2]; 
    robot.q_goal = [2; 2.5; -pi/2; -pi/2];
end

function InitObs_2321()
    obstacles.loc = [
        -2 4 0 -1 6 0; -2 -6 0 -1 -4 0; 
        -8 5 0 -2 6 0; -8 -6 0 -2 -5 0; -8 -6 0 -7 6 0]';
    robot.q_init = [-0.99*pi; linspace(pi/12,0.5,4)'; -linspace(pi/12,1.5,5)']; 
    robot.q_goal = [0.99*pi; -linspace(pi/12,0.5,4)'; linspace(pi/12,1.5,5)'];
end

function InitObs_3111()
    obstacles.loc = [1.5 0.7 1 0.5]';
    robot.q_init = [-2*pi/3; -2*pi/3; pi/2; 0; 0; 0]; 
    robot.q_goal = [pi/3; 2*pi/3; pi/2; 0; 0; 0];   
end

function InitObs_3112()
    obstacles.loc = [1.5 0.7 1 0.5; -1.5 0.7 1 0.5]';
    robot.q_init = [-2*pi/3; -2*pi/3; pi/2; 0; pi; 0]; 
    robot.q_goal = [pi/3; 2*pi/3; pi/2; 0; pi; 0];                 
end

function InitObs_3113()
    obstacles.loc = [-3 0 0 0.5; 2 2 2 0.5; 2 -2 1 1; -1 -2 2 0.5]'; 
    robot.q_init = [-pi/2; pi/6; pi/4; 0; 5*pi/4; 0]; 
    robot.q_goal = [pi/2; pi/6; pi/4; 0; 5*pi/4; 0];                
end

function InitObs_3114()
    obstacles.loc = [1.5 0.7 0 0.4; 1.5 -0.7 0 0.4; -1.5 -0.7 0 0.4; -1.5 0.7 0 0.4; 0.6 1.5 0 0.4;
        -0.6 1.5 0 0.4; 0.6 -1.5 0 0.4; -0.6 -1.5 0 0.4]';
    robot.q_init = [-2*pi/3; -2*pi/3; pi/2; 0; pi; 0]; 
    robot.q_goal = [pi/3; 2*pi/3; pi/2; 0; pi; 0];     
end

function InitObs_3121()
    obstacles.loc = [1 1 0, 2 2 2; 1 -2 0, 2 -1 2; -2 1 0, -1 2 2; -2 -2 0, -1 -1 2]';
    robot.q_init = [-2*pi/3; -2*pi/3; pi/2; 0; 0; 0]; 
    robot.q_goal = [pi/3; 2*pi/3; pi/2; 0; 0; 0];   
end

function InitObs_3122()
    obstacles.loc = [-2 2 -2, 2 4 0; -2 -4 2, 2 -2 4]';
    robot.q_init = [-2*pi/3; -2*pi/3; pi/2; 0; pi; 0]; 
    robot.q_goal = [pi/3; 2*pi/3; pi/2; 0; pi; 0];  
end

function InitObs_3123()
    obstacles.loc = [-1 0.5 0.5, 1 1 1.5; 0.5 -1 0.5, 1 1 1.5; -1 -1 0.5, 1 -0.5 1.5; -1 -1 0.5, -0.5 1 1.5]';
    robot.q_init = [-2*pi/3; -2*pi/3; pi/2; 0; pi; 0]; 
    robot.q_goal = [pi/3; 2*pi/3; pi/2; 0; pi; 0];   
end

function InitObs_3124()
    obstacles.loc = [-1 0.5 0.5, 1 1 1.5; 0.5 -1 0.5, 1 1 1.5; -1 -1 0.5, 1 -0.5 1.5; -1 -1 0.5, -0.5 1 1.5;
        -3 -3 3, -1 -1 3.1; -3 -3 -1.1, -1 -1 -1]';
    robot.q_init = [-2*pi/3; -2*pi/3; pi/2; 0; pi; 0]; 
    robot.q_goal = [pi/3; 2*pi/3; pi/2; 0; pi; 0];
end

function InitObs_3125()
    obstacles.loc = [-1 1 0.5, -0.5 4 2.5; 0.5 1 0.5, 1 4 2.5; -0.5 1 2, 0.5 4 2.5; -0.5 1 0.5, 0.5 4 1;
        -1 -4 0.5, -0.5 -2 2.5; 0.5 -4 0.5, 1 -2 2.5; -0.5 -4 2, 0.5 -2 2.5; -0.5 -4 0.5, 0.5 -2 1]';
    robot.q_init = [-pi/2; pi/6; pi/3; 0; pi; 0]; 
    robot.q_goal = [pi/2; pi/6; pi/3; 0; pi; 0]; 
end

function InitObs_3211()
    obstacles.loc = [1.5 0.7 1 0.5]';
    robot.q_init = [-2*pi/3; -2*pi/3; pi/2; 0; 0; 0]; 
    robot.q_goal = [pi/3; 2*pi/3; pi/2; 0; 0; 0];   
end

function InitObs_3221()
    obstacles.loc = [0.3 0.3 0, 0.8 0.8 1; 0.3 -0.8 0, 0.8 -0.3 1; -0.8 0.3 0, -0.3 0.8 1; -0.8 -0.8 0, -0.3 -0.3 1]';
    robot.q_init = [pi/2; pi/2; -3*pi/4; 0; 0; 0]; 
    robot.q_goal = [-pi/2; 0; -3*pi/4; 0; 0; 0];   
end

function InitObs_3222()
    obstacles.loc = [-0.5 0 -0.5, 0.5 1 0.5; -0.5 -1 0, 0.5 0 1]';
    robot.q_init = [pi/2; pi/2; -3*pi/4; 0; 0; 0]; 
    robot.q_goal = [-pi/2; 0; -3*pi/4; 0; 0; 0]; 
end

function InitObs_3223()
    obstacles.loc = [-1 0.5 0.5, 1 1 1.5; 0.5 -1 0.5, 1 1 1.5; -1 -1 0.5, 1 -0.5 1.5; -1 -1 0.5, -0.5 1 1.5]';
    robot.q_init = [-2*pi/3; -2*pi/3; pi/2; 0; pi; 0]; 
    robot.q_goal = [pi/3; 2*pi/3; pi/2; 0; pi; 0];   
end

function InitObs_3224()
    obstacles.loc = [-1 0.5 0.5, 1 1 1.5; 0.5 -1 0.5, 1 1 1.5; -1 -1 0.5, 1 -0.5 1.5; -1 -1 0.5, -0.5 1 1.5;
        -3 -3 3, -1 -1 3.1; -3 -3 -1.1, -1 -1 -1]';
    robot.q_init = [-2*pi/3; -2*pi/3; pi/2; 0; pi; 0]; 
    robot.q_goal = [pi/3; 2*pi/3; pi/2; 0; pi; 0];
end

function InitObs_3225()
%     obstacles.loc = [-0.35 0.5 0, -0.25 1 0.7; 0.25 0.5 0, 0.35 1 0.7; -0.25 0.5 0.6, 0.25 1 0.7; -0.25 0.5 0, 0.25 1 0.1;
%         -0.35 -1 0, -0.25 -0.6 0.7; 0.25 -1 0, 0.35 -0.6 0.7; -0.25 -1 0.6, 0.25 -0.6 0.7; -0.25 -1 0, 0.25 -0.6 0.1]';
    robot.q_init = [pi/2; pi/2-0.1859; pi+0.1859; pi/2; 0; 0]; 
    robot.q_goal = [-pi/2; pi/2-0.1859; pi+0.1859; pi/2; 0; 0];  
    
    obstacles.loc = [];
    dim= [0.1, 0.5, 0.7];
    trans= [-0.3, 0.75, 0.35];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.1, 0.5, 0.7];
    trans= [0.3, 0.75, 0.35];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];      
    dim= [0.5, 0.5, 0.1];
    trans= [0, 0.75, 0.65];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.5, 0.5, 0.1];
    trans= [0, 0.75, 0.05];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.1, 0.4, 0.7];
    trans= [-0.3, -0.8, 0.35];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.1, 0.4, 0.7];
    trans= [0.3, -0.8, 0.35];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.5, 0.4, 0.1];
    trans= [0, -0.8, 0.65];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
    dim= [0.5, 0.4, 0.1];
    trans= [0, -0.8, 0.05];
    obstacles.loc = [obstacles.loc, [trans-dim/2, trans+dim/2]'];
end


end
