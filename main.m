close all;
clear;
set(groot,'defaulttextinterpreter','latex'); 
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

global robot obstacles tree;

robot.dim       = 2;        % Robot dimensionality in workspace
robot.model     = 1;        % Model of robotic manipulator (See details in 'Initialization')
obstacles.model = 2;        % Model of obstacles (1: spherical, 2: boxed)
obstacles.type  = 2;        % See details bellow
obstacles.step  = 0.02;     % obstacles moves by the size of this step
Initialization;             % Initialization of the robot and obstacles
DrawRobot(robot.q_init, 'magenta', 0.3, 3, 20);     % The library 'geom3d' must be included. 
DrawRobot(robot.q_goal, 'green', 0.3, 3, 20);       % You can find it at https://github.com/mattools/matGeom
% DrawObstacles(0.05);

% ALG = RRT();  ALG = ALG.Run();
% ALG = RRTC(); ALG = ALG.Run();
% ALG = RBT();  ALG = ALG.Run();
% ALG = RGBT(); ALG = ALG.Run();
% ALG = RGBMT(0.1, 1000, 0.01, 0);  ALG = ALG.Run(); ALG.cost_opt
% ALG = RRTx(0.1, 1000);

% DrawDetails(ALG.path, 0.1);

ALG = DRGBT();  ALG = ALG.Run();
% ALG = RRTx(0.1, 2000);
