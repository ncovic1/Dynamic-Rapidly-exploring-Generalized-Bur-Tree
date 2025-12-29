% close all;
% clear;

set(groot,'defaulttextinterpreter','latex'); 
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

% fig = figure();
% fig.WindowState = 'maximized';
% set(gca,'FontSize',14);
% subplot(1,2,1); set(gca,'FontSize',14); subplot(1,2,2); set(gca,'FontSize',14);

global robot obstacles tree graphics_WS graphics_CS graphics_local;

% global writerObj;
% delete 'RGBMTStar_planar10dof_scenario1_path.mp4';
% writerObj = VideoWriter('RGBMTStar_planar10dof_scenario1_path', 'MPEG-4'); % Name it.
% writerObj.FrameRate = 10;   % How many frames per second.
% writerObj.Quality = 100;    % Quality of the video
% open(writerObj);

robot.dim       = 3;        % Robot dimensionality in workspace
robot.model     = 2;        % Model of robotic manipulator (See details in 'Initialization')
obstacles.model = 2;        % Model of obstacles (1: spherical, 2: boxed)
obstacles.type  = 1;        % See details bellow
obstacles.step  = 0.02;     % obstacles moves by the size of this step
Initialization;             % Initialization of the robot and obstacles
DrawRobot(robot.q_init, 'magenta', 0.3, 3, 20);     % The library 'geom3d' must be included for 3D robot. 
DrawRobot(robot.q_goal, 'green', 0.3, 3, 20);       % You can find it at https://github.com/mattools/matGeom
DrawObstacles(0.05, 4);

% ALG = RRT();
% ALG = RRT_Connect();
% ALG = RBT_Connect();
ALG = RGBT_Connect();
% ALG = RGBMT_star(0.1, 3000, 0.03, inf);

ALG = ALG.Run();
DrawDetails(ALG.path, 10, 0);

% ALG = DRGBT();  ALG = ALG.Run();
% ALG = RRTx(0.1, 3000);

% close(writerObj);