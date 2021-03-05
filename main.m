close all;
set(groot,'defaulttextinterpreter','latex'); 
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

global robot obstacles;

Initialization;
DrawInitAndGoalConfig(robot);
% [~, ~] = DrawObstacles(obstacles, robot, 0.08);

% ALG = RRT(); ALG = ALG.Run();
% ALG = RRTC(); ALG = ALG.Run();
% ALG = RBT(); ALG = ALG.Run();
% ALG = RGBT(); ALG = ALG.Run();
ALG = DRGBT(); ALG = ALG.Run();
% ALG = RRTx();

% DrawDetails(ALG.tree, ALG.pointers, ALG.path, robot);
