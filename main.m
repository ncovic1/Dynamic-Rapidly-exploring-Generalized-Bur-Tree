close all;
set(groot,'defaulttextinterpreter','latex'); 
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

global robot obstacles;

Initialization;
DrawInitAndGoalConfig(robot);   % The library 'geom3d' must be included. You can find it at https://github.com/mattools/matGeom
% [~, ~] = DrawObstacles(obstacles, robot, 0.08);

% ALG = RRT(); ALG = ALG.Run();
% ALG = RRTC(); ALG = ALG.Run();
% ALG = RBT(); ALG = ALG.Run();
% ALG = RGBT(); ALG = ALG.Run();
ALG = DRGBT(); ALG = ALG.Run();
% ALG = RRTx();

% DrawDetails(ALG.tree, ALG.pointers, ALG.path, robot);
