close all;
clear;
global robot obstacles;

robot.dim       = 3;        % Robot dimensionality in workspace
robot.model     = 2;        % Model of robotic manipulator (See details in 'Initialization')
obstacles.model = 2;        % Model of obstacles (1: spherical, 2: boxed)
obstacles.type  = 5;        % See details bellow
% obstacles.step  = 0.02;     % obstacles move by the size of this step
N_test          = 10;
N_max           = 10000;

filename = [num2str(robot.dim),num2str(robot.model),num2str(obstacles.model),num2str(obstacles.type),'_RGBMT.mat'];

for i = 1:N_test
    Initialization;
    disp(' '); disp(['Testing number: ', num2str(i), ' of ', num2str(N_test)]);
    disp(' '); disp('RGBMT: ');
    ALG1 = RGBMT(0.1, N_max, 0.01, 0);  ALG1 = ALG1.Run();
    disp(ALG1.costs(N_max));
    
%     Initialization;
%     disp(' '); disp('RRTx: ');
%     br_cvorova = inf;
%     while br_cvorova > 1.02*N_max || br_cvorova == N_max+1
%         Initialization;
%         ALG2 = RRTx(0.1, N_max);
%         br_cvorova = ALG2.i;
%     end
%     ALG2.costs(ALG2.costs < ALG2.costs(end)) = ALG2.costs(end);
%     [~, ind] = max(ALG2.costs < inf);
%     ALG2.costs(ALG2.i_start:ind-1) = ALG2.costs(ind);
%     disp(ind-ALG2.i_start);
%     disp(ALG2.costs(N_max));
    costs{i} = {ALG1.costs'};
%     collisions(i,:) = [ALG1.collision, ALG2.collision];
%     time_iter(i,:) = [ALG1.T_iter, ALG2.T_iter];
    time_alg(i,:) = [ALG1.T_alg];
%     lengths(i,:) = [GetPathLength(ALG1.path, ALG1.collision), GetPathLength(ALG2.path, ALG2.collision)];
    disp('-------------------------------------------------------------------');
%     save(filename, 'robot','obstacles','collisions','time_iter','time_alg','lengths','N_test','ALG2');
    save(filename, 'robot','obstacles','costs','time_alg','N_test','ALG1');
end

% N_test = size(collisions,1);
% disp('Algorithm success in [%]: ');
% disp((1-sum(collisions)/N_test)*100);
% disp('Mean runtime of a single iteration in [ms]: ');
% disp(sum(time_iter)/N_test*1000);
% disp('Mean runtime of algorithm in [s]: ');
% disp(GetMeanAlgTime(time_alg, collisions));
% disp('Mean path length in C-space: ');
% disp(sum(lengths)./sum(~collisions));

disp('Mean runtime of algorithm in [s]: ');
disp(mean(time_alg));
N_found = zeros(N_test,2);
c = zeros(N_test,2);
for i = 1:length(costs)
    plot(costs{i}{1}(1:N_max),'b'); hold on; %plot(costs{i}{2}(1:N_max),'r'); hold on;
    [~, ind1] = max(costs{i}{1} < inf);
    %[~, ind2] = max(costs{i}{2} < inf);
    N_found(i,:) = [ind1];
    c(i,:) = [costs{i}{1}(N_max)];
end
legend('RGBMT', 'RRT$^\mathrm{X}$'); 
xlabel('Node'); ylabel('Cost'); grid on;
disp('Initial path found after node: ');
disp(mean(N_found));
disp('Mean final path costs in [rad]: ');
disp(mean(c));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function path_len = GetPathLength(path, collision)
    path_len = 0;
    if ~collision
        for i = 2:size(path,2)
            path_len = path_len + norm(path(:,i)-path(:,i-1));
        end
    end
end


function time_alg_new = GetMeanAlgTime(time_alg, collisions)
    time_alg_new = zeros(1,size(collisions,2));
    for ii = 1:size(collisions,2)
        time_alg_new(ii) = sum(time_alg(~collisions(:,ii),ii))/sum(~collisions(:,ii));
    end
end


