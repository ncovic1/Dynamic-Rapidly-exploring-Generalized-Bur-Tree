% close all;
% global robot obstacles;
% Initialization;
% 
% N_test = 100;
% filename = [num2str(robot.dim),num2str(robot.model),num2str(obstacles.model),num2str(obstacles.type),...
%     '_',num2str(obstacles.step),'_dio5.mat'];
% 
% for i = 1:N_test
%     Initialization;
%     disp(' '); disp(['Testing number: ', num2str(i), ' of ', num2str(N_test)]);
%     disp(' '); disp('DRGBT: ');
%     ALG1 = DRGBT(); ALG1 = ALG1.Run();
%     Initialization;
%     disp(' '); disp('RRTx: ');
%     ALG2 = RRTx();
%     collisions(i,:) = [ALG1.collision, ALG2.collision];
%     time_iter(i,:) = [ALG1.T_iter, ALG2.T_iter];
%     time_alg(i,:) = [ALG1.T_alg, ALG2.T_alg];
%     lengths(i,:) = [GetPathLength(ALG1.path, ALG1.collision), GetPathLength(ALG2.path, ALG2.collision)];
%     disp('-------------------------------------------------------------------');
%     save(filename, 'robot','obstacles','collisions','time_iter','time_alg','lengths','N_test','ALG1','ALG2');
% end

N_test = size(collisions,1);
disp('Algorithm success in [%]: ');
disp((1-sum(collisions)/N_test)*100);
disp('Mean runtime of a single iteration in [ms]: ');
disp(sum(time_iter)/N_test*1000);
disp('Mean runtime of algorithm in [s]: ');
disp(GetMeanAlgTime(time_alg, collisions));
disp('Mean path length in C-space: ');
disp(sum(lengths)./sum(~collisions));


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

