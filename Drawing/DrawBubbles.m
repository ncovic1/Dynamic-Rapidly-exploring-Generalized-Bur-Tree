global robot obstacles;

robot.dim       = 2;        % Robot dimensionality in workspace
robot.model     = 1;        % Model of robotic manipulator (See details in 'Initialization')
obstacles.model = 2;        % Model of obstacles (1: spherical, 2: boxed)
obstacles.type  = 1;        % See details bellow
Initialization;             % Initialization of the robot and obstacles
DrawObstacles(0.05, 4);

ALG = RBT_Connect();
num_spines = 4;
num_layers = 5;

num = 1;
while num <= 10
    q_rand = ALG.GetRandomNode();
    if CheckCollision(q_rand)
        continue;
    end
    
    [d_c, planes] = GetDistance2(q_rand);
    DrawRobot(q_rand, 'k', 0.3, 1, 10);
    DrawBubble(ALG, q_rand, min(d_c));
    DrawExpandedBubble(ALG, q_rand, d_c);
    
%     delta_q = [[ALG.delta; 0], [0; ALG.delta], [-ALG.delta; 0], [0; -ALG.delta]];
%     for i = 1:num_spines
% %         q_e = q_rand + ALG.GetRandomNode();
% %         q_e = ALG.SaturateSpine(q_rand, q_e, ALG.delta);
%         q_e = q_rand + delta_q(:,i);
%         
%         q_new = q_rand;
%         d_c_temp = min(d_c);
%         for j = 1:num_layers
%             [q_new, reached] = ALG.GenerateSpine(q_new, q_e, d_c_temp);
%             d_c_temp = GetDistanceUnderestimation(q_new, planes);
% %             [d_c_temp, ~] = GetDistance(q_new);
%             DrawBubble(ALG, q_new, d_c_temp);
%             DrawRobot(q_new, 'r', 0.3, 1, 10);
%             if reached
%                 break;
%             end  
%         end
%         plot([q_rand(1), q_new(1)], [q_rand(2), q_new(2)], 'Color', [0.7,0.7,0.7]); hold on;
%         
% %         q_new = q_rand;
% %         d_c_temp = d_c;
% %         for j = 1:num_layers
% %             [q_new, reached] = ALG.GenerateSpine2(q_new, q_e, d_c_temp);
% %             d_c_temp = GetDistanceUnderestimation2(q_new, planes);
% %             DrawExpandedBubble(ALG, q_new, d_c_temp);
% %             DrawRobot(q_new, 'b', 0.3, 1, 10);
% %             if reached
% %                 break;
% %             end  
% %         end
% %         plot([q_rand(1), q_new(1)], [q_rand(2), q_new(2)], 'Color', [0.7,0.7,0.7]); hold on;
%     end
    
    drawnow;
    num = num + 1;
end

function DrawBubble(ALG, q, d_c)
    delta_q = [[ALG.delta; 0], [0; ALG.delta], [-ALG.delta; 0], [0; -ALG.delta]];
    for i = 1:4     % For 2D, four configurations are enough:
        q_e = q + delta_q(:,i);
        [q_new(:,i), ~] = ALG.GenerateSpine(q, q_e, d_c);
    end
    fill(q_new(1,:), q_new(2,:), 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'r'); hold on;
end

function DrawExpandedBubble(ALG, q, d_c)
    delta_q = [[ALG.delta; 0], [0; ALG.delta], [-ALG.delta; 0], [0; -ALG.delta]];
    for i = 1:4     % For 2D, four configurations are enough:
        q_e = q + delta_q(:,i);
        [q_new(:,i), ~] = ALG.GenerateSpine2(q, q_e, d_c);
    end
    fill(q_new(1,:), q_new(2,:), 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'b'); hold on;
end

