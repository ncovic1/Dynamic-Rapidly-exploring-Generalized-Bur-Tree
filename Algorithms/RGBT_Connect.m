classdef RGBT_Connect < RBT_Connect
properties
    N_layers = 5;           % Number of layers for building a generalized bur
end

methods
    function this = RGBT_Connect(eps, N_max, N_spines, d_crit, delta, N_layers, T_max)
        if nargin > 0
            RBT_Connect(eps, N_max, N_spines, d_crit, delta);
            this.N_layers = N_layers;
            this.T_max = T_max;
        end
    end
    
    
    function this = Run(this)
        global robot tree;        
        TN = 2;  % Determines which trees is chosen, 1: from q_init; 2: from q_goal
        tree.nodes = {robot.q_init, robot.q_goal};  % Consisting of two parts, one from q_init and another from q_goal
        tree.pointers = {{0}, {0}};     % Pointing to location of parent/children in trees
        tree.distances = {NaN, NaN};    % Distance to each obstacle for each node
        tree.planes = {{NaN}, {NaN}};   % Lines/planes dividing space into two subspaces (free and "occupied")
        this.T_alg = tic;
        
        if CheckCollision(robot.q_init)
            disp('Initial robot configuration is in the collision!');
            return;
        end        
        if CheckCollision(robot.q_goal)
            disp('Goal robot configuration is in the collision!');
            return;
        end
        
        while true     
            %% GBur is generated
            TN = 3 - TN;
            q_e = this.GetRandomNode();
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_e);
            [d_c, planes] = this.Get_dc_AndPlanes(TN, q_near_p);
            if d_c > this.d_crit
                for i = 1:this.N_spines
                    q_e = q_near + this.GetRandomNode();
                    q_e = this.SaturateSpine(q_near, q_e, this.delta);
                    q_e = this.PruneSpine(q_near, q_e);
                    [q_new_list, ~] = this.GenerateGSpine(q_near, q_e, d_c, planes);
                    q_new = q_new_list(:,end);
                    q_new_p = q_near_p;
                    for j = 1:size(q_new_list,2)
                        q_new_p = UpgradeTree(TN, q_new_p, q_new_list(:,j), NaN, NaN);                        
                    end
                end

%                 A = randn(robot.N_DOF); 
%                 M = (A + A') / sqrt(2); 
%                 [V, ~] = eig(M);
% %                 fi = 2*pi*rand;
% %                 V = [cos(fi), -sin(fi); sin(fi), cos(fi)];
%                 for k = 1:robot.N_DOF
%                     for sign = [-1,1]
%                         q_e = q_near;
%                         q_e(k) = q_e(k) + sign * this.delta;
% %                         plot([q_near(1), q_e(1)], [q_near(2), q_e(2)], 'b');
%                         q_e_new = q_near + V * (q_e - q_near);
%                         [q_new_list, ~] = this.GenerateGSpine(q_near, q_e_new, d_c, planes);
%                           q_new = q_new_list(:,end);
% %                         plot([q_near(1), q_new(1)], [q_near(2), q_new(2)], 'r');
%                     end
%                 end
            else     % Distance to obstacles is less than critical
                [q_new, ~, collision] = this.GenerateEdge(q_near, q_e);     % Spine is generated using RRT                
                if ~collision
                    q_new_p = UpgradeTree(TN, q_near_p, q_new, NaN, NaN);
                else
                    q_new_p = q_near_p;
                end 
            end
            if toc(this.T_alg) > this.T_max
                return;
            end
            
            %% GBur-Connect
            TN = 3 - TN;
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_new);
            collision = false; 	% Is collision occured
            reached = false; 	% Are trees connected
            while ~collision && ~reached
                [d_c, planes] = this.Get_dc_AndPlanes(TN, q_near_p);
                if d_c > this.d_crit
                    [q_new_list, reached] = this.GenerateGSpine(q_near, q_new, d_c, planes);
                    q_near = q_new_list(:,end);
                    for j = 1:size(q_new_list,2)
                        q_near_p = UpgradeTree(TN, q_near_p, q_new_list(:,j), NaN, NaN);                        
                    end
                else
                    [q_near, reached, collision] = this.GenerateEdge(q_near, q_new);     % Spine is generated using RRT                 
                    if ~collision
                        q_near_p = UpgradeTree(TN, q_near_p, q_near, NaN, NaN);
                    end 
                end
            end
            
            this.N_iter = this.N_iter + 1;
            %disp(['RGBT: ', num2str(toc(this.T_alg))]);
            if reached   % Two trees are connected
                this.path = GetPath(q_near_p, q_new_p, TN);
                this.T_alg = toc(this.T_alg);
                disp(['The path is found in ', num2str(this.T_alg*1000), ' [ms].']);
                return;
            elseif size(tree.nodes{1},2)+size(tree.nodes{2},2) >= this.N_max || toc(this.T_alg) > this.T_max
                this.T_alg = toc(this.T_alg);
                disp('The path is not found.');
                return;
            end
            TN = 3 - TN;
        end
    end
    
    
    function [q_new_list, reached] = GenerateGSpine(this, q, q_e, d_c, planes)
        % Generalized spine is generated from q towards q_e
        % d_c is the distance-to-obstacles for the configuration q
        % planes are corresponding planes for the configuration q
        % q_new represents a list of new reached nodes
        % reached means whether q_e is reached
        
        for i = 1:this.N_layers
            [q_new_list(:,i), reached] = this.GenerateSpine(q, q_e, d_c);
            d_c = GetDistanceUnderestimation(q_new_list(:,i), planes);
            q = q_new_list(:,i);
            if d_c < this.d_crit || reached     % Distance to obstacle is less than critical or q_e is reached
                return;
            end   
%             if toc(this.T_alg) > this.T_max     % Interrupt
%                 disp('Path is not found. Time is up!');
%                 return;
%             end
        end   
    end
    
    
    function [q_new_list, reached] = GenerateGSpine2(this, q, q_e, d_c, planes)
        % Generalized spine is generated from q towards q_e
        % d_c is the distance-to-obstacles for the configuration q
        % planes are corresponding planes for the configuration q
        % q_new represents a list of new reached nodes
        % reached means whether q_e is reached
        
        for i = 1:this.N_layers
            [q_new_list(:,i), reached] = this.GenerateSpine2(q, q_e, d_c);
            d_c = GetDistanceUnderestimation2(q_new_list(:,i), planes);
            q = q_new_list(:,i);
            if all(d_c < this.d_crit) || reached     % Distance to obstacle is less than critical or q_e is reached
                return;
            end   
%             if toc(this.T_alg) > this.T_max     % Interrupt
%                 disp('Path is not found. Time is up!');
%                 return;
%             end
        end   
    end
    
    
    function [d_c, planes] = Get_dc_AndPlanes(~, TN, q_p)
        % Get minimal distance from q (determined with q_p) to obstacles. Also get corresponding planes
        % TN - tree number
        % q_p - pointer at node q        
        global tree;

        if isnan(tree.distances{TN}(q_p))
            [d_c, planes] = GetDistance(tree.nodes{TN}(:,q_p));
            tree.distances{TN}(q_p) = d_c;
            tree.planes{TN}{q_p} = planes;
        else        
            d_c = tree.distances{TN}(q_p);
            planes = tree.planes{TN}{q_p};
        end
    end
    
end
end