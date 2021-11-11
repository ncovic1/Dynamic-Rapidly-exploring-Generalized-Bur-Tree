classdef RGBT < RBT
properties
    N_layers = 5;           % Number of layers for building generalized bur
    T_max = 1;              % Maximal algorithm runtime
end

methods
    function this = RGBT(eps, N_max, N_spines, N_layers, d_crit, delta, T_max)
        if nargin > 0
            this.eps = eps;
            this.N_max = N_max;
            this.N_spines = N_spines;
            this.N_layers = N_layers;
            this.d_crit = d_crit;
            this.delta = delta;
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
        [tree.distances{1}(1), ~] = this.Get_dc_AndPlanes(1, 1);
        if tree.distances{1}(1) == 0
            disp('Initial robot configuration is in the collision!');
            return;
        end
        [tree.distances{2}(1), ~] = this.Get_dc_AndPlanes(2, 1);
        if tree.distances{2}(1) == 0
            disp('Goal robot configuration is in the collision!');
            return;
        end 
        
        while true     
            % Bur is generated
            TN = 3 - TN;
            q_e = 2*pi*rand(robot.N_DOF,1)-pi;  % Adding random node in C-space
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_e);
            [d_c, planes] = this.Get_dc_AndPlanes(TN, q_near_p);
            if d_c < this.d_crit     % Distance to obstacles is less than critical
                [q_new, ~, collision] = this.GenerateEdge(q_near, q_e);     % Spine is generated using RRT                
                if ~collision
                    q_new_p = UpgradeTree(TN, q_near_p, q_new, NaN, NaN);
                    this.N_nodes = this.N_nodes + 1;
                else
                    q_new_p = q_near_p;
                end 
            else
                for i = 1:this.N_spines
                    q_new = q_near;
                    q_new_p = q_near_p;
                    d_c_temp = d_c;
                    q_e = q_near + 2*pi*rand(robot.N_DOF,1) - pi;
                    q_e = this.SaturateSpine(q_near, q_e, this.delta);
                    q_e = this.PruneSpine(q_near, q_e);
                    for j = 1:this.N_layers
                        [q_new, reached] = this.GenerateSpine(q_new, q_e, d_c_temp);
                        q_new_p = UpgradeTree(TN, q_new_p, q_new, NaN, NaN);
                        d_c_temp = this.Update_d_c(q_new, planes);
                        if d_c_temp < this.d_crit || reached     % Distance to obstacle is less than critical or q_e is reached
                            break;
                        end   
                        if toc(this.T_alg) > this.T_max     % Interrupt
                            %disp('Path is not found.');
                            return;
                        end
                    end            
                    this.N_nodes = this.N_nodes + j;
                end
            end
            if toc(this.T_alg) > this.T_max
                break;
            end
            
            % Bur-Connect
            TN = 3 - TN;
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_new);
            collision = false; 	% Is collision occured
            reached = false; 	% Are trees connected
            while ~collision && ~reached
                [d_c, planes] = this.Get_dc_AndPlanes(TN, q_near_p);
                if d_c < this.d_crit
                    [q_near, reached, collision] = this.GenerateEdge(q_near, q_new);     % Spine is generated using RRT                 
                    if ~collision
                        q_near_p = UpgradeTree(TN, q_near_p, q_near, NaN, NaN);
                        this.N_nodes = this.N_nodes + 1; 
                    end 
                else
                    for j = 1:this.N_layers
                        [q_near, reached] = this.GenerateSpine(q_near, q_new, d_c);
                        q_near_p = UpgradeTree(TN, q_near_p, q_near, NaN, NaN);
                        d_c = this.Update_d_c(q_near, planes);
                        if d_c < this.d_crit || reached     % Distance to obstacle is less than critical or trees are connected
                            break;
                        end
                        if toc(this.T_alg) > this.T_max     % Interrupt
                            %disp('Path is not found. Time is up!');
                            return;
                        end
                    end
                    this.N_nodes = this.N_nodes + j;
                end
            end
            
            this.N_iter = this.N_iter + 1;
            %disp(['RGBT: ', num2str(toc(this.T_alg)), ' ', num2str(this.N_nodes)]);
            if reached   % Two trees are connected
                this.path = GetPath(q_near_p, q_new_p, TN);
                this.T_alg = toc(this.T_alg);
%                 disp(['Path is found in ', num2str(this.T_alg*1000), ' [ms].']);
                break;
            elseif this.N_nodes >= this.N_max || toc(this.T_alg) > this.T_max
                this.T_alg = toc(this.T_alg);
%                 disp('Path is not found.');
                break;
            end
            TN = 3 - TN;
        end
    end
    
    
    function [d_c, planes] = Get_dc_AndPlanes(~, TN, q_p)
        % Get minimal distance from q (determined with q_p) to obstacles, and corresponding planes
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
    
    
    function d_c = Update_d_c(~, q, planes)
        global robot;        
        N_obstacles = size(planes,2);
        distance = zeros(robot.N_links, N_obstacles);
        [xyz, ~] = DirectKinematics(robot, q);

        for j = 1:N_obstacles
            for k = 1:robot.N_links          
                A = xyz(:,k);
                B = xyz(:,k+1);
                D = [planes(k,j,1); planes(k,j,2); planes(k,j,3)];
                n = [planes(k,j,4); planes(k,j,5); planes(k,j,6)];
                lambda_A = (n'*A - n'*D)/(n'*n);
                AS = norm(lambda_A*n);
                lambda_B = (n'*B - n'*D)/(n'*n);
                BR = norm(lambda_B*n);                
                distance(k,j) = min(AS, BR);
            end
        end
        d_c = min(min(distance));
    end
    
end
end