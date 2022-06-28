classdef RGBT < RBT
properties
    N_layers = 5;           % Number of layers for building a generalized bur
end

methods
    function this = RGBT(eps, N_max, N_spines, d_crit, delta, N_layers, T_max)
        if nargin > 0
            RBT(eps, N_max, N_spines, d_crit, delta);
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
            %% Bur is generated
            TN = 3 - TN;
            q_e = this.GetRandomNode();
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_e);
            [d_c, planes] = this.Get_dc_AndPlanes(TN, q_near_p);
            if d_c > this.d_crit
                for i = 1:this.N_spines
                    q_e = q_near + this.GetRandomNode();
                    q_e = this.SaturateSpine(q_near, q_e, this.delta);
                    q_e = this.PruneSpine(q_near, q_e);
                    [q_new, q_new_p, ~] = this.GenerateGSpine(q_near, q_e, d_c, planes, TN, q_near_p);
                end
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
            
            %% Bur-Connect
            TN = 3 - TN;
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_new);
            collision = false; 	% Is collision occured
            reached = false; 	% Are trees connected
            while ~collision && ~reached
                [d_c, planes] = this.Get_dc_AndPlanes(TN, q_near_p);
                if d_c > this.d_crit
                    [q_near, q_near_p, reached] = this.GenerateGSpine(q_near, q_new, d_c, planes, TN, q_near_p);
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
    
    
    function [q_new, q_new_p, reached] = GenerateGSpine(this, q, q_e, d_c, planes, TN, q_p)
        % Generalized spine is generated from q towards q_e
        % d_c is the distance-to-obstacles for the configuration q
        % planes are corresponding planes for the configuration q
        % q_new represents new reached node
        % reached means whether q_e is reached
        
        q_new = q;
        q_new_p = q_p;
        for i = 1:this.N_layers
            [q_new, reached] = this.GenerateSpine(q_new, q_e, d_c);
            q_new_p = UpgradeTree(TN, q_new_p, q_new, NaN, NaN);
            d_c = this.GetUnderestimation_dc(q_new, planes);
            if d_c < this.d_crit || reached     % Distance to obstacle is less than critical or q_e is reached
                return;
            end   
            if toc(this.T_alg) > this.T_max     % Interrupt
                %disp('Path is not found. Time is up!');
                return;
            end
        end   
    end
        
    
    function d_c = GetUnderestimation_dc(~, q, planes)
        % Returns the underestimation of distance-to-obstacle, i.e. returns the distance-to-planes
        
        global robot obstacles;        
        N_obstacles = size(obstacles.loc,2);
        distance = zeros(robot.N_links, N_obstacles);
        
        [xyz, ~] = DirectKinematics(robot, q);
        for j = 1:N_obstacles
            for k = 1:robot.N_links
                % planes = [P1; P2-P1];
                % A = xyz(:,k); B = xyz(:,k+1);
                P1 = planes(1:3,k,j);
                P21 = planes(4:6,k,j);
                distance(k,j) = min(abs(P21'*(xyz(:,k) - P1)) / norm(P21), ... 
                                    abs(P21'*(xyz(:,k+1) - P1)) / norm(P21)) - robot.radii(k);
            end
        end
        d_c = min(min(distance));
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