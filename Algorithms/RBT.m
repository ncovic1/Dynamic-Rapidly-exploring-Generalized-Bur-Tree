classdef RBT
properties
    eps = 0.1;              % Step in C-space used by RRT-based algorithms
    N_max = 10000;          % Max. number of considered nodes
    N_spines = 7;           % Number of bur spines
    d_crit = 0.03;          % Critical distance in W-space when RBT becomes RRT
    delta = pi;             % Radius of hypersphere from q to q_e
    path = [];              % Traversed path (sequence of nodes from q_init to q_goal)
    N_iter = 0;             % Iteration counter
    T_alg = 0;              % Total algorithm runtime
    T_max = 1;              % Maximal algorithm runtime in [s]
end

methods
    function this = RBT(eps, N_max, N_spines, d_crit, delta)
        if nargin > 0
            this.eps = eps;
            this.N_max = N_max;
            this.N_spines = N_spines;
            this.d_crit = d_crit;
            this.delta = delta;
        end
    end
    
    
    function this = Run(this)
        global robot tree;
        TN = 2;  % Determines which trees is chosen, 1: from q_init; 2: from q_goal
        tree.nodes = {robot.q_init, robot.q_goal};  % Consisting of two parts, one from q_init and another from q_goal
        tree.pointers = {{0}, {0}};     % Pointing to location of parent/children in trees
        tree.distances = {NaN, NaN};    % Distance to each obstacle for each node
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
            %% Generating bur
            TN = 3 - TN;
            q_e = this.GetRandomNode();
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_e);
            d_c = this.Get_dc(TN, q_near_p);
            if d_c > this.d_crit
                for i = 1:this.N_spines
                    q_e = q_near + this.GetRandomNode();
                    q_e = this.SaturateSpine(q_near, q_e, this.delta);
                    q_e = this.PruneSpine(q_near, q_e);
                    [q_new, ~] = this.GenerateSpine(q_near, q_e, d_c);
                    q_new_p = UpgradeTree(TN, q_near_p, q_new, NaN);
                end
            else    % Distance to obstacle is less than d_crit
                [q_new, ~, collision] = this.GenerateEdge(q_near, q_e);     % Spine is generated using RRT                
                if ~collision
                    q_new_p = UpgradeTree(TN, q_near_p, q_new, NaN);
                else
                    q_new_p = q_near_p;
                end 
            end
            
            %% Bur-Connect
            TN = 3 - TN;
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_new);
            collision = false;  % Is collision occured
            reached = false;    % If trees are connected
            while ~collision && ~reached 
                d_c = this.Get_dc(TN, q_near_p);
                if d_c > this.d_crit
                    [q_near, reached] = this.GenerateSpine(q_near, q_new, d_c);     % Spine is generated using RBT              
                else
                    [q_near, reached, collision] = this.GenerateEdge(q_near, q_new);     % Spine is generated using RRT   
                end
                if ~collision
                    q_near_p = UpgradeTree(TN, q_near_p, q_near, NaN);
                end 
            end
            
            this.N_iter = this.N_iter + 1;
            if reached
                this.path = GetPath(q_near_p, q_new_p, TN);
                this.T_alg = toc(this.T_alg);
                disp(['The path is found in ', num2str(this.T_alg*1000), ' [ms].']);
                return;
            elseif size(tree.nodes{1},2)+size(tree.nodes{2},2) >= this.N_max
                this.T_alg = toc(this.T_alg);
                disp('The path is not found.');
                return;
            end
            TN = 3 - TN;
        end
    end
    
    
    function q_e = GetRandomNode(~)
        % Adding a random node with uniform distribution in C-space
        global robot;
        
        q_e = (robot.range(:,2)-robot.range(:,1)).*rand(robot.N_DOF,1) + robot.range(:,1);
    end
    
    
    function [q_new, reached, collision] = GenerateEdge(this, q, q_e)
        % Edge is generated from q towards q_e for eps
        % q_new is new reached node
        % collision means whether collision occured when moving from q towards q_e
        % reached means whether q_e is reached
                
        D = norm(q_e-q);
        if D < this.eps  
            reached = true;
            if D == 0
                collision = false;
                q_new = q;
                return;
            end              
            step = D;
        else
            reached = false;
            step = this.eps;
        end
        
        eps0 = this.eps/10;
        K = ceil(step/eps0);
        Step = step/(K*D);
        for k = K:-1:1
            q_new = q + k*Step*(q_e-q);                
            collision = CheckCollision(q_new);
            if collision
                q_new = q;
                reached = false;
                return;
            end
        end
        q_new = q + K*Step*(q_e-q);
        
    end
    
    
    function [q_new, reached] = GenerateSpine(~, q, q_e, d_c)
        global robot;
        % Spine is generated from q towards q_e
        % q_new represents new reached node
        % reached means whether q_e is reached
        
        q_new = q;    
        if q_e == q
            reached = true;
            return;
        end
        
        reached = false;
        [xyz_q, ~] = DirectKinematics(robot, q);
        xyz_q_new = xyz_q;
        rho = 0;    % Path length in W-space
        K_max = 5;  % Number of iterations for computing q*
        k = 1;
        while true                 
            step = ComputeStep(q_new, q_e, d_c-rho, xyz_q_new);   % 'd_c-rho' is the remaining path length in W-space
            if step > 1
                q_new = q_e;
                reached = true;
            else          
                q_new = q_new + step*(q_e-q_new);
            end
            if k == K_max || reached
                break;
            end
            [xyz_q_new, ~] = DirectKinematics(robot, q_new);
            Rho = zeros(robot.N_links,1);
            for i = 2:robot.N_links+1
                Rho(i-1) = norm(xyz_q_new(:,i)-xyz_q(:,i));
            end
            rho = max(Rho);
            k = k + 1;
        end  
        
        function step = ComputeStep(q, q_e, fi, xyz)    
            if robot.dim == 2   % assumes that N_links = N_DOF
                d = 0;
                for ii = 1:robot.N_links
                    r = [robot.DH_table(ii,2), zeros(1,robot.N_links-ii)];
                    for kk = ii+1:robot.N_links
                        r(kk) = norm(xyz(:,kk+1)-xyz(:,ii));
                    end
                    d = d + max(r)*abs(q_e(ii)-q(ii));
                end
            else
                if robot.model == 1
                    L = [robot.DH_table(1,1), robot.DH_table(2,2), robot.DH_table(4,1), robot.DH_table(6,1)];
                    r(1) = max([norm(xyz(1:2,3)), norm(xyz(1:2,4)), norm(xyz(1:2,5))]);
                    r(2) = max([L(2), norm(xyz(:,4)-xyz(:,2)), norm(xyz(:,5)-xyz(:,2))]);
                    r(3) = max([L(3), norm(xyz(:,5)-xyz(:,3))]);
                    C_proj = Get_C_proj(xyz(:,3), xyz(:,4), xyz(:,5));
                    r(4) = norm(xyz(:,5)-C_proj);
                    r(5) = L(4);
                elseif robot.model == 2
                    L = [robot.DH_table(1,1), robot.DH_table(2,2), robot.DH_table(3,2), ...
                        robot.DH_table(4,1), robot.DH_table(5,2), robot.DH_table(6,1)];
                    r(1) = max([norm(xyz(1:2,3)), norm(xyz(1:2,4)), norm(xyz(1:2,5)), norm(xyz(1:2,6)), norm(xyz(1:2,7))]'+robot.radii(2:end));
                    r(2) = max([L(2), norm(xyz(:,4)-xyz(:,2)), norm(xyz(:,5)-xyz(:,2)), norm(xyz(:,6)-xyz(:,2)), norm(xyz(:,7)-xyz(:,2))]'+robot.radii(2:end));
                    r(3) = max([L(3), norm(xyz(:,5)-xyz(:,3)), norm(xyz(:,6)-xyz(:,3)), norm(xyz(:,7)-xyz(:,3))]'+robot.radii(3:end));
                    C_proj1 = Get_C_proj(xyz(:,4), xyz(:,5), xyz(:,6));
                    C_proj2 = Get_C_proj(xyz(:,4), xyz(:,5), xyz(:,7));
                    r(4) = max([norm(xyz(:,6)-C_proj1), norm(xyz(:,7)-C_proj2)]'+robot.radii(5:end));
                    r(5) = max([L(5), norm(xyz(:,7)-xyz(:,5))]'+robot.radii(5:end));
                end
                d = r*abs(q(1:end-1)-q_e(1:end-1));
            end
            step = fi/d;

            function C_proj = Get_C_proj(A, B, C)
                % C_proj is projection of C on line determined with A and B
                AB = B - A;
                t_opt = (C-A)'*AB/(AB'*AB);
                C_proj = A + t_opt*AB;
            end
        end
    end
    
    
    function q_e = SaturateSpine(~, q, q_e, length)
        Norm = norm(q_e-q);
        if Norm > 0
            q_e = q + length*(q_e-q)/Norm;
        end
    end
    
    
    function q_e = PruneSpine(~, q, q_e)
        % Prune spine from q to q_e, if it comes out C-space domain
        global robot;

        bounds = zeros(robot.N_DOF,1);
        indices = [];
        for k = 1:robot.N_DOF
            if q_e(k) > pi
                bounds(k) = pi;
                indices =  [indices, k];
            elseif q_e(k) < -pi
                bounds(k) = -pi;
                indices =  [indices, k];
            end
        end
        if length(indices) == 1
            t = (bounds(indices)-q(indices))/(q_e(indices)-q(indices));
            q_e = q + t*(q_e-q);
        elseif length(indices) > 1
            for k = indices
                t = (bounds(k)-q(k))/(q_e(k)-q(k));
                q_temp = q + t*(q_e-q);
                if q_temp >= -pi*ones(robot.N_DOF,1) & q_temp <= pi*ones(robot.N_DOF,1)
                    q_e = q_temp;
                    break;
                end
            end
        end
    end
    
    
    function d_c = Get_dc(~, TN, q_p)
        % Get minimal distance from q (determined with q_p) to obstacles
        % TN - tree number
        % q_p - pointer at node q
        global tree;

        if isnan(tree.distances{TN}(q_p))
            q = tree.nodes{TN}(:,q_p);
            [d_c, ~] = GetDistance(q);
            tree.distances{TN}(q_p) = d_c;
        else        
            d_c = tree.distances{TN}(q_p);
        end
    end
    
end
end