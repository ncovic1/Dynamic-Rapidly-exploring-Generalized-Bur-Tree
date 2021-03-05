classdef RBT
properties
    eps = 0.1;              % Step in C-space used by RRT-based algorithms
    eps0 = 0.01;            % Step in C-space for collsion check by RRT-based algorithms
    N_max = 10000;          % Max. number of considered nodes
    N_spines = 7;           % Number of bur spines
    d_crit = 0.01;          % Critical distance in W-space when RBT becomes RRT
    delta = pi;             % Radius of hypersphere from q to q_e
    tree;                   % Consisting of two parts, one from q_init and another from q_goal
    pointers = {{0},{0}};   % Pointing to location of children in tree
    path = [];              % Traversed path (sequence of nodes from q_init to q_goal)
    distance = {[],[]};     % Distance to each obstacle for each node
    N_nodes = 2;            % Number of considered nodes in tree
    N_iter = 0;             % Iteration counter
    T_alg = 0;              % Total algorithm runtime
end

methods
    function this = RBT(eps, eps0, N_max, N_spines, d_crit, delta)
        if nargin > 0
            this.eps = eps;
            this.eps0 = eps0;
            this.N_max = N_max;
            this.N_spines = N_spines;
            this.d_crit = d_crit;
            this.delta = delta;
        end
    end
    
    
    function this = Run(this)
        global robot obstacles;
        
        this.T_alg = tic;
        [collision, ~, ~] = CheckCollision(robot, obstacles, robot.q_init);
        if collision
            disp('Initial robot configuration is in collision!');
            return;
        end        
        [collision, ~, ~] = CheckCollision(robot, obstacles, robot.q_goal);
        if collision
            disp('Goal robot configuration is in collision!');
            return;
        end        
        this.tree = {robot.q_init, robot.q_goal};
        S = 2;  % Determines which tree is chosen, 1: from q_init; 2: from q_goal
        
        while true        
            % Generating bur
            S = 3 - S;
            q_e = 2*pi*rand(robot.N_DOF,1)-pi;  % Adding random node in C-space
            [q_near, q_near_p] = Find_q_near(this.tree{S}, q_e);
            [this.distance{S}, d_c] = this.Get_d_c(q_near, q_near_p, this.distance{S});
            if d_c < this.d_crit     % Distance to obstacle is less than d_crit
                [q_new, ~, collision] = this.GenerateEdge(q_near, q_e);     % Spine is generated using RRT                
                if ~collision
                    [this.tree{S}, this.pointers{S}, q_new_p] = UpdateTree(this.tree{S}, this.pointers{S}, q_near_p, q_new);
                    this.N_nodes = this.N_nodes + 1;
                else
                    q_new_p = q_near_p;
                end 
            else
                for i = 1:this.N_spines
                    q_new = q_near;
                    q_temp_p = q_near_p;
                    q_e = q_new + 2*pi*rand(robot.N_DOF,1) - pi;
                    q_e = this.SaturateSpine(q_new, q_e, this.delta);
                    q_e = this.PruneSpine(q_new, q_e);
                    [q_new, ~] = this.GenerateSpine(q_new, q_e, d_c);
                    [this.tree{S}, this.pointers{S}, q_new_p] = UpdateTree(this.tree{S}, this.pointers{S}, q_temp_p, q_new);
                    this.N_nodes = this.N_nodes + 1;
                end
            end
            
            % Bur-Connect
            S = 3 - S;
            [q_near, q_near_p] = Find_q_near(this.tree{S}, q_new);
            collision = false;  % Is collision occured
            reached = false;    % If trees are connected
            while ~collision && ~reached 
                [this.distance{S}, d_c] = this.Get_d_c(q_near, q_near_p, this.distance{S});
                if d_c < this.d_crit
                    [q_near, reached, collision] = this.GenerateEdge(q_near, q_new);     % Spine is generated using RRT                 
                else
                    [q_near, reached] = this.GenerateSpine(q_near, q_new, d_c);     % Spine is generated using RBT
                end
                if ~collision
                    [this.tree{S}, this.pointers{S}, q_near_p] = UpdateTree(this.tree{S}, this.pointers{S}, q_near_p, q_near);
                    this.N_nodes = this.N_nodes + 1; 
                end 
            end
            
            this.N_iter = this.N_iter + 1;
            if reached
                this.path = this.ComputePath(q_new_p, q_near_p, S);
                this.T_alg = toc(this.T_alg);
                disp(['Path is found in ', num2str(this.T_alg), ' [s].']);
                return;
            elseif this.N_nodes >= this.N_max
                this.T_alg = toc(this.T_alg);
                disp('Path is not found.');
                return;
            end
            S = 3 - S;
        end
    end
            
    
    function [distance, d_c] = Get_d_c(~, q, q_p, distance)
        % Get minimal distance from q to obstacles
        global robot obstacles;
        
        if length(distance) >= q_p
            d_c = distance(q_p);
        else
            [~, d_c, ~] = CheckCollision(robot, obstacles, q);
            distance(q_p) = d_c;
        end
    end
    
    
    function [q_new, reached, collision, d_c] = GenerateEdge(this, q, q_e)
        % Edge is generated from q towards q_e for eps
        % q_new is new reached node
        % collision means whether collision occured when moving from q towards q_e
        % reached means whether q_e is reached
        % d_c is minimal distance to obstacles
        global robot obstacles;
        
        q_new = q;
        D = norm(q_e-q);
        if D < this.eps
            reached = true;
            step = D;
            K_max = ceil(step/this.eps0);
        else
            reached = false;
            step = this.eps;
            K_max = ceil(this.eps/this.eps0);
        end
        if D == 0            
            [collision, d_c, ~] = CheckCollision(robot, obstacles, q_new);
            return;
        end
        
        collision = false;
        smjer = step*(q_e-q)/D;
        for k = 1:K_max
            q_new = q + k/K_max*smjer;
            [collision, d_c, ~] = CheckCollision(robot, obstacles, q_new);
            if collision
                q_new = q;
                collision = true;
                reached = false;
                break;
            end
        end  
    end
    
    
    function [q_new, reached] = GenerateSpine(~, q, q_e, d_c)
        global robot;
        % Spine is generated from q towards q_e
        % q_new represents new reached node
        % reached means whether q_e is reached
        
        q_new = q;    
        reached = false;
        if q_e == q
            reached = true;
            return;
        end
        
        [xyz_q, ~] = DirectKinematics(robot, q);
        rho = 0;    % Path length in W-space
        K_max = 5;  % Number of iterations for computing q*
        k = 1;
        while true                 
            fi = d_c - rho;    % Remaining path length in W-space
            step = ComputeStep(q_new, q_e, fi); 
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
        
        function step = ComputeStep(q, q_e, fi)
            [xyz, ~] = DirectKinematics(robot, q);        
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

        N_DOF = length(q);
        bounds = zeros(N_DOF,1);
        indices = [];
        for k = 1:N_DOF
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
                if q_temp >= -pi*ones(N_DOF,1) & q_temp <= pi*ones(N_DOF,1)
                    q_e = q_temp;
                    break;
                end
            end
        end
    end

    
    function path = ComputePath(this, q_new_p, q_near_p, S)
        if S == 2
            q_p1 = q_new_p;
            q_p2 = q_near_p;
        else
            q_p1 = q_near_p;
            q_p2 = q_new_p;
        end
        path1 = GetPath(this.tree{1}, this.pointers{1}, q_p1);
        path2 = GetPath(this.tree{2}, this.pointers{2}, q_p2); 
        path = [path1, path2(:,end-1:-1:1)];       
    end
    
end
end