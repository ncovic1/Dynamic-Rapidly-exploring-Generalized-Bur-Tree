classdef RGBT < RBT
properties
    N_layers = 5;           % Number of layers for building generalized bur
    separatrix = {{},{}};   % Lines/planes dividing space into two subspaces (free and "occupied")
    T_max = 1;              % Maximal algorithm runtime
end

methods
    function this = RGBT(eps, eps0, N_max, N_spines, N_layers, d_crit, delta, T_max)
        if nargin > 0
            this.eps = eps;
            this.eps0 = eps0;
            this.N_max = N_max;
            this.N_spines = N_spines;
            this.N_layers = N_layers;
            this.d_crit = d_crit;
            this.delta = delta;
            this.T_max = T_max;
        end
    end
    
    
    function this = Run(this)
        global robot obstacles;
        
        this.T_alg = tic;
        [collision, ~, ~] = CheckCollision(robot, obstacles, robot.q_init);
        if collision
%             disp('Initial robot configuration is in collision!');
            return;
        end        
        [collision, ~, ~] = CheckCollision(robot, obstacles, robot.q_goal);
        if collision
%             disp('Goal robot configuration is in collision!');
            return;
        end        
        this.tree = {robot.q_init, robot.q_goal};
        S = 2;  % Determines which tree is chosen, 1: from q_init; 2: from q_goal
        
        while true     
            % Bur is generated
            S = 3 - S;
            q_e = 2*pi*rand(robot.N_DOF,1)-pi;  % Adding random node in C-space
            [q_near, q_near_p] = Find_q_near(this.tree{S}, q_e);
            [this.distance{S}, d_c, this.separatrix{S}] = this.Get_d_c(q_near, q_near_p, this.distance{S}, this.separatrix{S});
            if d_c < this.d_crit     % Distance to obstacles is less than critical
                [q_new, ~, collision] = this.GenerateEdge(q_near, q_e);     % Spine is generated using RRT                
                if ~collision
                    [this.tree{S}, this.pointers{S}, q_new_p] = UpdateTree(this.tree{S}, this.pointers{S}, q_near_p, q_new);
                    this.N_nodes = this.N_nodes + 1;
                else
                    q_new_p = q_near_p;
                end 
            else
                Separatrix = this.separatrix{S}{q_near_p};
                for i = 1:this.N_spines
                    q_new = q_near;
                    q_new_p = q_near_p;
                    d_c_temp = d_c;
                    q_e = q_near + 2*pi*rand(robot.N_DOF,1) - pi;
                    q_e = this.SaturateSpine(q_near, q_e, this.delta);
                    q_e = this.PruneSpine(q_near, q_e);
                    for j = 1:this.N_layers
                        [q_new, reached] = this.GenerateSpine(q_new, q_e, d_c_temp);
                        [this.tree{S}, this.pointers{S}, q_new_p] = UpdateTree(this.tree{S}, this.pointers{S}, q_new_p, q_new);
                        d_c_temp = this.Update_d_c(q_new, Separatrix);
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
            S = 3 - S;
            [q_near, q_near_p] = Find_q_near(this.tree{S}, q_new);
            collision = false; 	% Is collision occured
            reached = false; 	% Are trees connected
            while ~collision && ~reached
                [this.distance{S}, d_c, this.separatrix{S}] = this.Get_d_c(q_near, q_near_p, this.distance{S}, this.separatrix{S});
                if d_c < this.d_crit
                    [q_near, reached, collision] = this.GenerateEdge(q_near, q_new);     % Spine is generated using RRT                 
                    if ~collision
                        [this.tree{S}, this.pointers{S}, q_near_p] = UpdateTree(this.tree{S}, this.pointers{S}, q_near_p, q_near);
                        this.N_nodes = this.N_nodes + 1; 
                    end 
                else
                    Separatrix = this.separatrix{S}{q_near_p};
                    for j = 1:this.N_layers
                        [q_near, reached] = this.GenerateSpine(q_near, q_new, d_c); 
                        [this.tree{S}, this.pointers{S}, q_near_p] = UpdateTree(this.tree{S}, this.pointers{S}, q_near_p, q_near);
                        d_c = this.Update_d_c(q_near, Separatrix);
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
                this.path = this.ComputePath(q_new_p, q_near_p, S);
                this.T_alg = toc(this.T_alg);
%                 disp(['Path is found in ', num2str(this.T_alg*1000), ' [ms].']);
                break;
            elseif this.N_nodes >= this.N_max || toc(this.T_alg) > this.T_max
                this.T_alg = toc(this.T_alg);
%                 disp('Path is not found.');
                break;
            end
            S = 3 - S;
        end
    end
    
    
    function [distance, d_c, separatrix] = Get_d_c(~, q, q_p, distance, separatrix)
        % Get minimal distance from q to obstacles
        global robot obstacles;
        
        if length(distance) >= q_p
            d_c = distance(q_p);
        else
            [~, d_c, separatrix{q_p}] = CheckCollision(robot, obstacles, q);
            distance(q_p) = d_c;
        end
    end
    
    
    function d_c = Update_d_c(~, q, separatrix)
        global robot;
        
        N_obstacles = size(separatrix,2);
        distance = zeros(robot.N_links, N_obstacles);
        [xyz, ~] = DirectKinematics(robot, q);

        for j = 1:N_obstacles
            for k = 1:robot.N_links          
                A = xyz(:,k);
                B = xyz(:,k+1);
                D = [separatrix(k,j,1); separatrix(k,j,2); separatrix(k,j,3)];
                n = [separatrix(k,j,4); separatrix(k,j,5); separatrix(k,j,6)];
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