classdef RRTC < RRT
properties        

end

methods
    function this = RRTC(eps, eps0, N_max)
        if nargin > 0
            this.eps = eps;
            this.eps0 = eps0;
            this.N_max = N_max;
        end
        this.pointers = {{0}, {0}};
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
            % Edge is generated
            S = 3 - S;
            q_rand = 2*pi*rand(robot.N_DOF,1)-pi;
            [q_near, q_near_p] = Find_q_near(this.tree{S}, q_rand);
            [q_new, ~, collision] = this.GenerateEdge(q_near, q_rand);
            if ~collision
                [this.tree{S}, this.pointers{S}, q_new_p] = UpdateTree(this.tree{S}, this.pointers{S}, q_near_p, q_new);
                this.N_nodes = this.N_nodes + 1; 
            else
                q_new_p = q_near_p;
            end
            
            % Two trees are trying to connect
            S = 3 - S;
            [q_near, q_near_p] = Find_q_near(this.tree{S}, q_new);
            collision = false;  % Whether the collision occured
            reached = false;    % Whether trees are connected
            while ~collision && ~reached
                [q_near, reached, collision] = this.GenerateEdge(q_near, q_new);
                if ~collision
                    [this.tree{S}, this.pointers{S}, q_near_p] = UpdateTree(this.tree{S}, this.pointers{S}, q_near_p, q_near);
                    this.N_nodes = this.N_nodes + 1; 
                end 
            end
            
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