classdef DRGBT < RGBT
properties
    collision = 0;          % Whether the collision occured
    w_min = 0.5;            % Treshold 1 for the replanning assessment
    w_mean_min = 0.5;       % Treshold 2 for the replanning assessment
    T_s = inf;              % Maximal runtime of a single DRGBT iteration (also 1/f_s, where f_s is the number of frames per second)
    T_iter = 0;             % Averaged runtime of a single DRGBT iteration
end

methods
    function this = DRGBT(eps, N_max, N_spines, N_layers, d_crit, delta, T_s)
        global robot;
        if nargin > 0
            this.eps = eps;
            this.N_max = N_max;
            this.N_spines = N_spines;
            this.N_layers = N_layers;
            this.d_crit = d_crit;
            this.delta = delta;
            this.T_s = T_s;
        end
        this.delta = pi/2;      % This value is better for DRGBT
        this.N_max = 250*robot.N_DOF+1000;
        this.N_iter = 1;
    end
    
    function this = Run(this)
        global robot horizon obstacles graphics writerObj OBS;
        graphics = {{line(0,0)}, {line(0,0)}, {line(0,0)}, {line(0,0)}, {line(0,0)}, line(0,0), {line(0,0)}, {line(0,0)}, line(0,0)};
        
%         for nn = 1:size(obstacles.loc,2)
%             OBS(:,:,nn) = obstacles.loc(:,nn);
%         end
                
        horizon.N_h0 = 10;              % Initial horizon size
        horizon.N_h = horizon.N_h0;     % Horizon size that may change during the algorithm execution
        horizon.nodes = [];            	% Nodes from the horizon
        horizon.nodes_reached = [];   	% Reached nodes from the horizon
        horizon.weights = [];          	% Node weights
        horizon.missing_ind = [];    	% Indices of the horizon nodes that are not in the path
        horizon.d_max_mean = 0;         % Averaged maximal distance-to-obstacles through iterations
        horizon.distances = [];         % Underestimation of distances-to-obstacle for each node from nodes
        horizon.q_curr = robot.q_init;  % Current robot configuration
        horizon.q_next = [];            % Next robot configuration
        horizon.index_next = 1;         % Index of the next node from the horizon

        this.T_alg = tic;   % Total DRGBT runtime        
        T1 = inf;           % Maximal runtime for function GenerateHorizon is 0.006 [s]        
        k = 1;              % Index of next node from predefined path
        this.path = horizon.q_curr;
        
        % Obtaining the inital path
        rgbt = RGBT();
        rgbt = rgbt.Run();
        
        while true
            T_iter_temp = tic;
            replanning = false;   % Whether path replanning is required
            k = k + horizon.index_next;     
            if k+horizon.N_h-1 <= size(rgbt.path,2)
                horizon.nodes = rgbt.path(:,k:k+horizon.N_h-1);
            elseif k <= size(rgbt.path,2)
                horizon.nodes = rgbt.path(:,k:end);
            else
                k = size(rgbt.path,2);
                if isempty(rgbt.path)
                    horizon.nodes = this.GetRandomNodes(horizon.N_h);  % If initial path was not found, horizon.N_h random nodes are added
                else
                    horizon.nodes = rgbt.path(:,end);   % At least q_goal remains in horizon
                end
            end
            
            horizon.q_next = horizon.nodes(:,1);
            this.GenerateHorizon(horizon.nodes, [], T1);            
            horizon.q_next = this.Get_q_next(zeros(1,horizon.N_h));
            index_prev = horizon.index_next;
            
            if k == size(rgbt.path,2) && ~prod(horizon.q_next == robot.q_goal)
                replanning = true;
            end
            
            var = false;
            ind_good = 1:horizon.N_h;   % Indices of good nodes in the horizon
            ind_bad = [];               % Indices of bas nodes in the horizon
            reached = false;
            while ~reached           
                obstacles = UpdateObstacles(robot, obstacles);
%                 OBS = [OBS, zeros(6,1,size(obstacles.loc,2))];
%                 for nn = 1:size(obstacles.loc,2)
%                     OBS(:,end,nn) = obstacles.loc(:,nn);
%                 end
                
                this.T_iter = ((this.N_iter-1)*this.T_iter + toc(T_iter_temp))/this.N_iter;
                this.N_iter = this.N_iter + 1;                
                if var
                    T_iter_temp = tic;
                end
                
                D = norm(horizon.q_next-horizon.q_curr);
                q_p = horizon.q_curr;
                if D > this.eps
                    horizon.q_curr = q_p + this.eps*(horizon.q_next-q_p)/D;               
                else
                    horizon.q_curr = horizon.q_next;
                    reached = true;
                end
                this.path = [this.path, horizon.q_curr];
                var = true;
                
                this.collision = CheckCollision(horizon.q_curr);
                if this.collision
                    this.T_alg = toc(this.T_alg);
                    this.Draw(q_p, rgbt.path);
                    disp('Collision !!!');                    
%                     pause(0.000000025);
%                     writeVideo(writerObj, getframe(gcf));
                    return;
                end
                
                if horizon.q_curr == robot.q_goal
                    this.T_alg = toc(this.T_alg);
                    this.Draw(q_p, rgbt.path); 
                    disp('Goal configuration has been successfully reached!');                    
%                     pause(0.000000025);
%                     writeVideo(writerObj, getframe(gcf));
                    return;
                end
                
                distances_prev = horizon.distances;
                this.GenerateHorizon(horizon.nodes(:,ind_good), horizon.nodes(:,ind_bad), T1);                
                ind_good = 1:horizon.N_h;
                ind_bad = [];
                
                if length(horizon.distances) > length(distances_prev)
                    distances_prev(end+1:length(horizon.distances)) = horizon.distances(length(distances_prev)+1:length(horizon.distances));
                elseif length(horizon.distances) < length(distances_prev)
                    distances_prev = distances_prev(1:length(horizon.distances));
                end                
                delta_dist = horizon.distances - distances_prev;   % Change of the distances
                
                horizon.q_next = this.Get_q_next(delta_dist, index_prev);                  
                index_prev = horizon.index_next;       
                
                % Replanning procedure assessment
                if max(horizon.weights) < this.w_min && mean(horizon.weights) < this.w_mean_min || replanning  
                    T_remain = this.T_s - toc(T_iter_temp) - 0.001;     % Added 1 [ms] for the remaining code lines
                    horizon.index_next = 1;
                    robot.q_init = horizon.q_curr;
                    A_new = RGBT(this.eps, this.N_max, this.N_spines, this.N_layers, this.d_crit, this.delta, T_remain);
                    A_new = A_new.Run();

                    if ~isempty(A_new.path)  % Update path to the goal
%                         disp('The path has been replanned.');
                        replanning = false;
                        reached = true;
                        rgbt.path = A_new.path;
                        k = 1;
                    else    % New path is not found
%                         disp('New path is not found.');
                        replanning = true;   
                        ind_good = [];
                        ind_bad = [];
                        for ii = 1:horizon.N_h
                            if horizon.weights(ii) == 0  % Bad nodes
                                ind_bad = [ind_bad, ii];
                            else
                                ind_good = [ind_good, ii];
                            end
                        end
                    end
                end
                
                % Real-time checking
%                 T_remain = this.T_s - toc(T_start);
% %                 disp(['Time remaining: ', num2str(T_remain*1000), ' [ms]']);
%                 if T_remain < 0
% %                     disp('Real-time is broken !!!');
%                 else
%                     while this.T_s-toc(T_start) > 0
%                     end
%                 end
                
%                 disp('----------------------------------------------------------------------------------------');
            
                this.Draw(q_p, rgbt.path); 
                
%                 pause(0.000000025);
%                 writeVideo(writerObj, getframe(gcf));

            end            
        end
    end
    
    
    function GenerateHorizon(this, nodes_good, nodes_bad, T_max)
        % 'nodes_good' contains good nodes in the beginning, but later it contains updated horizon
        % 'nodes_bad' are bad nodes that will be replaced with "better" nodes
        % 'T_max' is the maximal runtime for this function
        
        global robot horizon;        
        T = tic;
        [d_c, planes] = GetDistance(horizon.q_curr);
        horizon.N_h = floor(horizon.N_h0*(1 + this.d_crit/d_c));
        if horizon.N_h > 5*robot.N_DOF*horizon.N_h0
            horizon.N_h = 5*robot.N_DOF*horizon.N_h0;
        end
        
        Q_h_good_size = size(nodes_good,2);
        Q_h_bad_size = size(nodes_bad,2);
        horizon.missing_ind = Q_h_good_size+1:horizon.N_h;
        
        if Q_h_good_size > horizon.N_h    % Surplus nodes are deleted. Best nodes holds priority.
            nodes_good = nodes_good(:,1:horizon.N_h);
            nodes_bad = [];
            Q_h_bad_size = 0;
            horizon.missing_ind = [];
        elseif Q_h_good_size + Q_h_bad_size >= horizon.N_h
            nodes_bad = nodes_bad(:,1:horizon.N_h-Q_h_good_size);
            Q_h_bad_size = size(nodes_bad,2);
            nodes_good = [nodes_good, nodes_bad];
        else    % If horizon.N_h has increased or little nodes exist, random/lateral nodes are added
            nodes_good = [nodes_good, nodes_bad];
            if robot.N_DOF == 2
                nodes_good(:,Q_h_good_size+Q_h_bad_size+1:horizon.N_h) = this.GetRandomNodes(horizon.N_h-Q_h_good_size-Q_h_bad_size);
            else
                nodes_good(:,Q_h_good_size+Q_h_bad_size+1:horizon.N_h) = this.GetRandomNodes(horizon.N_h-Q_h_good_size-Q_h_bad_size, []);
            end
        end
        
        % Bad nodes are modified
        if Q_h_bad_size > 0
            nodes_good(:,Q_h_good_size+1:Q_h_good_size+Q_h_bad_size) = this.GetRandomNodes(Q_h_bad_size, nodes_bad, true);
        end
        
        % Lateral spines are added
        N_h_old = horizon.N_h;
        horizon.N_h = N_h_old + 2*robot.N_DOF - 2;
        nodes_good(:,N_h_old+1:horizon.N_h) = this.GetRandomNodes(horizon.N_h-N_h_old, []);
        horizon.missing_ind = [horizon.missing_ind, N_h_old+1:horizon.N_h];     % Node indices that are not in the path
        
        i = 0;  % Number of added spines
        while i < horizon.N_h
            i = i + 1; 
            [q_new, reached] = this.GenerateSpine(horizon.q_curr, nodes_good(:,i), d_c);
            d_c_new = this.GetUnderestimation_dc(q_new, planes); 
                 
            if d_c_new < this.d_crit && ~prod(q_new == robot.q_goal)    % Underestimation to obstacle is less than critical
                nodes_good(:,i) = this.GetRandomNodes(1, q_new, false);    % q_e is added instead of nodes_good(:,i), which is "bad"
                [q_new, reached] = this.GenerateSpine(horizon.q_curr, nodes_good(:,i), d_c);
                d_c_new = this.GetUnderestimation_dc(q_new, planes); 
                horizon.missing_ind(end+1) = i;
            end
            
            for jj = 2:this.N_layers
                if reached   % q_e has been reached     
                    break;
                end
                [q_new, reached] = this.GenerateSpine(q_new, nodes_good(:,i), d_c_new); 
                d_c_new = this.GetUnderestimation_dc(q_new, planes);
            end
            horizon.distances(i) = d_c_new;  
            horizon.nodes_reached(:,i) = q_new; 
            
            if toc(T) > T_max     % Interrupt
                horizon.nodes = nodes_good(:,1:i);
                horizon.N_h = i;
                if i >= horizon.missing_ind(1)
                    horizon.missing_ind = horizon.missing_ind(1:i-horizon.missing_ind(1)+1);
                else
                    horizon.missing_ind = [];
                end
                return;
            end
        end
        
        horizon.nodes = nodes_good;
    end    
    
    
    function Q_e = GetRandomNodes(this, N, q, orientation)
        global robot horizon;
        
        % Adding random nodes
        Q_e = zeros(robot.N_DOF, N);
        for i = 1:N
            Q_e(:,i) = horizon.q_curr + (robot.range(:,2)-robot.range(:,1)).*rand(robot.N_DOF,1) + robot.range(:,1);
        end
        
        if nargin == 3  % Adding lateral spines
            if robot.N_DOF == 2   % In 2D C-space only two possible spines exist
                Q_e = [horizon.q_curr(1), horizon.q_curr(1); horizon.q_curr(2)-1, horizon.q_curr(2)+1];
            end
            if horizon.q_next(1)-horizon.q_curr(1) ~= 0
                for i = 1:N
                    Q_e(1,i) = ((horizon.q_next-horizon.q_curr)'*horizon.q_curr - ...
                        (horizon.q_next(2:end)-horizon.q_curr(2:end))'*Q_e(2:end,i))/(horizon.q_next(1)-horizon.q_curr(1));
                end
            end
        elseif nargin == 4  % Adding random node with oriented weight around q (orientation = true) or around -q (orientation = false)
            for i = 1:N
                a_norm = norm(q(:,i)-horizon.q_curr);
                temp_set = [-1,1];
                r = (2*rand(robot.N_DOF,1)-1)*a_norm/sqrt(robot.N_DOF-1);
                r(1) = temp_set(randi(2))*sqrt(a_norm^2-norm(r(2:end))^2);
                if orientation
                    Q_e(:,i) = q(:,i) + r;
                else
                    Q_e(:,i) = 2*horizon.q_curr - q(:,i) + r;
                end
            end
        end

        for i = 1:N
            Q_e(:,i) = this.SaturateSpine(horizon.q_curr, Q_e(:,i), this.delta);
            Q_e(:,i) = this.PruneSpine(horizon.q_curr, Q_e(:,i));
        end

    end
    
    
    function q_next = Get_q_next(this, delta_dist, index_prev)
        global robot horizon;
        
        horizon.d_max_mean = ((this.N_iter-1)*horizon.d_max_mean + max(horizon.distances))/this.N_iter;
        d_i = zeros(1,horizon.N_h);
        for i = 1:horizon.N_h
            d_i(i) = norm(robot.q_goal-horizon.nodes_reached(:,i));
        end
        [d_best, index] = min(d_i);  % Minimal distance-to-goal
        if d_best == 0  % q_goal is cosidered
            d_best = 0.00001;  % Added only to avoid 0/0 when d_i(i)==0
            d_i(index) = d_best;
        end
        p_dist = d_best./d_i;
        p_dist_mean = mean(p_dist);
        horizon.weights = zeros(1,horizon.N_h);
        for i = 1:horizon.N_h
            if horizon.distances(i) > this.d_crit
                horizon.weights(i) = (horizon.distances(i) + delta_dist(i))/horizon.d_max_mean + p_dist(i) - p_dist_mean;   
                if horizon.weights(i) > 1
                    horizon.weights(i) = 1;
                elseif horizon.weights(i) < 0
                    horizon.weights(i) = 0;
                end
            end
        end
        horizon.weights(horizon.missing_ind) = horizon.weights(horizon.missing_ind)/2;  % weight is halved if node does not exist in the path
        
        hysteresis = 0.1;
        [w_h_max, horizon.index_next] = max(horizon.weights);
        D_min = inf;
        for i = 1:horizon.N_h
            if abs(w_h_max - horizon.weights(i)) < hysteresis
                if d_i(i) < D_min    % The best node nearest to q_goal is chosen
                    D_min = d_i(i);
                    horizon.index_next = i;
                end
            end
        end
        
        if nargin == 3  % If weights of previous and new node are close, previous node remains
            if horizon.index_next ~= index_prev && index_prev <= length(horizon.weights)
                if D_min == 0   % If q_goal has been reached, hysteresis is set to zero
                    hysteresis = 0;
                end
                if abs(horizon.weights(horizon.index_next) - horizon.weights(index_prev)) < hysteresis
                    horizon.index_next = index_prev;
                end
            end
        end        
        q_next = horizon.nodes_reached(:,horizon.index_next);
    end
    
    
    function Draw(this, q_p, predefined_path)
        global robot horizon graphics;

        NN = 1;
        for jj = 1:length(graphics{NN})
            delete(graphics{NN}{jj});
        end
        if robot.dim == 2
            graphics{NN} = DrawRobot(horizon.q_curr, 'red', 0.2, 3, 20);
        else
            graphics{NN} = DrawRobot(horizon.q_curr, 'red', 0.3);
        end

        NN = 2;
        graphics{NN} = DrawPath(predefined_path, graphics{NN}, [0.7,0.7,0.7]);
        graphics{NN+1} = DrawPath(this.path, graphics{NN+1}, 'g');
        if robot.N_DOF == 2
            subplot(1,2,2); 
            plot([q_p(1),horizon.q_curr(1)],[q_p(2),horizon.q_curr(2)],'Color','g','LineWidth',4); hold on;
        end

        NN = 4;
        [graphics_WS_new, graphics_CS_new] = DrawObstacles();
%         [graphics_WS_new, graphics_CS_new] = DrawObstacles(0.05);

        for i = 1:length(graphics{NN})
            delete(graphics{NN}{i});
        end
        for i = 1:length(graphics{NN+1})
            delete(graphics{NN+1}{i});
        end
        graphics{NN} = graphics_WS_new;
        graphics{NN+1} = graphics_CS_new;        

        if robot.N_DOF == 2
            NN = 6;   
            subplot(1,2,2);
            delete(graphics{NN});
            graphics{NN} = plot(horizon.q_curr(1),horizon.q_curr(2),'r.','MarkerSize',20); hold on;

            NN = 7;  
            for jj = 1:length(graphics{NN})
                delete(graphics{NN}{jj});             
                delete(graphics{NN+1}{jj});            
            end        
            for jj = 1:horizon.N_h
                if horizon.weights(jj) <= 0
                    p_h_temp = 1;
                else
                    p_h_temp = ceil(20*horizon.weights(jj));
                end
                try
                    graphics{NN}{jj} = plot(horizon.nodes_reached(1,jj),horizon.nodes_reached(2,jj),'bo','MarkerSize', p_h_temp); hold on;  
                catch
                end
                graphics{NN+1}{jj} = plot([horizon.q_curr(1),horizon.nodes_reached(1,jj)],[horizon.q_curr(2),horizon.nodes_reached(2,jj)],'Color','red','LineWidth',0.5); hold on;
            end 

            NN = 9;        
            delete(graphics{NN});
            graphics{NN} = plot(horizon.q_next(1),horizon.q_next(2),'rx','MarkerSize',20,'LineWidth',2);
            subplot(1,2,1);     
        end

        drawnow;
        
        
        function graphics = DrawPath(path, graphics, color)

            if robot.N_DOF == 2
                subplot(1,2,2);
                for ii = 1:length(graphics)
                    delete(graphics{ii});
                end               
                for ii = 1:size(path,2)  
                    if ii < size(path,2) 
                        x = path(:,ii);
                        y = path(:,ii+1);
                        graphics{ii} = plot([x(1),y(1)],[x(2),y(2)],'Color',color,'LineWidth',4); hold on;
                    end
                end
            else
                graphics = {};
            end
        end

    end
end
end