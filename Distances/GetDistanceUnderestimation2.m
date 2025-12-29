function d_c = GetDistanceUnderestimation2(q, planes)
    % Returns the underestimation of distance-to-obstacle from each robot link to obstacles, 
    % i.e. returns the distance-to-planes

    global robot obstacles;        
    N_obstacles = size(obstacles.loc,2);
    d_c = zeros(1, robot.N_links);

    [xyz, ~] = DirectKinematics(robot, q);
    for k = 1:robot.N_links
        distances = zeros(1, N_obstacles);
        for j = 1:N_obstacles
            % planes = [P1; P2-P1];
            % A = xyz(:,k); B = xyz(:,k+1);
            P1 = planes(1:3,k,j);
            P21 = planes(4:6,k,j);
            distances(j) = min(abs(P21'*(xyz(:,k) - P1)) / norm(P21), ... 
                               abs(P21'*(xyz(:,k+1) - P1)) / norm(P21)) - robot.radii(k);
        end
        d_c(k) = min(distances);
    end
end