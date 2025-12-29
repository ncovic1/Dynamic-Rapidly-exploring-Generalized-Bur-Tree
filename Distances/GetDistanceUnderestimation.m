function d_c = GetDistanceUnderestimation(q, planes)
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