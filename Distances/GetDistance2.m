function [d_c, planes] = GetDistance2(q)
    % Returns the distance from each robot link to obstacles
    
    global robot obstacles;    
    N_obstacles = size(obstacles.loc,2);
    planes = zeros(6, robot.N_links, N_obstacles);
    d_c = zeros(1, robot.N_links);
    [xyz, ~] = DirectKinematics(robot, q);

    for k = 1:robot.N_links
        distances = zeros(1, N_obstacles);
        for j = 1:N_obstacles
            if obstacles.model == 1    
                [collision, distances(j), planes(:,k,j)] = DistanceLineSegToSphere(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
            else
                if robot.dim == 2
                    [collision, distances(j), planes(:,k,j)] = DistanceLineSegToRectangle(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
                else
                    [collision, distances(j), planes(:,k,j)] = DistanceLineSegToCuboid(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
%                     [collision, distance(k,j), planes(:,k,j)] = DistanceLineSegToCuboid_v2(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
                end                
            end
            distances(j) = distances(j) - robot.radii(k);
            if collision || distances(j) <= 0
                planes = NaN;
                return;
            end
        end
        d_c(k) = min(distances);
    end
end