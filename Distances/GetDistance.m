function [d_c, planes] = GetDistance(q)
    global robot obstacles;
    
    N_obstacles = size(obstacles.loc,2);
    planes = zeros(6, robot.N_links, N_obstacles);
    distances = zeros(robot.N_links, N_obstacles);
    [xyz, ~] = DirectKinematics(robot, q);

    for j = 1:N_obstacles
        for k = 1:robot.N_links
            if obstacles.model == 1    
                [collision, distances(k,j), planes(:,k,j)] = DistanceLineSegToSphere(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
            else
                if robot.dim == 2
                    [collision, distances(k,j), planes(:,k,j)] = DistanceLineSegToRectangle(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
                else
                    [collision, distances(k,j), planes(:,k,j)] = DistanceLineSegToCuboid(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
%                     [collision, distance(k,j), planes(:,k,j)] = DistanceLineSegToCuboid_v2(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
                end                
            end
            distances(k,j) = distances(k,j) - robot.radii(k);
            if collision || distances(k,j) <= 0
                d_c = 0;
                planes = NaN;
                return;
            end
        end        
    end
    d_c = min(min(distances));
end