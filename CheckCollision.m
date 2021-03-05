function [collision, d_c, separatrix] = CheckCollision(robot, obstacles, q)
    d_c = 0;
    N_obstacles = size(obstacles.loc,2);
    separatrix = zeros(robot.N_DOF,N_obstacles,6);
    distance = zeros(robot.N_links, N_obstacles);
    [xyz, ~] = DirectKinematics(robot, q);
    
    for j = 1:N_obstacles
        for k = 1:robot.N_links
            if obstacles.model == 1    
                [collision, distance(k,j), separatrix(k,j,:)] = DistanceLineSegToSphere(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
            else
                if robot.dim == 2
                    [collision, distance(k,j), separatrix(k,j,:)] = DistanceLineSegToRectangle(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
                else
                    [collision, distance(k,j), separatrix(k,j,:)] = DistanceLineSegToCuboid(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
%                     [collision, distance(k,j), separatrix(k,j,:)] = DistanceLineSegToCuboid_v2(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
                end                
            end
            distance(k,j) = distance(k,j) - robot.radii(k);
            if collision || distance(k,j) < 0
                distance(k,j) = 0;
                return;
            end
        end        
    end
    d_c = min(min(distance));
end