function collision = CheckCollision(q)
    global robot obstacles;
    
    [xyz, ~] = DirectKinematics(robot, q);    
    for j = 1:size(obstacles.loc,2)
        for k = 1:robot.N_links
            if obstacles.model == 1    
                collision = CollisionLineSegToSphere(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j), robot.radii(k));
            else
                if robot.dim == 2
                    collision = CollisionLineSegToRectangle(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j));
                else
                    collision = CollisionLineSegToCuboid(xyz(:,k), xyz(:,k+1), obstacles.loc(:,j), robot.radii(k));
                end                
            end
            if collision
                return;
            end
        end        
    end
end