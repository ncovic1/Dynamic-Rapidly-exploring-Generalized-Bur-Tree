function collision = CollisionLineSegToCuboid(A, B, obstacle, radius)
    collision = false;
    obs = [obstacle(1:3)-radius; obstacle(4:6)+radius];
    radius_new = radius*sqrt(2)/2;
    if prod(A >= obstacle(1:3)-radius_new & A <= obstacle(4:6)+radius_new) || ...
            prod(B >= obstacle(1:3)-radius_new & B <= obstacle(4:6)+radius_new)   % 0.7071 = sqrt(2)/2
        collision = true;
        return;
    elseif A(1) < obs(1) && B(1) < obs(1) || A(1) > obs(4) && B(1) > obs(4) || ...
           A(2) < obs(2) && B(2) < obs(2) || A(2) > obs(5) && B(2) > obs(5) || ...
           A(3) < obs(3) && B(3) < obs(3) || A(3) > obs(6) && B(3) > obs(6)
        collision = false;
        return;
    end
        
    if A(1) <= obstacle(1)      % < x_min
        collision = CollisionLineSegToSide(A, B, obstacle, 1, radius); 
    elseif A(1) >= obstacle(4)  % > x_max
        collision = CollisionLineSegToSide(A, B, obstacle, 4, radius); 
    end
    
    if ~collision && A(2) <= obstacle(2)     % < y_min
        collision = CollisionLineSegToSide(A, B, obstacle, 2, radius); 
    elseif ~collision && A(2) >= obstacle(5) % > y_max
        collision = CollisionLineSegToSide(A, B, obstacle, 5, radius); 
    end
    
    if ~collision && A(3) <= obstacle(3)     % < z_min
        collision = CollisionLineSegToSide(A, B, obstacle, 3, radius); 
    elseif ~collision && A(3) >= obstacle(6) % > z_max
        collision = CollisionLineSegToSide(A, B, obstacle, 6, radius); 
    end
        
end