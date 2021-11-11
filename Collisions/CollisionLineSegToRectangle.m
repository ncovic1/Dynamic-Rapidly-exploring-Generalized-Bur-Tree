function collision = CollisionLineSegToRectangle(A, B, obstacle)
    collision = false;
    
    if prod(A(1:2) >= obstacle(1:2) & A(1:2) <= obstacle(4:5)) || prod(B(1:2) >= obstacle(1:2) & B(1:2) <= obstacle(4:5))
        collision = true;
        return;
    elseif A(1) < obstacle(1) && B(1) < obstacle(1) || A(1) > obstacle(4) && B(1) > obstacle(4) || ...
        A(2) < obstacle(2) && B(2) < obstacle(2) || A(2) > obstacle(5) && B(2) > obstacle(5)
        collision = false;
        return;
    end
        
    if A(1) <= obstacle(1)      % < x_min
        collision = CollisionLineSegToLineSeg(A, B, obstacle([1,2,3]), obstacle([1,5,3]));  
    elseif A(1) >= obstacle(4)  % > x_max
        collision = CollisionLineSegToLineSeg(A, B, obstacle([4,2,3]), obstacle([4,5,3]));
    end
    
    if ~collision && A(2) < obstacle(2)     % < y_min
        collision = CollisionLineSegToLineSeg(A, B, obstacle([1,2,3]), obstacle([4,2,3]));
    elseif ~collision && A(2) > obstacle(5) % > y_max
        collision = CollisionLineSegToLineSeg(A, B, obstacle([1,5,3]), obstacle([4,5,3]));
    end
end