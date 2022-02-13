function collision = CollisionLineSegToSphere(A, B, obstacle, radius)
    collision = false; 
    radius = radius + obstacle(4);
    
    if norm(A-obstacle(1:3)) <= radius || norm(B-obstacle(1:3)) <= radius
        collision = true;
    else
        a = norm(B-A)^2;
        b = 2*(A'*B + (A-B)'*obstacle(1:3) - norm(A)^2);
        c = norm(A)^2 + norm(obstacle(1:3))^2 - 2*A'*obstacle(1:3) - radius^2;
        D = b^2-4*a*c;
        if D >= 0
            t1 = (-b+sqrt(D))/(2*a);
            t2 = (-b-sqrt(D))/(2*a);
            if t1 >= 0 && t1 <= 1 || t2 >= 0 && t2 <= 1
                collision = true;
            end         
        end
    end
    
    
end