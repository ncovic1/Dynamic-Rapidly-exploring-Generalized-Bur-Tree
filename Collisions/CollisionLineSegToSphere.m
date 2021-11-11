function collision = CollisionLineSegToSphere(A, B, obstacle, radius)
    collision = false; 
    radius_new = obstacle(4)+radius;
    
    if norm(A-obstacle(1:3)) <= radius_new || norm(B-obstacle(1:3)) <= radius_new
        collision = true;
    else
        a = norm(B-A)^2;
        b = 2*(A'*B - B'*obstacle(1:3) + A'*obstacle(1:3) - norm(A)^2);
        c = norm(A)^2 + norm(obstacle(1:3))^2 - 2*A'*obstacle(1:3) - radius_new^2;
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