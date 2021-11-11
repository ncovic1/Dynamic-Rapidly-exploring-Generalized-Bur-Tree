function [collision, distance, plane] = DistanceLineSegToSphere(A, B, obstacle)
    collision = false;
    plane = zeros(6,1);
    
    AO = norm(A-obstacle(1:3));
    distance = AO-obstacle(4);
    if distance <= 0
        collision = true;
        return;
    end
    BO = norm(B-obstacle(1:3));
    distance_temp = BO-obstacle(4);
    if distance_temp < distance
        distance = distance_temp;
    end
    if distance_temp < 0
        collision = true;
        return;
    end    
    AB = norm(A-B);      
    s = (AB+AO+BO)/2;
    P = sqrt(s*(s-AB)*(s-AO)*(s-BO));
    h = 2*P/AB;
    distance_temp = h-obstacle(4);
    alpha = acos((AO^2+AB^2-BO^2)/(2*AO*AB));
    if alpha <= pi/2
        beta = acos((BO^2+AB^2-AO^2)/(2*BO*AB));
        if beta <= pi/2    % acute triangle
            distance = distance_temp;
            if distance_temp < 0
                collision = true;
                return;
            end
        end
    end

    if alpha <= pi/2 && beta <= pi/2    % acute triangle
        S = A + AO*cos(alpha)/AB*(B-A);
        D = S + distance/norm(obstacle(1:3)-S)*(obstacle(1:3)-S);             
    else                
        if alpha > pi/2
            D = A + distance/AO*(obstacle(1:3)-A);  
            S = A;
        else
            D = B + distance/BO*(obstacle(1:3)-B);  
            S = B;
        end
    end
    plane = [D; S-D];  
end
