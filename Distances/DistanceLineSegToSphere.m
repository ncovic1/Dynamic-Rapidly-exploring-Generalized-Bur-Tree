function [collision, d_c, plane] = DistanceLineSegToSphere(A, B, obstacle)
    collision = false;
    plane = zeros(6,1);
    
    AO = norm(A-obstacle(1:3));
    d_c = AO-obstacle(4);
    if d_c <= 0
        collision = true;
        return;
    end
    BO = norm(B-obstacle(1:3));
    d_c_temp = BO-obstacle(4);
    if d_c_temp <= 0
        collision = true;
        return;
    end
    if d_c_temp < d_c
        d_c = d_c_temp;
    end
    AB = norm(A-B);
    s = (AB+AO+BO)/2;
    d_c_temp = 2*sqrt(s*(s-AB)*(s-AO)*(s-BO))/AB - obstacle(4);    % h = 2*P/AB; d_c_temp = h - obs_loc(3);
    alpha = acos((AO^2+AB^2-BO^2)/(2*AO*AB));
    if alpha <= pi/2
        beta = acos((BO^2+AB^2-AO^2)/(2*BO*AB));
        if beta <= pi/2    % Acute triangle
            d_c = d_c_temp;
            if d_c_temp <= 0
                collision = true;
                return;
            end            
            P2 = A + AO*cos(alpha)/AB*(B-A);
            P1 = P2 + d_c/norm(obstacle(1:3)-P2)*(obstacle(1:3)-P2);
        else            
            P1 = B + d_c/BO*(obstacle(1:3)-B);  
            P2 = B;
        end
    else
        P1 = A + d_c/AO*(obstacle(1:3)-A);  
        P2 = A;
    end
    plane = [P1; P2-P1];  
end
