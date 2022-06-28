function collision = CollisionLineSegToSide(A, B, obstacle, k, radius)
% k determines which coordinate is constant: [1,2,3,4,5,6] = [x_min, y_min, z_min, x_max, y_max, z_max]
    
    collision = false;
    if k > 3
        kk = k-3;
    else
        kk = k;
    end
    t = (obstacle(k)-A(kk))/(B(kk)-A(kk));
    side = [obstacle([1:kk-1,kk+1:3]); obstacle([4:kk+2,kk+4:6])];
    A_proj = A([1:kk-1,kk+1:3]);
    B_proj = B([1:kk-1,kk+1:3]);
    M_t = A_proj + t*(B_proj-A_proj);
    if t >= 0 && t <= 1
        if M_t >= side(1:2) & M_t <= side(3:4)  % Whether point lies on a side
            collision = true;
            return;
        end
    end
    
    if radius > 0     % Considering collision between cylinder and side
        if A_proj >= side(1:2) & A_proj <= side(3:4)
            if B_proj >= side(1:2) & B_proj <= side(3:4)    % Both projections
                distance(1) = norm(A-[A_proj(1:kk-1); obstacle(k); A_proj(kk:end)]);
                distance(2) = norm(B-[B_proj(1:kk-1); obstacle(k); B_proj(kk:end)]);
                if min(distance) <= radius
                    collision = true;
                end
            else
                Check(B_proj);
                if min([distance, norm(A-[A_proj(1:kk-1); obstacle(k); A_proj(kk:end)])]) <= radius
                    collision = true;
                end
            end
        elseif B_proj >= side(1:2) & B_proj <= side(3:4)
            Check(A_proj);
            if min([distance, norm(B-[B_proj(1:kk-1); obstacle(k); B_proj(kk:end)])]) <= radius
                collision = true;
            end
        else
            Check(A_proj);
            if collision
                return;
            end
            Check(B_proj);
        end
    end
    
    
    function Check(point)
        if point(1) < side(1)
            DLSTLS(side([1,2]), side([1,4]));
        elseif point(1) > side(3)
            DLSTLS(side([3,2]), side([3,4]));
        end

        if ~collision && point(2) < side(2)
            DLSTLS(side([1,2]), side([3,2]));
        elseif ~collision && point(2) > side(4)
            DLSTLS(side([1,4]), side([3,4]));
        end        
    end
    
    function DLSTLS(C, D)
        [~, distance, ~] = DistanceLineSegToLineSeg(A, B, ... 
            [C(1:kk-1); obstacle(k); C(kk:end)], [D(1:kk-1); obstacle(k); D(kk:end)]);
        if distance > radius
            collision = false;
        else
            collision = true;
        end
    end

end


