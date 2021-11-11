function [collision, distance, plane] = DistanceLineSegToRectangle(A, B, obstacle)
    collision = false;
    distance = inf;
    plane = zeros(6,1);
    
    if prod(A(1:2) >= obstacle(1:2) & A(1:2) <= obstacle(4:5)) || prod(B(1:2) >= obstacle(1:2) & B(1:2) <= obstacle(4:5))
        collision = true;
        distance = 0;
        return;
    end
    
    if A(1) < obstacle(1) && B(1) < obstacle(1)
        if A(2) < obstacle(2) && B(2) < obstacle(2) % < x_min, < y_min
            [collision, distance, D_opt, S_opt] = DistanceLineSegToPoint(A, B, obstacle([1,2,3]));
        elseif A(2) > obstacle(5) && B(2) > obstacle(5) % < x_min, > y_max
            [collision, distance, D_opt, S_opt] = DistanceLineSegToPoint(A, B, obstacle([1,5,3]));
        else    % < x_min
            [collision, distance, D_opt, S_opt] = DistanceLineSegToLineSeg(A, B, obstacle([1,2,3]), obstacle([1,5,3]));
        end
    elseif A(1) > obstacle(4) && B(1) > obstacle(4)
        if A(2) < obstacle(2) && B(2) < obstacle(2) % > x_max, < y_min
            [collision, distance, D_opt, S_opt] = DistanceLineSegToPoint(A, B, obstacle([4,2,3]));
        elseif A(2) > obstacle(5) && B(2) > obstacle(5) % > x_max, > y_max
            [collision, distance, D_opt, S_opt] = DistanceLineSegToPoint(A, B, obstacle([4,5,3]));
        else    % > x_max
            [collision, distance, D_opt, S_opt] = DistanceLineSegToLineSeg(A, B, obstacle([4,2,3]), obstacle([4,5,3]));
        end
    else
        if A(2) < obstacle(2) && B(2) < obstacle(2) % < y_min
            [collision, distance, D_opt, S_opt] = DistanceLineSegToLineSeg(A, B, obstacle([1,2,3]), obstacle([4,2,3]));
        elseif A(2) > obstacle(5) && B(2) > obstacle(5) % > y_max
            [collision, distance, D_opt, S_opt] = DistanceLineSegToLineSeg(A, B, obstacle([1,5,3]), obstacle([4,5,3]));
        else
            C = [obstacle(1), obstacle(4), obstacle(4), obstacle(1) 
                 obstacle(2), obstacle(2), obstacle(5), obstacle(5)];
            D = [obstacle(4), obstacle(4), obstacle(1), obstacle(1) 
                 obstacle(2), obstacle(5), obstacle(5), obstacle(2)];
            for kk = 1:size(C,2)
                [collision, d_c_temp, D_temp, S_temp] = DistanceLineSegToLineSeg(A, B, [C(:,kk); 0], [D(:,kk); 0]);
                if collision
                    distance = 0;
                    return;
                end
                if d_c_temp < distance
                    distance = d_c_temp;
                    D_opt = D_temp;
                    S_opt = S_temp;
                end
            end
        end
    end
    
    if ~collision
        plane = [D_opt; S_opt-D_opt];    
    end
end
