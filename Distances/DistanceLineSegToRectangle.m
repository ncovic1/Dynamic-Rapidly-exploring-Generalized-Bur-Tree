function [collision, d_c, plane] = DistanceLineSegToRectangle(A, B, obstacle)
    collision = false;
    plane = zeros(6,1);
    
    if prod(A(1:2) >= obstacle(1:2) & A(1:2) <= obstacle(4:5)) || prod(B(1:2) >= obstacle(1:2) & B(1:2) <= obstacle(4:5))
        collision = true;
        d_c = 0;
        return;
    end
    
    if A(1) < obstacle(1) && B(1) < obstacle(1)
        if A(2) < obstacle(2) && B(2) < obstacle(2) % < x_min, < y_min
            [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([1,2,3]));
        elseif A(2) > obstacle(5) && B(2) > obstacle(5) % < x_min, > y_max
            [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([1,5,3]));
        else    % < x_min
            [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,2,3]), obstacle([1,5,3]));
        end
    elseif A(1) > obstacle(4) && B(1) > obstacle(4)
        if A(2) < obstacle(2) && B(2) < obstacle(2) % > x_max, < y_min
            [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([4,2,3]));
        elseif A(2) > obstacle(5) && B(2) > obstacle(5) % > x_max, > y_max
            [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([4,5,3]));
        else    % > x_max
            [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([4,2,3]), obstacle([4,5,3]));
        end
    else
        if A(2) < obstacle(2) && B(2) < obstacle(2) % < y_min
            [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,2,3]), obstacle([4,2,3]));
        elseif A(2) > obstacle(5) && B(2) > obstacle(5) % > y_max
            [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,5,3]), obstacle([4,5,3]));
        else
            C = [obstacle(1), obstacle(4), obstacle(4), obstacle(1) 
                 obstacle(2), obstacle(2), obstacle(5), obstacle(5)];
            D = [obstacle(4), obstacle(4), obstacle(1), obstacle(1) 
                 obstacle(2), obstacle(5), obstacle(5), obstacle(2)];
            d_c = inf;
            for kk = 1:size(C,2)
                [collision, d_c_temp, plane_temp] = DistanceLineSegToLineSeg(A, B, [C(:,kk); 0], [D(:,kk); 0]);
                if collision
                    d_c = 0;
                    return;
                end
                if d_c_temp < d_c
                    d_c = d_c_temp;
                    plane = plane_temp;
                end
            end
        end
    end
end
