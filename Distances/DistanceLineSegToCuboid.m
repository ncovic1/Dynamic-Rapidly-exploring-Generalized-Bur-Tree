function [collision, d_c, plane] = DistanceLineSegToCuboid(A, B, obstacle)
    collision = false;
    d_c = inf;  
    plane = zeros(6,1);
    projections = zeros(length(obstacle),2);     % Determines whether projections on obstacle exist. First column is for point A, and second is for point B
    AB = [A, B];
    dist_AB_obs = [inf, inf];    % Distances of A and B to obstacle (if projections exist)
    
    ProjectionLineSegOnSide(2, 3, 1, 5, 6, 4);   % Projection on x_min or x_max
    ProjectionLineSegOnSide(1, 3, 2, 4, 6, 5);   % Projection on y_min or y_max
    ProjectionLineSegOnSide(1, 2, 3, 4, 5, 6);   % Projection on z_min or z_max     
    if collision
        d_c = 0;
        return;
    end
    
    M = max(sum(projections,2));
    if M == 0       % There is no projection of any point
        CheckOtherCases();
    else            % Projection of one or two points exists
        [d_c, ind] = min(dist_AB_obs);
        P2 = AB(:,ind);
        [~, I] = max(projections(:,ind));
        if I == 1 || I == 4
            P1 = [obstacle(I); AB(2:3,ind)];
        elseif I == 2 || I == 5
            P1 = [AB(1,ind); obstacle(I); AB(3,ind)];
        elseif I == 3 || I == 6
            P1 = [AB(1:2,ind); obstacle(I)];
        end
        plane = [P1; P2-P1];
        
        if M == 1 && ind == 1   % Projection of A exists, but projection of B does not exist
            CheckEdges(B, ind);
        elseif M == 1           % Projection of B exists, but projection of A does not exist
            CheckEdges(A, ind);
        end
    end
    
    if collision
        d_c = 0;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function ProjectionLineSegOnSide(min1, min2, min3, max1, max2, max3)
        % min3 and max3 corresponds to coordinate which is constant
        for i = 1:2
            if AB(min1,i) >= obstacle(min1) && AB(min1,i) <= obstacle(max1) && ...
                    AB(min2,i) >= obstacle(min2) && AB(min2,i) <= obstacle(max2)
                if AB(min3,i) >= obstacle(min3) && AB(min3,i) <= obstacle(max3)
                    collision = true;
                    return;
                elseif AB(min3,i) <= obstacle(min3)
                    projections(min3,i) = true;
                    dist_AB_obs(i) = obstacle(min3) - AB(min3,i);
                elseif AB(min3,i) >= obstacle(max3)
                    projections(max3,i) = true;
                    dist_AB_obs(i) = AB(min3,i) - obstacle(max3);
                end
            end
        end
    end


    function CheckEdges(point, k)
        if projections(1,k)  % Projection on x_min
            collision = CollisionLineSegToSide(A, B, obstacle, 1, 0);
            if ~collision
                line_segments = ChooseLineSegments(point([2,3]), obstacle(2), obstacle(3), obstacle(5), obstacle(6));
                line_segments = [obstacle(1)*ones(1,size(line_segments,2)); line_segments];
            else
                return;
            end
        elseif projections(4,k)  % Projection on x_max    
            collision = CollisionLineSegToSide(A, B, obstacle, 4, 0);
            if ~collision            
                line_segments = ChooseLineSegments(point([2,3]), obstacle(2), obstacle(3), obstacle(5), obstacle(6));
                line_segments = [obstacle(4)*ones(1,size(line_segments,2)); line_segments];
            else
                return;
            end
        elseif projections(2,k)  % Projection on y_min
            collision = CollisionLineSegToSide(A, B, obstacle, 2, 0);
            if ~collision
                line_segments = ChooseLineSegments(point([1,3]), obstacle(1), obstacle(3), obstacle(4), obstacle(6));
                line_segments = [line_segments(1,:); obstacle(2)*ones(1,size(line_segments,2)); line_segments(2,:)];
            else
                return;
            end
        elseif projections(5,k)  % Projection on y_max    
            collision = CollisionLineSegToSide(A, B, obstacle, 5, 0);
            if ~collision            
                line_segments = ChooseLineSegments(point([1,3]), obstacle(1), obstacle(3), obstacle(4), obstacle(6));
                line_segments = [line_segments(1,:); obstacle(5)*ones(1,size(line_segments,2)); line_segments(2,:)];
            else
                return;
            end
        elseif projections(3,k)  % Projection on z_min
            collision = CollisionLineSegToSide(A, B, obstacle, 3, 0);
            if ~collision
                line_segments = ChooseLineSegments(point([1,2]), obstacle(1), obstacle(2), obstacle(4), obstacle(5));
                line_segments = [line_segments; obstacle(3)*ones(1,size(line_segments,2))];
            else
                return;
            end
        elseif projections(6,k)  % Projection on z_max 
            collision = CollisionLineSegToSide(A, B, obstacle, 6, 0);
            if ~collision               
                line_segments = ChooseLineSegments(point([1,2]), obstacle(1), obstacle(2), obstacle(4), obstacle(5));
                line_segments = [line_segments; obstacle(6)*ones(1,size(line_segments,2))];
            else
                return;
            end
        end
        
        DistanceToMoreLineSegments(line_segments);
    end

    
    function line_segments = ChooseLineSegments(point, min1, min2, max1, max2)
        line_segments = [];          
        if point(1) < min1
            line_segments = [line_segments, [min1; min2], [min1; max2]];
        elseif point(1) > max1
            line_segments = [line_segments, [max1; min2], [max1; max2]];
        end
        if point(2) < min2
            line_segments = [line_segments, [min1; min2], [max1; min2]];
        elseif point(2) > max2
            line_segments = [line_segments, [min1; max2], [max1; max2]];
        end
            
    end


    function CheckOtherCases()
        if A(1) < obstacle(1) && B(1) < obstacle(1)
            if A(2) < obstacle(2) && B(2) < obstacle(2)
                if A(3) < obstacle(3) && B(3) < obstacle(3) % < x_min, < y_min, < z_min
                    [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([1,2,3]));
                elseif A(3) > obstacle(6) && B(3) > obstacle(6) % < x_min, < y_min, > z_max
                    [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([1,2,6]));
                else    % < x_min, < y_min
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,2,3]), obstacle([1,2,6]));                    
                end
            elseif A(2) > obstacle(5) && B(2) > obstacle(5)
                if A(3) < obstacle(3) && B(3) < obstacle(3) % < x_min, > y_max, < z_min
                    [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([1,5,3]));                        
                elseif A(3) > obstacle(6) && B(3) > obstacle(6) % < x_min, > y_max, > z_max
                    [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([1,5,6]));
                else    % < x_min, > y_max
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,5,3]), obstacle([1,5,6]));                     
                end                    
            else
                if A(3) < obstacle(3) && B(3) < obstacle(3) % < x_min, < z_min
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,2,3]), obstacle([1,5,3]));
                elseif A(3) > obstacle(6) && B(3) > obstacle(6) % < x_min, > z_max
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,2,6]), obstacle([1,5,6]));
                else    % < x_min
                    DistanceToMoreLineSegments([obstacle([1,2,3]), obstacle([1,5,3]), obstacle([1,5,3]), obstacle([1,5,6]), ...
                        obstacle([1,5,6]), obstacle([1,2,6]), obstacle([1,2,6]), obstacle([1,2,3])]);
                end      
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif A(1) > obstacle(4) && B(1) > obstacle(4)
            if A(2) < obstacle(2) && B(2) < obstacle(2)
                if A(3) < obstacle(3) && B(3) < obstacle(3) % > x_max, < y_min, < z_min
                    [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([4,2,3]));
                elseif A(3) > obstacle(6) && B(3) > obstacle(6) % > x_max, < y_min, > z_max
                    [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([4,2,6]));                            
                else    % > x_max, < y_min
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([4,2,3]), obstacle([4,2,6]));                    
                end
            elseif A(2) > obstacle(5) && B(2) > obstacle(5)
                if A(3) < obstacle(3) && B(3) < obstacle(3) % > x_max, > y_max, < z_min
                    [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([4,5,3]));
                elseif A(3) > obstacle(6) && B(3) > obstacle(6) % > x_max, > y_max, > z_max
                    [collision, d_c, plane] = DistanceLineSegToPoint(A, B, obstacle([4,5,6]));
                else    % > x_max, > y_max
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([4,5,3]), obstacle([4,5,6]));                     
                end                    
            else
                if A(3) < obstacle(3) && B(3) < obstacle(3) % > x_max, < z_min
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([4,2,3]), obstacle([4,5,3]));
                elseif A(3) > obstacle(6) && B(3) > obstacle(6) % > x_max, > z_max
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([4,2,6]), obstacle([4,5,6]));
                else    % > x_max
                    DistanceToMoreLineSegments([obstacle([4,2,3]), obstacle([4,5,3]), obstacle([4,5,3]), obstacle([4,5,6]), ...
                        obstacle([4,5,6]), obstacle([4,2,6]), obstacle([4,2,6]), obstacle([4,2,3])]);
                end      
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        else
            if A(2) < obstacle(2) && B(2) < obstacle(2)
                if A(3) < obstacle(3) && B(3) < obstacle(3) % < y_min, < z_min
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,2,3]), obstacle([4,2,3])); 
                elseif A(3) > obstacle(6) && B(3) > obstacle(6) % < y_min, > z_max
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,2,6]), obstacle([4,2,6])); 
                else    % < y_min
                    DistanceToMoreLineSegments([obstacle([1,2,3]), obstacle([4,2,3]), obstacle([4,2,3]), obstacle([4,2,6]), ...
                        obstacle([4,2,6]), obstacle([1,2,6]), obstacle([1,2,6]), obstacle([1,2,3])]);                        
                end
            elseif A(2) > obstacle(5) && B(2) > obstacle(5)
                if A(3) < obstacle(3) && B(3) < obstacle(3) % > y_max, < z_min
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,5,3]), obstacle([4,5,3]));                        
                elseif A(3) > obstacle(6) && B(3) > obstacle(6) % > y_max, > z_max
                    [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, obstacle([1,5,6]), obstacle([4,5,6]));                             
                else    % > y_max
                    DistanceToMoreLineSegments([obstacle([1,5,3]), obstacle([4,5,3]), obstacle([4,5,3]), obstacle([4,5,6]), ...
                        obstacle([4,5,6]), obstacle([1,5,6]), obstacle([1,5,6]), obstacle([1,5,3])]);  
                end                    
            else
                if A(3) < obstacle(3) && B(3) < obstacle(3) % < z_min
                    DistanceToMoreLineSegments([obstacle([1,2,3]), obstacle([4,2,3]), obstacle([4,2,3]), obstacle([4,5,3]), ...
                        obstacle([4,5,3]), obstacle([1,5,3]), obstacle([1,5,3]), obstacle([1,2,3])]);                         
                elseif A(3) > obstacle(6) && B(3) > obstacle(6) % > z_max
                    DistanceToMoreLineSegments([obstacle([1,2,6]), obstacle([4,2,6]), obstacle([4,2,6]), obstacle([4,5,6]), ...
                        obstacle([4,5,6]), obstacle([1,5,6]), obstacle([1,5,6]), obstacle([1,2,6])]);   
                else
                    for kk = 1:6    % Check collision with all sides
                        collision = CollisionLineSegToSide(A, B, obstacle, kk, 0);
                        if collision
                            break;
                        end
                    end
                    if ~collision
                        DistanceToMoreLineSegments([obstacle([1,2,3]), obstacle([4,2,3]), obstacle([4,2,3]), obstacle([4,5,3]), ...
                            obstacle([4,5,3]), obstacle([1,5,3]), obstacle([1,5,3]), obstacle([1,2,3]),...
                            obstacle([1,2,6]), obstacle([4,2,6]), obstacle([4,2,6]), obstacle([4,5,6]), ...
                            obstacle([4,5,6]), obstacle([1,5,6]), obstacle([1,5,6]), obstacle([1,2,6]),...
                            obstacle([1,2,3]), obstacle([1,2,6]), obstacle([4,2,3]), obstacle([4,2,6]), ...
                            obstacle([4,5,3]), obstacle([4,5,6]), obstacle([1,5,3]), obstacle([1,5,6])]);
                    end
                end      
            end
        end
    end
    

    function DistanceToMoreLineSegments(line_segments)        
        for k = 1:2:size(line_segments,2)
            [collision, d_c_temp, plane_temp] = DistanceLineSegToLineSeg(A, B, line_segments(:,k), line_segments(:,k+1));
            if collision
                return;
            end
            if d_c_temp < d_c
                d_c = d_c_temp;
                plane = plane_temp;
            end
        end
    end

end
