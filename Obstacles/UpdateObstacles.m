function [obs] = UpdateObstacles(robot, obs)    

eval(['Obstacles_',num2str(robot.dim),num2str(robot.model),num2str(obs.model),num2str(obs.type)]);    

function Obstacles_2111()
    global pp1;
    
    obs_num = 1;
    if obs.loc(1,obs_num) < -0.5
        if obs.loc(2,obs_num) < pp1
            obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
            pp1 = 2;
        else
            obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
            pp1 = 1;
        end                
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + 0*obs.step;
        pp1 = 2;
    end 
end

function Obstacles_2112()
    global pp1 pp2;
    
    obs_num = 1;
    if obs.loc(1,obs_num) < -0.5
        if obs.loc(2,obs_num) < pp1
            obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
            pp1 = 2;
        else
            obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
            pp1 = 1;
        end                
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + 0*obs.step;
        pp1 = 2;
    end  

    obs_num = 2;
    if obs.loc(1,obs_num) < -0.5
        if obs.loc(2,obs_num) > pp2
            obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
            pp2 = -2;
        else
            obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
            pp2 = -1;
        end                
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + 0*obs.step;
        pp2 = -2;
    end
    
end

function Obstacles_2113()
    global pp1 pp2;
    
    R = sqrt(obs.loc(1,1)^2 + obs.loc(2,1)^2);
    for ii = 1:size(obs.loc,2)
        fi = atan2(obs.loc(2,ii), obs.loc(1,ii));
        obs.loc(1,ii) = R*cos(fi + obs.step);
        obs.loc(2,ii) = R*sin(fi + obs.step);
    end
end

function Obstacles_2114()
    global pp1 pp2;
    
    R = sqrt(obs.loc(1,1)^2 + obs.loc(2,1)^2);
    for ii = 1:size(obs.loc,2)
        fi = atan2(obs.loc(2,ii), obs.loc(1,ii));
        obs.loc(1,ii) = R*cos(fi + obs.step);
        obs.loc(2,ii) = R*sin(fi + obs.step);
    end
end

function Obstacles_2121()
    global pp1 pp2;
    
    obs_num = 1;
    if obs.loc(1,obs_num) < -0.5
        if obs.loc(2,obs_num) < pp1
            obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
            obs.loc(5,obs_num) = obs.loc(5,obs_num) + obs.step;
            pp1 = 2;
        else
            obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
            obs.loc(5,obs_num) = obs.loc(5,obs_num) - obs.step;
            pp1 = 0.5;
        end                
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(4,obs_num) = obs.loc(4,obs_num) - obs.step;
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + 0*obs.step;
        obs.loc(5,obs_num) = obs.loc(5,obs_num) + 0*obs.step;
        pp1 = 2;
    end  
end

function Obstacles_2122()
    global pp1 pp2;
    
    obs_num = 1;
    if obs.loc(1,obs_num) < -0.5
        if obs.loc(2,obs_num) < pp1
            obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
            obs.loc(5,obs_num) = obs.loc(5,obs_num) + obs.step;
            pp1 = 2;
        else
            obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
            obs.loc(5,obs_num) = obs.loc(5,obs_num) - obs.step;
            pp1 = 0.5;
        end                
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(4,obs_num) = obs.loc(4,obs_num) - obs.step;
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + 0*obs.step;
        obs.loc(5,obs_num) = obs.loc(5,obs_num) + 0*obs.step;
        pp1 = 2;
    end  

    obs_num = 2;
    if obs.loc(1,obs_num) < -0.5
        if obs.loc(5,obs_num) > pp2
            obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
            obs.loc(5,obs_num) = obs.loc(5,obs_num) - obs.step;
            pp2 = -2;
        else
            obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
            obs.loc(5,obs_num) = obs.loc(5,obs_num) + obs.step;
            pp2 = -0.5;
        end                
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(4,obs_num) = obs.loc(4,obs_num) - obs.step;
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + 0*obs.step;
        obs.loc(5,obs_num) = obs.loc(5,obs_num) + 0*obs.step;
        pp2 = -2;
    end
end

% function Obstacles_2123()
%     global pp1 pp2;
% 
%     R = 1.1;
%     for ii = 1:size(obs.loc,2)
%         fi = atan2((obs.loc(2,ii)+0.25)/R, (obs.loc(1,ii)+0.25)/(1.5*R));
%         obs.loc(1,ii) = 1.5*R*cos(fi + obs.step)-0.25;
%         obs.loc(2,ii) = R*sin(fi + obs.step)-0.25;
%         obs.loc(4,ii) = 1.5*R*cos(fi + obs.step)+0.5-0.25;
%         obs.loc(5,ii) = R*sin(fi + obs.step)+0.5-0.25;
%     end
% end

function Obstacles_2123()
    for ii = 1:size(obs.loc,2)
        pos = (obs.loc(4:6,ii) + obs.loc(1:3,ii)) / 2;
        phi = atan2(pos(2), pos(1));
        vel = [-sin(phi); cos(phi); 0] * obs.max_vel;
        pos_next = pos + vel * obs.delta_t * obs.decimation;
        obs.loc(:,ii) = [pos_next-obs.dim/2; pos_next+obs.dim/2];
    end
end

function Obstacles_2124()
    Obstacles_2123();
end

function Obstacles_21210()
    base_radius = robot.radii(1);
    robot_max_vel = pi;
    WS_center = [0, 0, 0]';
    WS_radius = 2.5;
    delta_time = obs.delta_t * obs.decimation;

    for ii = 1 : size(obs.loc,2)
        tol_radius = max(norm(obs.vel(:,ii)) / robot_max_vel, base_radius);
        pos_ii = (obs.loc(4:6,ii) + obs.loc(1:3,ii)) / 2;
        pos_next = pos_ii + obs.vel(:,ii) * delta_time;
        change = true;
        
        if pos_next(3) < 0
            vec_normal = [0, 0, 1]';
        elseif norm(pos_next - WS_center) > WS_radius
            vec_normal = [-pos_next(1), -pos_next(2), -(pos_next(3) - WS_center(3))]';
        elseif norm(pos_next(1:2)) < tol_radius && pos_next(3) < WS_center(3)
            vec_normal = [pos_next(1), pos_next(2), 0]';
        elseif norm(pos_next - WS_center) < tol_radius
            vec_normal = [pos_next(1), pos_next(2), pos_next(3) - WS_center(3)]';
        else
            obs.loc(:,ii) = [pos_next-obs.dim/2; pos_next+obs.dim/2];
            change = false;
        end

        if change
            t_param = (pos_next - pos_ii)' * vec_normal / norm(vec_normal)^2;
            pos_new = 2*pos_next - pos_ii - 2*t_param * vec_normal;
            obs.loc(:,ii) = [pos_new-obs.dim/2; pos_new+obs.dim/2];
            obs.vel(:,ii) = (pos_new - pos_next) / delta_time;
        end
    end
end

function Obstacles_2221()
    global pp1 pp2;
    
    obs_num = 1;
    if obs.loc(2,obs_num) < pp1
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
        obs.loc(5,obs_num+1) = obs.loc(5,obs_num+1) - obs.step;
        pp1 = 0.8;
    else
        obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
        obs.loc(5,obs_num+1) = obs.loc(5,obs_num+1) + obs.step;
        pp1 = 0.2;
    end    
end

function Obstacles_2321()
    global pp1 pp2;
    
    obs_num = 1;
    if obs.loc(2,obs_num) < pp1
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
        obs.loc(5,obs_num+1) = obs.loc(5,obs_num+1) - obs.step;
        pp1 = 4;
    else
        obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
        obs.loc(5,obs_num+1) = obs.loc(5,obs_num+1) + obs.step;
        pp1 = 0.5;
    end    
end

function Obstacles_3111()
    global pp1 pp2;
    
    obs_num = 1;
    if obs.loc(1,obs_num) < -0.5
        if obs.loc(2,obs_num) < pp1
            obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
            pp1 = 2;
        else
            obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
            pp1 = 1;
        end                
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + 0*obs.step;
        pp1 = 2;
    end    
end

function Obstacles_3112()
    global pp1 pp2;
    
    obs_num = 1;
    if obs.loc(1,obs_num) < -0.5
        if obs.loc(2,obs_num) < pp1
            obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
            pp1 = 2;
        else
            obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
            pp1 = 1;
        end                
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + 0*obs.step;
        pp1 = 2;
    end

    obs_num = 2;
    if obs.loc(1,obs_num) < -0.5
        if obs.loc(2,obs_num) > pp2
            obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
            pp2 = -2;
        else
            obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
            pp2 = -1;
        end                
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + 0*obs.step;
        pp2 = -2;
    end      
end

function Obstacles_3221()
    global pp1 pp2;
    
    R = 0.7;
    a = obs.loc(4,1)-obs.loc(1,1);
    for ii = 1:size(obs.loc,2)
        fi = atan2((obs.loc(2,ii)+0.25)/R, (obs.loc(1,ii)+0.25)/R);
        obs.loc(1,ii) = R*cos(fi + obs.step)-a/2;
        obs.loc(2,ii) = R*sin(fi + obs.step)-a/2;
        obs.loc(4,ii) = R*cos(fi + obs.step)+a/2;
        obs.loc(5,ii) = R*sin(fi + obs.step)+a/2;
    end  
end

function Obstacles_3222()
    global pp1 pp2;
    
    obs_num = 1;
    if obs.loc(2,obs_num) < pp1
        obs.loc(2,obs_num) = obs.loc(2,obs_num) + obs.step;
        obs.loc(5,obs_num+1) = obs.loc(5,obs_num+1) - obs.step;
        pp1 = 2;
    else
        obs.loc(2,obs_num) = obs.loc(2,obs_num) - obs.step;
        obs.loc(5,obs_num+1) = obs.loc(5,obs_num+1) + obs.step;
        pp1 = 0.5;
    end
end

function Obstacles_3223()
    global pp1 pp2;
    
    obs_num = 1;
    if obs.loc(4,obs_num) < pp1
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(4,obs_num) = obs.loc(4,obs_num) + obs.step;
        obs.loc(2,obs_num+1) = obs.loc(2,obs_num+1) - obs.step;
        obs.loc(5,obs_num+1) = obs.loc(5,obs_num+1) + obs.step;
        obs.loc(1,obs_num+2) = obs.loc(1,obs_num+2) - obs.step;
        obs.loc(4,obs_num+2) = obs.loc(4,obs_num+2) + obs.step;
        obs.loc(2,obs_num+3) = obs.loc(2,obs_num+3) - obs.step;
        obs.loc(5,obs_num+3) = obs.loc(5,obs_num+3) + obs.step;
        pp1 = 1;
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) + obs.step;
        obs.loc(4,obs_num) = obs.loc(4,obs_num) - obs.step;
        obs.loc(2,obs_num+1) = obs.loc(2,obs_num+1) + obs.step;
        obs.loc(5,obs_num+1) = obs.loc(5,obs_num+1) - obs.step;
        obs.loc(1,obs_num+2) = obs.loc(1,obs_num+2) + obs.step;
        obs.loc(4,obs_num+2) = obs.loc(4,obs_num+2) - obs.step;
        obs.loc(2,obs_num+3) = obs.loc(2,obs_num+3) + obs.step;
        obs.loc(5,obs_num+3) = obs.loc(5,obs_num+3) - obs.step;
        pp1 = obs.step;
    end
end

function Obstacles_3224()
    global pp1 pp2;
    
    obs_num = 1;
    if obs.loc(4,obs_num) < pp1
        obs.loc(1,obs_num) = obs.loc(1,obs_num) - obs.step;
        obs.loc(4,obs_num) = obs.loc(4,obs_num) + obs.step;
        obs.loc(2,obs_num+1) = obs.loc(2,obs_num+1) - obs.step;
        obs.loc(5,obs_num+1) = obs.loc(5,obs_num+1) + obs.step;
        obs.loc(1,obs_num+2) = obs.loc(1,obs_num+2) - obs.step;
        obs.loc(4,obs_num+2) = obs.loc(4,obs_num+2) + obs.step;
        obs.loc(2,obs_num+3) = obs.loc(2,obs_num+3) - obs.step;
        obs.loc(5,obs_num+3) = obs.loc(5,obs_num+3) + obs.step;
        pp1 = 1;
    else
        obs.loc(1,obs_num) = obs.loc(1,obs_num) + obs.step;
        obs.loc(4,obs_num) = obs.loc(4,obs_num) - obs.step;
        obs.loc(2,obs_num+1) = obs.loc(2,obs_num+1) + obs.step;
        obs.loc(5,obs_num+1) = obs.loc(5,obs_num+1) - obs.step;
        obs.loc(1,obs_num+2) = obs.loc(1,obs_num+2) + obs.step;
        obs.loc(4,obs_num+2) = obs.loc(4,obs_num+2) - obs.step;
        obs.loc(2,obs_num+3) = obs.loc(2,obs_num+3) + obs.step;
        obs.loc(5,obs_num+3) = obs.loc(5,obs_num+3) - obs.step;
        pp1 = obs.step;
    end  
    R = 1.5;
    for ii = obs_num+4:size(obs.loc,2)
        fi = atan2(obs.loc(2,ii)+1, obs.loc(1,ii)+1);
        obs.loc(1,ii) = R*cos(fi + 2*obs.step)-1;
        obs.loc(2,ii) = R*sin(fi + 2*obs.step)-1;
        obs.loc(4,ii) = R*cos(fi + 2*obs.step)+2-1;
        obs.loc(5,ii) = R*sin(fi + 2*obs.step)+2-1;
    end
end

function Obstacles_3225()
    global pp1 pp2;
    
    if obs.loc(2,1) < pp1
        for ii = 1:4
            obs.loc(2,ii) = obs.loc(2,ii) + obs.step;
            obs.loc(1,ii+4) = obs.loc(1,ii+4) - 0.5*obs.step;
            obs.loc(4,ii+4) = obs.loc(4,ii+4) - 0.5*obs.step;
        end
        pp1 = 0.7;
    else
        for ii = 1:4
            obs.loc(2,ii) = obs.loc(2,ii) - obs.step;
            obs.loc(1,ii+4) = obs.loc(1,ii+4) + 0.5*obs.step;
            obs.loc(4,ii+4) = obs.loc(4,ii+4) + 0.5*obs.step;
        end
        pp1 = 0.4;
    end
end

end