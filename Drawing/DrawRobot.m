function graphics = DrawRobot(q, color, transp, LineWidth, MarkerSize)
    % robot.DH_table is n x 3 matrix with each row containing parameters alpha, a, d
    % for that DOF. It is assumed that all DOFs are rotational and parameter
    % theta is taken from joint vector q.
    % transp is transparency from 0 to 1;
    
    global robot;
    
    if robot.dim == 2
        k = 1;
        subplot(1,2,1);
        [xyz, ~] = DirectKinematics(robot, q);
        for j = 1:robot.N_links      
            graphics{k} = plot([xyz(1,j),xyz(1,j+1)], [xyz(2,j),xyz(2,j+1)], 'Color',color,'LineWidth',LineWidth); hold on; k = k+1; %drawnow;
            graphics{k} = plot(xyz(1,j),xyz(2,j),'Color','k','Marker','.','MarkerSize',MarkerSize); hold on; k = k+1;
        end
        L = sum(robot.DH_table(:,1:2),2);
        grid on; xlabel('$x$'); ylabel('$y$'); axis equal; axis(sum(L)*[-1, 1, -1, 1]);
        
        subplot(1,2,2);
        graphics{k} = plot(q(1), q(2), 'Color',color,'Marker','.','MarkerSize',MarkerSize); hold on; k = k+1;
        grid on; xlabel('$\theta_1$'); ylabel('$\theta_2$'); axis equal; axis([robot.range(1,:), robot.range(2,:)]);
        
    else
        d = robot.DH_table(:,1);
        a = robot.DH_table(:,2);
        alpha = robot.DH_table(:,3);

        Tc = eye(4);
        Ps = zeros(3,robot.N_DOF+1);
        zs = zeros(3,robot.N_DOF);
        for i=1:robot.N_DOF
            zs(:,i) = Tc(1:3,3);
            Tc = Tc*DHmatrix(q(i)+robot.offsets(i), d(i), a(i), alpha(i));
            Ps(:,i+1) = Tc(1:3,4);
        end

        s = 1.618*robot.radii;
        r1 = 1.2*robot.radii;

        k = 1;
        ii = 1;
        for i = 1:robot.N_DOF
            P1 = Ps(:,i);
            P2 = Ps(:,i+1);
            if P1 == P2
                continue;
            end

        %   Plotting the joint
    %         zi = zs(:,i);
    %         X1 = P1+s(i)*zi;
    %         X2 = P1-s(i)*zi;
    %         graphics{k} = drawCylinder([X1' X2' r1(i)], 'closed','FaceColor',[0.5 0.5 0.5]*0.25,'FaceAlpha',transp); k = k+1;

        %   Plotting the link
    %         graphics{k} = drawCylinder([P1' P2' robot.radii(i)], 'closed','FaceColor',color,'FaceAlpha',transp); k = k+1;
            graphics{k} = drawCapsule([P1' P2' robot.radii(ii)],'FaceColor',color,'FaceAlpha',transp); hold on; k = k+1;

            graphics{k} = plot3([P1(1), P2(1)], [P1(2), P2(2)], [P1(3), P2(3)], 'Color','k','LineWidth',2); k = k+1;
            graphics{k} = plot3(P1(1), P1(2), P1(3), 'Color','k','Marker','.','MarkerSize',15); k = k+1;

            ii = ii+1;
        end

        if robot.radii(end) > 0
            pp = robot.radii(end);
        else
            pp = 0.1;
        end
        plane = [Ps(:,end); Tc(1:3,1); Tc(1:3,2)]';
        graphics{k} = drawPlatform(plane, [2*pp, 5*pp], 'FaceColor', color, 'FaceAlpha', transp); k = k+1;

    %      camlight(140,30); lighting phong; material dull;
    end

end
