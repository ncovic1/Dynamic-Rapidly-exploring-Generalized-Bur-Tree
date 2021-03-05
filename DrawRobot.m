function graphics = DrawRobot(robot, q, color, transp)
    % robot.DH_table is n x 3 matrix with each row containing parameters alpha, a, d
    % for that DOF. It is assumed that all DOFs are rotational and parameter
    % theta is taken from joint vector q.
    % transp is transparency from 0 to 1;
    
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
    for i = 1:robot.N_DOF
        P1 = Ps(:,i);
        P2 = Ps(:,i+1);
        
    %   Plotting the joint
%         zi = zs(:,i);
%         X1 = P1+s(i)*zi;
%         X2 = P1-s(i)*zi;
%         graphics{k} = drawCylinder([X1' X2' r1(i)], 'closed','FaceColor',[0.5 0.5 0.5]*0.25,'FaceAlpha',transp); k = k+1;

    %   Plotting the link
%         graphics{k} = drawCylinder([P1' P2' robot.radii(i)], 'closed','FaceColor',color,'FaceAlpha',transp); k = k+1;
        graphics{k} = drawCapsule([P1' P2' robot.radii(i)],'FaceColor',color,'FaceAlpha',transp); hold on; k = k+1;
        
        graphics{k} = plot3([P1(1), P2(1)], [P1(2), P2(2)], [P1(3), P2(3)], 'Color','k','LineWidth',2); k = k+1;
        graphics{k} = plot3(P1(1), P1(2), P1(3), 'Color','k','Marker','.','MarkerSize',15); k = k+1;

    end
    
    plane = [Ps(:,end); Tc(1:3,1); Tc(1:3,2)]';
    graphics{k} = drawPlatform(plane, [2*robot.radii(i), 5*robot.radii(i)], 'FaceColor', color, 'FaceAlpha', transp); k = k+1;

%      camlight(140,30); lighting phong; material dull;

end
