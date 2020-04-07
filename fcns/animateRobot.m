function animateRobot(tin,Xin,p)

f = figure;
v = VideoWriter('video.avi');
open(v)
set(f, 'doublebuffer', 'on');

N = p.N_animate;
[t,X] = even_sample(tin,Xin,N);
L_boom = 1.1*p.params(3);
HIP = [];
FOOT = [];
COM = [];

nt = length(t);
for ii = 1:nt
    X_ = X(ii,:)';
    HIP = [HIP fcn_p2(X_(1:4),p.params)];      %Hip trajectory
    FOOT = [FOOT fcn_p_toe(X_(1:4),p.params)]; %Foot trajectory
    COM = [COM fcn_CoM(X_(1:4),p.params)]; %CoMtrajectory
    
    set(gcf, 'Position',  [100, 100, 1000, 600])
    
    %Top plot
    subplot(2,2,1:2)
    plot3(HIP(1,:),HIP(2,:),HIP(3,:),'m','LineWidth',1);
    hold on
    plot3(FOOT(1,:),FOOT(2,:),FOOT(3,:),'c','LineWidth',1);
    plot3(COM(1,:),COM(2,:),COM(3,:),'g','LineWidth',1);
    grid on; box on;
    plotRobot(X_,p);
    axis equal;
    view(3)
    xlim([-L_boom L_boom])
    ylim([-L_boom L_boom])
    zlim([0 .5])
    legend('Hip traj.','Foot traj.','CoM traj.')
    hold off
    
    %Bottom plot
    subplot(2,2,3:4)
    plotRobot(X_,p);
    plot3(HIP(1,:),HIP(2,:),HIP(3,:),'m','LineWidth',1);
    plot3(FOOT(1,:),FOOT(2,:),FOOT(3,:),'c','LineWidth',1);
    plot3(COM(1,:),COM(2,:),COM(3,:),'g','LineWidth',1);
    grid on; box on;
    axis equal;
    view(3)
    xlim([-L_boom L_boom])
    ylim([-L_boom L_boom])
    zlim([0 .5])
    title(['Time =' num2str(t(ii),'%6.2f') 's'])
    view([X_(1)*180/pi+90 0])
    hold off
    
    
     F = getframe(f);
    drawnow
     writeVideo(v,F)
end

close(v)




