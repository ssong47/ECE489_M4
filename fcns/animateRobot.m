function [t,HIP] = animateRobot(tin,Xin,Uin,Fin,p)

f = figure;
v = VideoWriter('video.avi');
open(v)
set(f, 'doublebuffer', 'on');

N = p.N_animate;
R = 0.44;
[t,X] = even_sample(tin,Xin,N);
[t,U] = even_sample(tin,Uin,N);
[t,F] = even_sample(tin,Fin,N);
L_limit = 0.5;
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
    
    % Robot
    subplot(3,3,[1,2,4,5])
    hold on;grid on;box on;
    plot3(HIP(1,:),HIP(2,:),HIP(3,:),'m','LineWidth',1);
    plot3(FOOT(1,:),FOOT(2,:),FOOT(3,:),'c','LineWidth',1);
    plot3(COM(1,:),COM(2,:),COM(3,:),'g','LineWidth',1);
    title(['Time =' num2str(t(ii),'%6.2f') 's'])
    grid on; box on;
    plotRobot(X_,p);
    axis equal;
    view(3)
    xlim([-L_limit L_limit])
    ylim([-L_limit L_limit])
    zlim([0 .5])
    
    % Saggital plane
    subplot(3,3,3)
    hold on;grid on; box on;
    plotRobot(X_,p);
    plot3(HIP(1,:),HIP(2,:),HIP(3,:),'m','LineWidth',1);
    plot3(FOOT(1,:),FOOT(2,:),FOOT(3,:),'c','LineWidth',1);
    plot3(COM(1,:),COM(2,:),COM(3,:),'g','LineWidth',1);
    xlim([-L_limit L_limit])
    ylim([-L_limit L_limit])
    zlim([0 .5])
    title(['Time =' num2str(t(ii),'%6.2f') 's'])
    view([X_(1)*180/pi+90 0])
    hold off
    
    % velocity
    subplot(3,3,6)
    hold on;grid on; box on;
    plot(t(1:ii),X(1:ii,5)*R,'linewidth',1)
    xlim([0 t(end)])
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    
    % torque
    subplot(3,3,7)
    hold on;grid on; box on;
    plot(t(1:ii),U(1:ii,1),'linewidth',1,'color','b')
    plot(t(1:ii),U(1:ii,2),'linewidth',1,'color','r')
    xlim([0 t(end)])
    xlabel('Time [s]')
    ylabel('tau [Nm]')
    
    % force
    subplot(3,3,8)
    hold on;grid on; box on;
    plot(t(1:ii),F(1:ii,1),'linewidth',1,'color','b')
    plot(t(1:ii),F(1:ii,2),'linewidth',1,'color','r')
    xlim([0 t(end)])
    xlabel('Time [s]')
    ylabel('Force [N]')
    
    set(gca,'color','white')
    pause(0.0001)
    Fra = getframe(f);
    if ii < nt
        clf
    end
    writeVideo(v,Fra)
end

close(v)




