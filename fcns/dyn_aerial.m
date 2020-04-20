function [dXdt,u,F] = dyn_aerial(t,X,p)


params = p.params;

[m,n] = size(X);
if n == 8
    X = X';
end

N = size(X,2);
u = zeros(N,2);
F = zeros(N,2);
dXdt = zeros(size(X,1),N);
for ii = 1:N
    q = X(1:4,ii);                 %joint positions
    dq = X(5:8,ii);                %joint velocities

    De = fcn_De(q,params);      %inertia matrix
    Ce = fcn_Ce(q,dq,params);   %Coriolis matrix
    Ge = fcn_Ge(q,params);      %gravity vector
    Be = fcn_Be(q,params);      %actuation selection matrix
    J_h2f_b = fcn_J_h2f_b(q,params);
    

    % swing controller
    qd = [pi/2.5; -pi*0.65];     % desired joint position Knee forward
%     qd = [-pi/3; pi*0.65];     % desired joint position Knee backward 

    %qd = [pi/10, -30*pi/180];
    q1 = q(3);q2 = q(4);
    dq1 = dq(3);dq2 = dq(4);

    % joint PD control during aerial phase
    u_ = [20*(qd(1)-q1) + 1*(0-dq1);...
          20*(qd(2)-q2) + 1*(0-dq2)];
   
    F_sw = J_h2f_b(2:3,3:4)' \ u_;
%     % swing controller
%     Kp = diag(800*[1 1]);
%     Kd = diag(5*[1 1]);
%     Krh = 0.1;          % constant for raibert hopper
%     vx = dq(1) * 0.44;
%     p_h2f_b_d = [Krh * vx;-0.15];   % swing foot position
%     p_h2f_b = fcn_p_h2f_b(q,params);
%     J_h2f_b = fcn_J_h2f_b(q,params);
%     v_h2f_b = J_h2f_b * dq;
% 
%     F_sw = Kp * (p_h2f_b_d - p_h2f_b(2:3)) + Kd * (-v_h2f_b(2:3));
%     u_ = J_h2f_b(2:3,3:4)' * F_sw;
    u(ii,:) = u_';
    F(ii,:) = F_sw';

    % spring effect
    tau_s = -0.0242 * q(4) + 0.0108;
    v_taus = [0;0;0;2*tau_s];

    % dynamics
    ddq = De \ (Be * u_ + v_taus - Ce*dq - Ge);

    dXdt(:,ii) = [dq;ddq];
    
    
    
end
end
