%% 1a,1b

%params value
tau_Imax = 0.162;
omega_NL = 645.2;
tau_stall = 0.124;
N_H = 26.9;
N_K = 28.8;

% operating reigion for hip actuator

x = -60:0.1:60;
y = -5:0.01:5;  
[omega_H,tau_H] = meshgrid(x,y);  

% constraints for hip actuator
phi_1H = tau_H/N_H - tau_Imax <= 0 ;  
phi_2H = -tau_H/N_H - tau_Imax <= 0 ;
phi_3H = tau_H/N_H + tau_stall*N_H*omega_H/omega_NL - tau_stall <= 0;
phi_4H = -tau_H/N_H - tau_stall*N_H*omega_H/omega_NL - tau_stall <= 0;

% convert to double for plotting
phi_1H = double(phi_1H);  
phi_2H = double(phi_2H); 
phi_3H = double(phi_3H); 
phi_4H = double(phi_4H); 

% set the 0s to NaN
phi_1H(phi_1H == 0) = NaN;
phi_2H(phi_2H == 0) = NaN;
phi_3H(phi_3H == 0) = NaN;
phi_4H(phi_4H == 0) = NaN;

phi_H = phi_1H.*phi_2H.*phi_3H.*phi_4H;  

% plot operating region for hip actuator
subplot(2,1,1);
surf(omega_H,tau_H,phi_H);
title('Operating region for hip actuator')
xlabel('\omega_H')
ylabel('\tau_H')
view(0,90);   % change to top view


% operating reigion for knee actuator

x = -60:0.1:60;
y = -5:0.01:5;  
[omega_K,tau_K] = meshgrid(x,y);  

% constraints for knee actuator
phi_1K = tau_K/N_K - tau_Imax <= 0 ; 
phi_2K = -tau_K/N_K - tau_Imax <= 0 ;
phi_3K = tau_K/N_K + tau_stall*N_K*omega_K/omega_NL - tau_stall <= 0;
phi_4K = -tau_K/N_K - tau_stall*N_K*omega_K/omega_NL - tau_stall <= 0;

% convert to double for plotting
phi_1K = double(phi_1K);  
phi_2K = double(phi_2K); 
phi_3K = double(phi_3K); 
phi_4K = double(phi_4K); 

% set the 0s to NaN
phi_1K(phi_1K == 0) = NaN;
phi_2K(phi_2K == 0) = NaN;
phi_3K(phi_3K == 0) = NaN;
phi_4K(phi_4K == 0) = NaN;

phi_K = phi_1K.*phi_2K.*phi_3K.*phi_4K;  

% plot operating region for knee actuator

subplot(2,1,2);
surf(omega_K,tau_K,phi_K);
title('Operating region for knee actuator')
xlabel('\omega_K')
ylabel('\tau_K')
view(0,90);   % change to top view

%% 1c
syms theta3 theta4 tau_H tau_K real

%params value
L_H = 0.096;
L_K = 0.155;
D_K = 0.052;
L = sqrt(L_K^2+D_K^2);
mu = 0.6;


J_HIP =  [L_H*cos(theta3)+L*cos(theta3+theta4) L*cos(theta3+theta4); L_H*sin(theta3)+L*sin(theta3+theta4) L*sin(theta3+theta4)];
tau = [tau_H;tau_K];

F_GRF = transpose(J_HIP)\tau; % F_GRF = (J_HIP^T)^(-1)*tau

% constraints for ground reaction force

phi_1GRF = -F_GRF(2) <= 0; %positive vertical component
phi_2GRF = F_GRF(1)+ mu*F_GRF(2) <= 0; % contraints on horizontal force due to friction
phi_3GRF = -F_GRF(1) + mu*F_GRF(2) <= 0;

%% 1e


subplot(2,1,1);
hold on
title('Operating region for hip actuator')
%plot(Xout(:,7), Uout(:,1))
plot(omegas(:,1), Torques(:,1));
xlabel('\omega_H')
ylabel('\tau_H')
view(0,90);   % change to top view

subplot(2,1,2);
hold on
title('Operating region for knee actuator')
%plot(Xout(:,8), Uout(:,2))
plot(omegas(:,2), Torques(:,2));
xlabel('\omega_H')
ylabel('\tau_H')
view(0,90);   % change to top view


figure
title("grfy/grfx")

%%

%params value
L_H = 0.096;
L_K = 0.155;
D_K = 0.052;
L = sqrt(L_K^2+D_K^2);
mu = 0.6;


GRFs = []

for i = 1:length(Uout)
    theta3 = Xout(i,3);
    theta4 = Xout(i,4);
    J_HIP = [L_H*cos(theta3)+L*cos(theta3+theta4) L*cos(theta3+theta4); L_H*sin(theta3)+L*sin(theta3+theta4) L*sin(theta3+theta4)];
    tau = Uout(i,:)';
    F_GRF = J_HIP'\tau;
    GRFs = [GRFs F_GRF];
end

% plot(GRFs(2,:),GRFs(1,:)); hold on;
% title("Slope < 0.6")
% plot(GRFs(2,:), -0.6*GRFs(2,:));
% plot(GRFs(2,:), +0.6*GRFs(2,:));

plot(Forces(:,1),Forces(:,2)); hold on;
plot(Forces(:,1), Forces(:,1)/0.6, 'LineWidth', 2);
plot(Forces(:,1), -Forces(:,1)/0.6, 'LineWidth', 2);
title("F_Z vs F_Y");
xlabel('F_Y (N)');
ylabel('F_Z (N)');
