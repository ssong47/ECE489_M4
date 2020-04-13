function [modified_tau] = fcn_constraints(X,tau_H, tau_K)
%FCN_CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

tau_Imax = 0.162;
omega_NL = 645.2;
tau_stall = 0.124;
N_H = 26.9;
N_K = 28.8;

omega_H = X(7);
omega_K = X(8);

% constraints for hip actuator
phi_1H = tau_H/N_H - tau_Imax <= 0 ;  
phi_2H = -tau_H/N_H - tau_Imax <= 0 ;
phi_3H = tau_H/N_H + tau_stall*N_H*omega_H/omega_NL - tau_stall <= 0;
phi_4H = -tau_H/N_H - tau_stall*N_H*omega_H/omega_NL - tau_stall <= 0;

% constraints for knee actuator
phi_1K = tau_K/N_K - tau_Imax <= 0 ; 
phi_2K = -tau_K/N_K - tau_Imax <= 0 ;
phi_3K = tau_K/N_K + tau_stall*N_K*omega_K/omega_NL - tau_stall <= 0;
phi_4K = -tau_K/N_K - tau_stall*N_K*omega_K/omega_NL - tau_stall <= 0;

if phi_1H && phi_2H && phi_3H && phi_4H
    tau_H = tau_H;
    tau_K = tau_K;
else
    
    tau_H_original  = tau_H;
    if ~phi_1H
        tau_H = tau_Imax * N_H;
    end
    
    if ~phi_2H
        tau_H = -tau_Imax*N_H;
    end
    
    if ~phi_3H
        tau_H = (tau_stall - tau_stall*N_H*omega_H/omega_NL)*N_H;
    end
    
    if ~phi_4H
        tau_H = (tau_stall + tau_stall*N_H*omega_H/omega_NL)*N_H;
    end
    
    % scale K according to H
    tau_K = tau_H/ tau_H_original*tau_K;
end

if phi_1K && phi_2K && phi_3K && phi_4K
    tau_H = tau_H;
    tau_K = tau_K;
else
    
    tau_K_original  = tau_K;
    if ~phi_1K
        tau_K = tau_Imax * N_K;
    end
    
    if ~phi_2K
        tau_K = -tau_Imax*N_K;
    end
    
    if ~phi_3K
        tau_K = (tau_stall - tau_stall*N_K*omega_K/omega_NL)*N_K;
    end
    
    if ~phi_4K
        tau_K = (tau_stall + tau_stall*N_K*omega_K/omega_NL)*N_K;
    end
    
    % scale H according to K
    tau_H = tau_K/ tau_K_original*tau_H;
end


% theta3 = X(3);
% theta4 = X(4);
% 
% %params value
% L_H = 0.096;
% L_K = 0.155;
% D_K = 0.052;
% L = sqrt(L_K^2+D_K^2);
% mu = 0.6;
% 
% 
% J_HIP =  [L_H*cos(theta3)+L*cos(theta3+theta4) L*cos(theta3+theta4); L_H*sin(theta3)+L*sin(theta3+theta4) L*sin(theta3+theta4)];
% tau = [tau_H;tau_K];
% 
% F_GRF = transpose(J_HIP)\tau; % F_GRF = (J_HIP^T)^(-1)*tau
% 
% % constraints for ground reaction force
% 
% phi_1GRF = -F_GRF(2) <= 0; %positive vertical component
% phi_2GRF = F_GRF(1)- mu*F_GRF(2) <= 0; % contraints on horizontal force due to friction
% phi_3GRF = -F_GRF(1) - mu*F_GRF(2) <= 0;
% 
% if phi_1GRF && phi_2GRF && phi_3GRF
%     tau_H = tau_H;
%     tau_K = tau_K;
% else
%     if -F_GRF(2) <= 0
%         F_GRF(1) = 0;
%         F_GRF(2) = 0;
%     end
%     
%     if F_GRF(1) > mu*F_GRF(2)
%         F_GRF(1) = mu*F_GRF(2);
%     end
%     
%     if F_GRF(1) < -mu*F_GRF(2)
%         F_GRF(1) = -mu*F_GRF(2);
%     end
%     
%     modified_tau = transpose(J_HIP)*F_GRF;
%     tau_H = modified_tau(1);
%     tau_K = modified_tau(2);
% end


modified_tau = [tau_H tau_K];

end

