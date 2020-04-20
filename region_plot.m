tau_Imax = 0.162;
omega_NL = 645.2;
tau_stall = 0.124;
N_H = 26.9;
N_K = 28.8;

% operating reigion for hip actuator

x = -60:0.1:60;
y = -5:0.01:5;  
[omega_H,tau_H] = meshgrid(x,y);  

phi_1H = tau_H/N_H - tau_Imax <= 0 ;  
phi_2H = -tau_H/N_H - tau_Imax <= 0 ;
phi_3H = tau_H/N_H + tau_stall*N_H*omega_H/omega_NL - tau_stall <= 0;
phi_4H = -tau_H/N_H - tau_stall*N_H*omega_H/omega_NL - tau_stall <= 0;

phi_1K = tau_H/N_K - tau_Imax <= 0 ; 
phi_2K = -tau_H/N_K - tau_Imax <= 0 ;
phi_3K = tau_H/N_K + tau_stall*N_K*omega_H/omega_NL - tau_stall <= 0;
phi_4K = -tau_H/N_K - tau_stall*N_K*omega_H/omega_NL - tau_stall <= 0;

both_H = phi_1H & phi_2H & phi_3H & phi_4H ;    
both_K = phi_1K & phi_2K & phi_3K & phi_4K ;%# Intersection of both inequations
both = both_H & both_K;
figure, hold on
c = 1:3;                                   %# Contour levels

contourf(omega_H,tau_H,c(1) * both_H, [c(1), c(1)], 'red')
contourf(omega_H,tau_H,c(2) * both_K, [c(2), c(2)], 'green')  %# Fill area for second inequation
 
contourf(omega_H,tau_H,c(3) * both, [c(3), c(3)], 'blue')
colormap(white)
xlabel('\omega_H (rad/s)')
ylabel('\tau_H (N.m)')
legend('Region of Hip', 'Region of Knee','Overlap')
title('Operating region for knee actuator')




