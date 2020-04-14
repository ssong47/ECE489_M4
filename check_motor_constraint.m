function val = check_motor_constraint(tau_H, tau_K, dq3 , dq4)
%This function checks if the motor and knee motor constraints
%are satisfied, outputs 1 if they are satisfied and 0 otherwise

%Parameters
tau_Imax = 0.162;
omega_NL = 645.2;
tau_stall = 0.124;
N_H = 26.9;
N_K = 28.8;

val = 1;

%Constraints as inline functions

constraint1 = @(N, tau, dq)tau/N <= tau_Imax;
constraint2 = @(N, tau, dq)tau/N >= -tau_Imax;
constraint3 = @(N, tau, dq)tau/N <= -tau_stall*dq*N/omega_NL + tau_stall;
constraint4 = @(N, tau, dq)tau/N >= -tau_stall*dq*N/omega_NL - tau_stall;

%Checking Hip Constraints

if constraint1(N_H, tau_H, dq3) == 0 || constraint2(N_H, tau_H, dq3) == 0 ...
   || constraint3(N_H, tau_H, dq3) == 0 || constraint4(N_H, tau_H, dq3) == 0
    val = 0;
end

%Checking Knee Constraints

if constraint1(N_K, tau_K, dq4) == 0 || constraint2(N_K, tau_K, dq4) == 0 ...
   || constraint3(N_K, tau_K, dq4) == 0 || constraint4(N_K, tau_K, dq4) == 0
    val = 0;
end

