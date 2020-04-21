function [power, avg_power] = compute_power(voltage_hip, voltage_knee, theta_dot_hip, theta_dot_knee) 

%% Compute power for hip and knee motors

R_W = 1.3; % ohms 
K_V = 0.0186; % V.s/rad 
N_H = 26.9;
N_K = 28.8;

current_hip = (voltage_hip - K_V * N_H*theta_dot_hip)/R_W;
power_hip = voltage_hip .* current_hip;

current_knee = (voltage_knee - K_V *N_K* theta_dot_knee)/R_W;
power_knee = voltage_knee .* current_knee;

power = power_hip + power_knee;
avg_power =  mean(power);


return