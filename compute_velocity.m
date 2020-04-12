function [avg_velocity, max_velocity] = compute_velocity(theta_dot)
%% Compute average and absolute maximum velocity of a joint
L_B = 0.5; % meters

avg_velocity = mean(L_B * theta_dot);
max_velocity = max(abs(L_B * theta_dot));



return