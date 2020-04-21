function cost_transport = compute_cost_transport(theta_1,avg_power)

% Computes the cost of transportation. 

M = 1.336595; % kg
g = 9.81; % m/s
L_B = 0.5; % meter
avg_theta_1 = mean(theta_1); %rad/s

cost_transport = abs(avg_power/(g * M * L_B * avg_theta_1));
return