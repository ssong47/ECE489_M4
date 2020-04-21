function plot_speed(time, theta_dot)

% Plot speed (l*theta_dot) vs time 
L_B = 0.5; % meters
velocity = L_B * theta_dot;

line_width = 1.5;
figure(1)
plot(time, velocity,'Linewidth',line_width); 
xlabel('Time(sec)');
ylabel('Velocity (m/s)');


return

