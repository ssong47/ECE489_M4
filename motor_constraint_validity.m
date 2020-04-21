L_H = 0.096;
L_K = 0.155;
D_K = 0.052;
L = sqrt(L_K^2+D_K^2);
mu = 0.6;
tau_Imax = 0.162;
w_NL = 645.2;
tau_stall = 0.124;
N_H = 26.9;
N_K = 28.8;

constr_tau_val = @(N,dq)N*[tau_Imax;
                           -tau_Imax;...
                           (-tau_stall*dq*N/w_NL + tau_stall);...
                           (-tau_stall*dq*N/w_NL - tau_stall)];

constraint1 = @(N, tau, dq)tau/N <= tau_Imax;
constraint2 = @(N, tau, dq)tau/N >= -tau_Imax;
constraint3 = @(N, tau, dq)tau/N <= -tau_stall*dq*N/w_NL + tau_stall;
constraint4 = @(N, tau, dq)tau/N >= -tau_stall*dq*N/w_NL - tau_stall;

constraint_valid = @(N,tau,dq)constraint1(N,tau,dq) && ...
                              constraint2(N,tau,dq) && ...
                              constraint3(N,tau,dq) && ...
                              constraint4(N,tau,dq);   

omega = [-100:0.001:100];
counter_H = 1;
counter_K = 1;
points_omegaH = zeros(1,2*length(omega));
points_tauH = zeros(1,2*length(omega));
points_omegaK = zeros(1,2*length(omega));
points_tauK = zeros(1,2*length(omega));


for i = 1:length(omega)
    
    tau_H = constr_tau_val(N_H, omega(i));
    tau_K = constr_tau_val(N_K, omega(i));
    
    for j = 1:4
        % Checking if the points are valid for hip motor
        if constraint_valid(N_H, tau_H(j),omega(i))
            points_tauH(counter_H) = tau_H(j);
            points_omegaH(counter_H) = omega(i);
            counter_H = counter_H + 1;
        end
        
        % Checking if points are valid for knee motor
        if constraint_valid(N_K, tau_K(j),omega(i))
            points_tauK(counter_K) = tau_K(j);
            points_omegaK(counter_K) = omega(i);
            counter_K = counter_K + 1;
        end
    end
    
end    

%Plot for hip actuator
subplot(1,2,1);

scatter(points_omegaH(1:counter_H-1), points_tauH(1:counter_H-1),1); hold on;
scatter(omegas(1:counter,1),Torques(1:counter,1),5); 
legend('Operating Region', 'Simulation');

title('Operating Region for Hip Actuator', 'FontSize', 15);
xlabel('\omega_H (rad/s)', 'FontSize', 15);
ylabel('Tau_H (Nm)');
xlim([-60,60]);

% Plot for knee actuator
subplot(1,2,2);
scatter(points_omegaK(1:counter_K-1), points_tauK(1:counter_K-1),1); hold on;
scatter(omegas(1:counter,2),Torques(1:counter,2),5); 
legend('Operating Region', 'Simulation');

title('Operating Region for Knee Actuator', 'FontSize', 15);
xlabel('\omega_K (rad/s)', 'FontSize', 15);
ylabel('Tau_K (Nm)');
xlim([-60,60])