function val = check_contact_constraint(tau_H, tau_K, q)
%This function checks if the contact constraints
%are satisfied, outputs 1 if they are satisfied and 0 otherwise

val = 1;

%Parameters
L_H = 0.096;
L_K = 0.155;
D_K = 0.052;
L = sqrt(L_K^2+D_K^2);
mu = 0.6;

%Initializations
tau = [tau_H;tau_K];
theta3 = q(3);
theta4 = q(4);

%Foot Jacobian w.r.t the hip
J_HIP =  [L_H*cos(theta3)+L*cos(theta3+theta4) L*cos(theta3+theta4); ...
    L_H*sin(theta3)+L*sin(theta3+theta4) L*sin(theta3+theta4)];


F_GRF = J_HIP'\tau; % F_GRF = (J_HIP^T)^(-1)*tau


%disp(F_GRF);

%Checking the contact constraints

if F_GRF(2) > 0
    val = 0;
elseif abs(F_GRF(1)) > mu*abs(F_GRF(2))
    val = 0;
end


end

