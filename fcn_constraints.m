function [tau_H, tau_K] = fcn_constraints(q,dq, tau)
%This function takes as input the robot states and torques and 
%outputs values that satisfy the constraints
t

%Initializations
dq3 = dq(3);
dq4 = dq(4);

tau_H = tau(1);
tau_K = tau(2);

theta3 = q(3); theta4 = q(4);

%Checking for motor operating regions
valid = 1;

if check_motor_constraint(tau_H, tau_K, dq3, dq4) == 0
    valid = 0;
end

%Checking Contact Constraints

if check_contact_constraint(tau_H, tau_K, q) == 0
    valid = 0;
end

%Algorithm in case the constraints are not satisfied

%Parameters
L_H = 0.096;
L_K = 0.155;
D_K = 0.052;
L = sqrt(L_K^2+D_K^2);
mu = 0.6;


if valid == 0
    
    %Defining the hip jacobian
    J_HIP =  [L_H*cos(theta3)+L*cos(theta3+theta4) L*cos(theta3+theta4); ...
    L_H*sin(theta3)+L*sin(theta3+theta4) L*sin(theta3+theta4)];
    
    %Determining ground forces desired by the current torque
    F_GRF = J_HIP'\tau;
    
    if F_GRF(2) > 0
        F_GRF(2) = 0;
    end
    if abs(F_GRF(1)) > mu*abs(F_GRF(2))
        F_GRF(1) = -sign(F_GRF(1))*(mu-0.05)*F_GRF(2);
    end
    
    F_GRF_new = zeros(2,1);
    
    for i = 0:1:100
           
        F_GRF_new = (100-i)/100*F_GRF;
        
        %disp('Force');
        %disp(F_GRF_new);
        
        tau_new = J_HIP'\F_GRF_new;
        tau_H_new = tau_new(1); tau_K_new = tau_new(2);
        
        %disp('Torque:');
        %disp(tau_new);
        
        if check_motor_constraint(tau_H_new, tau_K_new, dq3, dq4) == 1 && ...
                check_contact_constraint(tau_H_new, tau_K_new, q) == 1
            
            tau_H = tau_H_new;
            tau_K = tau_K_new;
            
            break;
        end
    end

end

