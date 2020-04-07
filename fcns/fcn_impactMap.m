function X_post = fcn_impactMap(X_prev,p)

params = p.params;

q_prev = X_prev(1:4)';  %Joint positions before impact
dq_prev = X_prev(5:8)';  %Joint velocoties before impact

De = fcn_De(q_prev,params); %Inertia matrix
J = fcn_J_toe(q_prev,params); %Jacobian of the foot global position

% M*(qdot_post - qdot_prev) = J^T*Fimp
% J*qdot_post = 0;  
Aimp = [De -J';
        J zeros(3)];
bimp = [De*dq_prev ; zeros(3,1)];
dqF = Aimp \ bimp;

dq_post = dqF(1:4); %Joint positions after impact
F_impact = dqF(5:7); % impact impulse

%Joint positions after impact are the same as before impact, just the velocities
%are discontinuous.
X_post = [q_prev; dq_post];










