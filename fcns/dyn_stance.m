function [dXdt,u,F] = dyn_stance(t,X,p)

global Fz

% --- parameters ---
params = p.params;
tTD = p.tTD;            % touchdown time
ptTD = p.ptTD;          % touchdown toe position
Tst = p.Tst;

% actuator dynamics
Rboom = 0.44;
Nh = 26.9;
Nk = 28.8;
I_rotor = 7e-6;
Rw = 1.3;
kT = 0.0135;
kv = 0.0186;

% damping from back-emf
Bh = kv * kT * Nh^2 / Rw;
Bk = kv * kT * Nk^2 / Rw;
J_rotor = [zeros(2) zeros(2);
           zeros(2) diag([I_rotor*Nh^2,I_rotor*Nk^2])];
B_damp = [zeros(2) zeros(2);
          zeros(2) diag([Bh,Bk])];
      
% --- get matrices ---

[m,n] = size(X);
if n == 8
    X = X';
end


N = size(X,2);
u = zeros(N,2);
F = zeros(N,2);
dXdt = zeros(size(X,1),N);

for ii = 1:N
    q = X(1:4,ii);
    dq = X(5:8,ii);
    De = fcn_De(q,params);
    Ce = fcn_Ce(q,dq,params);
    Ge = fcn_Ge(q,params);
    Be = fcn_Be(q,params);

    % Holonomic constraints
    Jhc = fcn_Jhc(q,ptTD,params);
    dJhc = fcn_dJhc(q,dq,ptTD,params);

    %% Controller
    % Feedforward force
    s = (t(ii) - tTD) / Tst;        % stance phase parametrization s = [0, 1]
    % Force profile using Bezier polynomials
    Fz = polyval_bz([0 0 120 0 0], s);
%     Fx = 40*(0-q(1));
    Fx = polyval_bz([0 0 0 0 0], s);

    %Joint-level control
    Jc_HIP = fcn_J_toe_HIP(q,params);
    u_ = -Jc_HIP'*[Fx; Fz];    %Torques of joints 3 (hip) and 4 (knee)
    u(ii,:) = u_';
    F(ii,:) = [Fx Fz];

    % Solve the linear system:
    % De * ddq + Ce * dq + Ge = J' * GRF + Be * u (4 eqns)
    % Jhc * ddq + dJhc * dq = 0 (2 eqns)
    % [De  -Jhc'] * [ddq] = [Be*u - Ce*dq - Ge]
    % [Jhc  0   ]   [GRF]   [-dJhc*dq         ]
    % unknowns: ddq(4x1), GRF(2x1) 
    % control: u(2x1)
    tau_s = -0.0242 * q(4) + 0.0108;
    v_taus = [0;0;0;2*tau_s];

    Amat = [De+J_rotor -Jhc'; 
            Jhc zeros(2,2)];
    bvec = [Be*u_ - (Ce+B_damp)*dq - Ge + v_taus; 
            -dJhc*dq];

    ddqu = Amat \ bvec;
    ddq = ddqu(1:4);
    GRF = ddqu(5:6);
    Fz = GRF(2);

    dXdt(:,ii) = [dq; ddq];
    
end
end
