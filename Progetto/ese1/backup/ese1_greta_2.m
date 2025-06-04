clear all
close all
clc

% modello matematico non lineare della dinamica del volo di un
% quadricottero
%   VARIABILI DI STATO: [phi theta psi x y z dphi dtheta dpsi dx dy dz]
%   INGRESSI: [U1 U2 U3 U4]
%   USCITE: [phi theta psi x y z] % U [dphi dtheta dpsi dx dy dz] se si vogliono vedere anche le velocità del drone


syms p q r dxb dyb dzb phi theta psi x y z dphi dtheta dpsi dx dy dz U1 U2 U3 U4 Ixx Iyy Izz m g

% Definizione delle equazioni del sistema
ff = [ p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
       cos(phi)*q - sin(phi)*r; 
       sin(phi)*q/cos(theta) + cos(phi)*r/cos(theta);
       dxb*cos(theta)*cos(psi) + dyb*(-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi)) + dzb*(sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi));
       dxb*sin(psi)*cos(theta) + dyb*(cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi)) + dzb*(-cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi));
       -dxb*sin(theta) + dyb*cos(theta)*sin(phi) + dzb*cos(theta)*cos(psi);
       (U2 + (Iyy - Izz)*dtheta*dpsi)/Ixx;
       (U3 + (Izz - Ixx)*dphi*dpsi)/Iyy;
       (U4 + (Ixx - Iyy)*dtheta*dphi)/Izz;
       -1/m * (cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi)) * U1;
       -1/m * (cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)) * U1;
       -1/m * (cos(phi)*cos(theta)) * U1 + g ];

% CRITICITA'
% punto di equilibrio quando le velocità sono uguali a 0 e in qualsiasi
% punto nello spazio (x,y,z), e con qualsiasi angolo psi (angolo asse z)

% la forza prodotta dai 4 motori dovrà uguagliare la forza peso del drone: U1=m*g
% dall'equazione: -1/m * (cos(phi)*cos(theta)) * U1 + g, 
% anche (psi) può assumere un qualsiasi valore

% psi può assumere un qualsiasi valore 
equilibrium_point = [0,0,0,0,0,0, 0,0,x,y,z,0,0,0,0,0,0, 0.5, 0.5, 0.1, 1.4,9.81];
xx=solve(subs(ff, [p q r dxb dyb dzb phi theta x y z dphi dtheta dpsi dx dy dz Ixx Iyy Izz m g], equilibrium_point)==0);
    % U1: 6867/500 = m*g
    % U2: 0
    % U3: 0
    % U4: 0
    % psi: qualsiasi

% punto di equilibrio (non fissiamo le variabili Ixx Iyy Izz m g, situazione per un qualsiasi quadricottero)
equilibrium_point = [0,0,0,0,0,0,0,0,psi,x,y,z,0,0,0,0,0,0]; % 18 variabili


ff_eq=subs(ff,[p, q, r, dxb, dyb, dzb, phi, theta, psi, x, y, z, dphi, dtheta, dpsi, dx, dy, dz],equilibrium_point);
% Risolvere l'equazione per U1
U1_equation = ff_eq(12); % Equazione che include U1
U1_value = solve(U1_equation == 0, [U1]);

U2_value = solve(subs(ff_eq(7),[phi,theta, psi, dphi,dpsi,dtheta],[0,0,0,0,0,0]) == 0, [U2], 'ReturnConditions', true);
U3_value = solve(subs(ff_eq(8),[phi,theta, psi, dphi,dpsi,dtheta],[0,0,0,0,0,0]) == 0, [U3], 'ReturnConditions', true);
U4_value = solve(subs(ff_eq(9),[phi,theta, psi, dphi,dpsi,dtheta],[0,0,0,0,0,0]) == 0, [U4], 'ReturnConditions', true);

% Visualizzare i punti di equilibrio
disp('Valore di U1 nel punto di equilibrio:');
disp(U1_value);


% studio punto di equilibrio con jacobiana
JJ=jacobian(ff, [phi theta psi x y z dphi dtheta dpsi dx dy dz])
J_eq = subs(JJ, [p, q, r, dxb, dyb, dzb, phi, theta, psi, x, y, z, dphi, dtheta, dpsi, dx, dy, dz], equilibrium_point);
aval=eig(J_eq)
% aval = 
%       0
%       0
%       0
%       0
%       0
%       0
%       0
%       0
%       0
%       0
%       0
%       0

%% LINEARIZZAZIONE

% Linearizzazione intorno al punto di equilibrio noto (phi, theta, psi, p, q, r, dxb, dyb, dzb, x,y = 0)

% Linearizzazione intorno al punto di equilibrio noto con assunzioni
% angoli che rappresentano l'assetto del drone assumono valori molto piccoli -> 0
% Assumiamo piccoli angoli: sin(phi) ≈ phi, cos(phi) ≈ 1
ff_linearized = subs(ff, [sin(phi), cos(phi), sin(theta), cos(theta), sin(psi), cos(psi)], [phi, 1, theta, 1, psi, 1]);

% Jacobiana del sistema
JJ_lin=jacobian(ff_linearized, [phi theta psi x y z dphi dtheta dpsi dx dy dz])

% Valutare la Jacobiana nel punto di equilibrio
J_eq_lin = subs(JJ_lin, [p, q, r, dxb, dyb, dzb, phi, theta, psi, x, y, z, dphi, dtheta, dpsi, dx, dy, dz], equilibrium_point);
aval_lin=eig(J_eq_lin)
% aval_lin = 
%       0
%       0
%       0
%       0
%       0
%       0
%       0
%       0
%       0
%       0
%       0
%       0

%% fissato ingressi trovo i punti di equilibrio
U1_value = g * m;
U2_value = 0;
U3_value = 0;
U4_value = 0;

ff_substituted = subs(ff_linearized, [U1, U2, U3, U4], [U1_value, U2_value, U3_value, U4_value]);

% Risolvere il sistema per le altre variabili nel punto di equilibrio
vars = [p, q, r, dxb, dyb, dzb, phi, theta, psi, x, y, z, dphi, dtheta, dpsi, dx, dy, dz];
equilibrium_solutions = solve(ff_substituted == 0, vars, 'IgnoreAnalyticConstraints', true);

% Visualizzare i punti di equilibrio
disp('Punti di equilibrio dati gli ingressi:');
disp(equilibrium_solutions);

%Visualizzare ogni soluzione
% Numero di soluzioni
num_solutions = length(sol_p);

for i = 1:num_solutions
    fprintf('Soluzione %d:\n', i);
    pt=[equilibrium_solutions.p(i), equilibrium_solutions.q(i), equilibrium_solutions.r(i), equilibrium_solutions.dxb(i), equilibrium_solutions.dyb(i), equilibrium_solutions.dzb(i), equilibrium_solutions.phi(i), equilibrium_solutions.theta(i), equilibrium_solutions.psi(i), equilibrium_solutions.x(i), equilibrium_solutions.y(i), equilibrium_solutions.z(i), equilibrium_solutions.dphi(i), equilibrium_solutions.dtheta(i), equilibrium_solutions.dpsi(i), equilibrium_solutions.dx(i), equilibrium_solutions.dy(i), equilibrium_solutions.dz(i)]
    %disp(X)
    fprintf('\n');
end
% 
% U1_value = g * m;
% U2_value = 0;
% U3_value = 0;
% U4_value = 0;
%
% Soluzione 1:
% pt = [0, 0, 0, 0, 0, 0, 1i, -1, -1i, 0, 0, 0, 0, 0, 0, 0, 0, 0]
% 
% Soluzione 2:
% pt = [0, 0, 0, 0, 0, 0, -1i, 1, -1i, 0, 0, 0, 0, 0, 0, 0, 0, 0]
% 
% Soluzione 3:
% pt = [0, 0, 0, 0, 0, 0, -1i, -1, 1i, 0, 0, 0, 0, 0, 0, 0, 0, 0]
% 
% Soluzione 4:
% pt = [0, 0, 0, 0, 0, 0, 1i, 1, 1i, 0, 0, 0, 0, 0, 0, 0, 0, 0]
% 
% Soluzione 5:
% pt = [0, 0, 0, 0, 0, 0, 0, 0, -1i, 0, 0, 0, 0, 0, 0, 0, 0, 0]
% 
% Soluzione 6:
% pt = [0, 0, 0, 0, 0, 0, 0, 0, 1i, 0, 0, 0, 0, 0, 0, 0, 0, 0]
% 
% Soluzione 7: 
% pt = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]