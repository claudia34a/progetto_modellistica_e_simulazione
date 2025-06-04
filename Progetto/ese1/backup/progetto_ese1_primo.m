%% inizializzazione WorkSpace
clear all; clc

%% Inizializza variabili
g=9.81; % Acc. gravitazionale (m/s^2)
m=1.4; % Massa quadricottero (Kg)

Jx = .05; % Momento di inerzia lungo l'asse X (kg-m^2)
Jy = .05; % Momento di inerzia lungo l'asse Y (kg-m^2)
Jz = .05; % Momento di inerzia lungo l'asse Z (kg-m^2)

%valori iniziali per raggiungere l'equilibrio
xi = 0; yi = 0; zi = 0;
U1i = .5; U2i = 0; U3i = 0; U4i = 0;
pi = 0; qi = 0; ri= 0;
phii = 0; thetai = 0; psii = 5; %NB: psi può essere un valore qualsiasi

X_des_GF = .5; % Valore desiderato di X nel frame inerziale (metri)
Y_des_GF = .5; % Valore desiderato di Y nel frame inerziale (metri)
Z_des_GF = .5; % Valore desiderato di Z nel frame inerziale (metri)
psi_des = pi/6; % Valore di psi desiderato (radianti)

%% Modello
%Il sistema è descritto dalle seguenti 12 equazioni:

syms phi psi theta U1 U2 U3 U4 xb yb zb p q r
%scrivo equazioni delle accelerazioni [ddx, ddy, ddz]'
ddx = -1/m*(cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi))*U1; %x''
ddy = -1/m*(cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi))*U1; %y''
ddz = -1/m*(cos(phi)*cos(theta))*U1 + g; %z''

%scrivo le equazioni delle velocità [dx, dy, dz]'
dx = cos(psi)*cos(theta)*xb -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi)*xb + sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)*xb; %x'    
dy = sin(phi)*cos(theta)*yb + cos(phi)*cos(theta) + sin(phi)*sin(theta)*sin(psi)*yb +cos(theta)*sin(psi) + sin(phi)*sin(theta)*cos(psi)*yb; %y'    
dz = sin(theta)*zb + cos(theta)*sin(phi)*zb + cos(theta)*cos(phi)*zb; %z'
%XB, YB, ZB SONO VARIABILI DI STATO?

%scrivo le equazioni delle velocità angolari [ddphi, ddtheta, ddpsi]'
dphi= 1*p +sin(phi)*tan(theta)*p + cos(phi)*tan(theta)*p;  
dtheta= 0*q+ cos(phi)*q + -sin(phi)*q; 
dpsi= 0*r + sin(phi)*r/cos(theta) + cos(phi)*r/cos(theta);

%scrivo le equazioni delle accelerazioni angolari [ddphi, ddtheta, ddpsi]'
ddphi = 1/Jx*((Jy-Jz)*dtheta*dpsi + U2);
ddtheta = 1/Jy*((Jz-Jy)*dpsi*dphi + U3); 
ddpsi = 1/Jz*((Jx-Jy)*dtheta*dphi + U4);
    
%Variabili di stato
% X = [phi, theta, psi, dphi, dtheta, dpsi, dx, dy, dz, x, y, z]

%% studio equilibrio (solo traslazionale)
%scrivo funzione
f=[ddx; ddy; ddz];

xeq_s=solve(f==0, [U1, phi, theta]);

%xeq1_s=([xeq_s.phi;xeq_s.theta;xeq_s.psi; xeq_s.U1])
xeq1_s=([xeq_s.phi(1);xeq_s.theta(1);0; xeq_s.U1(1)]) % converto pti di equilibrio in double
xeq1=double(xeq1_s);
% per studiare la stabilità devo calcolare la jacobiana di f nel punto di equilibrio
% calcolo la jacobiana
J_s=jacobian(f,[phi,theta,psi,  U1])
% calcoliamo la jacobiana nel punto di equilibrio xeq
% sostituendo ai simboli phi, theta e psi i valori all'equilibrio
% attraverso il comando subs
Jxeq_s=subs(J_s,[phi,theta,psi,U1]',xeq1) % Jxeq a valori simbolici
Jxeq=double(Jxeq_s) % Jxeq a valori reali
aval=eig(Jxeq) % xeq ha parte reale > 0 quindi il pto di equilibrio è instabile
 
% Essendo un sistema del secondo ordine
% possiamo classificare il pto di equilibrio:
% AUTOVALORI REALI
% concordi negativi=>nodo stabile
% concordi positivi => nodo instabile
% uno nullo e uno positivo=>nodo instabile
% discordi => sella
% AUTOVALORI SONO COMPLESSI E CONIUGATI
% parte reale negativa => fuoco stabile
% parte reale positiva => fuoco instabile

%% plotto easy
SimOutEasy = sim("progetto_ese1_sim_easy.slx");
x=SimOutEasy.yout{1}.Values.x;
y=SimOutEasy.yout{1}.Values.y;
z=SimOutEasy.yout{1}.Values.z;
dataX=x.Data;
dataY=y.Data;
dataZ=z.Data;
t=x.Time;

%faccio plot
figure(1); clf
% VEDIAMO LE COORDINATE Z NEGATIVE CON U1 POSITIVO PERCHE' IL RIFERIMENTO E'
% POSITIVO VERSO IL BASSO
plot(t, dataZ)
title('Percorso 2D del Drone'); 
xlabel('tempo'); ylabel('Posizione Z'); 
grid on;
%disegno percorso in 3D
figure(2); 
hold on
plot3(dataX, dataY, dataZ) 
title('Percorso 3D del Drone'); 
xlabel('Posizione X'); ylabel('Posizione Y'); zlabel('Posizione Z'); 
grid on;

%% Lancio Simulazione (non lineare)
SimOutNL = sim('progetto_ese1_sim.slx');

% grafico 3D
figure()
plot3(SimOutNL.X_out.signals.values,SimOutNL.Y_out.signals.values,SimOutNL.Z_out.signals.values) %Traiettoria non lineare
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
title('3D')

%grafici 2D
figure()
hold on 
plot(SimOutNL.tout, SimOutNL.X_out.signals.values) %traiettoria non lineare
title('X')

figure()
hold on 
plot(SimOutNL.tout, SimOutNL.Y_out.signals.values) %traiettoria non lineare
title('Y')

figure()
hold on 
plot(SimOutNL.tout, SimOutNL.Z_out.signals.values) %traiettoria non lineare
title('Z')