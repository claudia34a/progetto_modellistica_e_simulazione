% progetto_ese3
clear all
%% punto 1
%dx1/dt=3(x1^2+x1)*x2
%dx2/dt=-4x2+x1-x2*u+3*u

syms x1 x2 u
f=[3*(x1.^2+x1).*x2; -4.*x2+x1-x2.*u+3.*u];
ueq=0;

xeq_s=solve(subs(f,u,ueq)==0);%ci permette di calcolare f calcolata per u=0
%il fatto che sia che x1 e x2 hanno 2 componenti se runniamo ci fa capire
%che ci sono 2 punti di equilibrio

%ora si trasformano in double 
xeq1=double([xeq_s.x1(1) xeq_s.x2(1)]);
xeq2=double([xeq_s.x1(2) xeq_s.x2(2)]);

J_s=jacobian(f,[x1,x2]);%calcolo jacobiano

J1=double(subs(J_s,[x1,x2,u],[xeq1,0]));
aval1=eig(J1);%calcolo autovalori primo punto
if(all(real(aval1)<0))
    fprintf('Equilibrio in (%d,%d) stabile\n',xeq1)
else
    fprintf('Equilibrio in (%d,%d) instabile\n',xeq1)
end
%punto di indecidibilità

J2=double(subs(J_s,[x1,x2,u],[xeq2,0]));
aval2=eig(J2);%calcolo autovalori secondo punto
if(all(real(aval2)<0))
    fprintf('Equilibrio in (%d,%d) stabile\n',xeq2)
else
    fprintf('Equilibrio in (%d,%d) instabile\n',xeq2)
end
%punto di sella

%I punti di equilibrio sono xeq1=(0,0) e xeq2=(-1,-0.25)
%La jacobiana in xeq1 ha autovalori -4 e 0: Avendo un autovalore reale 
% negativo ed uno nullo, non possiamo stabilire con certezza se il punto 
% di equilibrio sia stabile o no (punto di indecidibilità).
%Per stabilirlo facciamo una simulazione con simulink che, ponendo come
%condizioni iniziali (-0.1,0.1), ci permette di dire che il punto di
%equilibrio è effettivamente stabile, convergendo a 0

%la jacobiana in (-1,-0.25) ha autovalori in -4 e 0.75, quindi abbiamo
%almeno un autovalore a parte reale non negativa o nulla, pertanto il punto
%di equilibrio è un nodo instabile, in particolare è un punto di sella


%% punto 2
g1=5*x2;
g2=2*x1;
%ora valutiamo quale delle due uscite possa essere utilizzata per la
%linearizzazione IO del sistema

%Caso y1=5*x2
F=f;
%y1 non dipende direttamente da u, quindi non ha grado relativo 0
%ora calcolo la derivata con lo jacobiano:
dy1=jacobian(g1,[x1,x2])*F;
%che vale: 15*u + 5*x1 - 20*x2 - 5*u*x2
%esso dipende da u, perciò il grado relativo sarà = 1
%Per questo motivo escludiamo questa uscita per la linearizzazione IO in
%quanto non ha grado relativo massimo, utilizzandola si otterrebbero delle
%parti non raggiungibili.

%Caso y2=2*x1
%y2 non dipende direttamente da u, quindi non ha grado relativo 0
%ora calcolo la derivata con lo jacobiano:
dy2=jacobian(g2,[x1,x2])*F;
%che vale: 2*x2*(3*x1^2 + 3*x1)
%esso non dipende da u, quindi non ha grado relativo 1
ddy2=jacobian(dy2,[x1,x2])*F
%che vale: (6*x1^2 + 6*x1)*(3*u + x1 - 4*x2 - u*x2) + 2*x2^2*(6*x1 + 3)*(3*x1^2 + 3*x1)
%esso dipende da u, perciò possiamo utilizzare y2 per la linearizzazione IO
%avendo grado relativo 2, cioè grado relativo massimo. I-O non causa
% generazione di parti non osservabili o non raggiungibili

%% parte 3
%y=2*x1
g=g2;
%il sistema linearizzato attorno al punto di equilibrio è un sistema in
%deltax, deltau, dove deltax=x-xeq1 e deltau=u-ueq.
%Ha in generale la forma:
%d(deltax)/dt=A*deltax+B*deltau
%deltay=C*deltaX+deltau

%dove:
%A=df/dx (cioè derivo tutte le componenti di f rispetto tutte le componenti
%di x, cioè lo JACOBIANO)
%B=df/du
%C=dg/dx
%D=dg/du

A1=J1;
B_s=jacobian(f,u)
B1=double(subs(B_s,[x1,x2,u],[xeq1,ueq]))
C_s=jacobian(g,[x1 x2]);
C1=double(subs(C_s,[x1,x2,u],[xeq1,ueq]))
D_s=jacobian(g,u);
D1=double(subs(D_s,[x1,x2,u],[xeq1,ueq]))

%procedo alla linearizzazione 

%la forma del sistema per cui possiamo utilizzare la linearizzazione i-o è
%dx/dt=f(x)+g(x)*u
%y=h(x)

%nota: f non è quella di partenza (che ora è F), g non è quella di partenza
%(che ora è G), ma è diventata h
%in pratica: F=f+g*u

%per simulink
syms v
u_lin=solve(ddy2==v,u);
%u_lin = ((6*x1^2 + 6*x1)*(x1 - 4*x2) - v + 2*x2^2*(6*x1 + 3)*(3*x1^2 + 3*x1))/((6*x1^2 + 6*x1)*(x2 - 3))

%% punto 4
%Determinare controllo in retroazione per la regolazione a 0 dello stato
%che permetta di avere dinamica definita dalla coppia di autovalori
%[a1,2*a1], doev a1 deve permettere al sistema (considerando la
%linearizzazione "perfetta") di raggiungere l'equilibrio in un tempo t=2s
%IO LINEARIZATION   
T=2;
%sistema visto dal controllore a seguito della linearizzazione
% che nello spazio degli stati è Y/V=1/s^2
A=[0 1; 0 0];
B=[0;1];
C=[1 0];

% K lo calcolo con il comando place per posizionamento autovalori 
% Quanto vale a1? (che deve garantire tempo di raggiungimento della
% stabilità T=2sec) il che significa che la costante a tempo dominante sarà
% 2/5 e quindi il polo dominante a1 = -2,5

tau_dom=T/5; % Teq = 5*tau_dominante
pd=-1/tau_dom; %polo dominante
a1=pd;%a1 deve garantire tempo di raggiungimento della stabilità in t=2s
% il che significa che la costante a tempo dominante sarà
% 2/5 e quindi il polo dominante a1 = -2,5
K=place(A,B,[a1, 2*a1]); %K lo calcolo con il comando place, per posizionamento autovalori

%NB: è nello stato z diverso da x, quindi il controllo sarà del 
% tipo v=-Kz + F
% Devo determinare il diffeomorfismo che lega z e x -> z=T(x)
F=[3*(x1^2+x1)*x2;-4*x2+x1]; % La prendo da f togliendo tutto ciò che dipende da u
h=g;
Tlin=[h;jacobian(h,[x1 x2])*F];

%K =  [12.5000    7.5000]
%Tlin = [2*x1, 2*x2*(3*x1^2 + 3*x1)]

%punto B
%Linearizzo per trovare Alin Blin Clin

Alin=J1; % Jacobiana di f rispetto a [x1,x2], calcolata in xeq1
Blin=double(subs(jacobian(f,u),[x1,x2,u],[xeq1,ueq]));
Clin1=double(subs(jacobian(g1,[x1,x2]),[x1,x2,u],[xeq1,ueq]));
Clin2=double(subs(jacobian(g2,[x1,x2]),[x1,x2,u],[xeq1,ueq]));

Rr=rank(ctrb(Alin,Blin))
% il rango è 1, per cui non posso controllare il sistema con una retroazione dello stato
% non calcolo K

%Farò il simulink del sistema non controllato