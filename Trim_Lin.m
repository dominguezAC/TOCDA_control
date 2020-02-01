% Parámetros del actuador: 
% el actuador no debería existir en la primera parte del trabajo! (fase I),
% aparece en la fase II y se mantiene en la fase III. 
wnAct=20; 
psiAct=0.8;
maxDefAct = 0;
minDefAct = -70;
rateLimAct = 50;
initialPosAct = 0; 
initialVelAct = 0;


Vw=10;
Tf=0.1;
Ke=1.3717;
Bm=0.001;
J=10;
R=10;
Ra=2.58;
La=0.028;

Rpala=1.6;
Rho=1.225;
c1=0.51763;
c2=116;
c3=0.4;
c4=5;
c5=21;
c6=0.006795;

%Trim:
X0 = [];    % initial condition on x whatever
Y0 = 50;    % initial condition on y -> 50 (watts out)
U0 = 0;     % initial condition on u -> 0

iy = 1;     % Constrain Y(1) to be fixed
ix = [];    % No constrain over x
iu = [];    % No constrain over u

[X,U,Y,DX] = trim('Aerogen2019',X0,U0,Y0,ix,iu,iy)
                                
% X(1) -> Watts
% X(2) -> 
% X(3) -> 
% X(4) -> 
%%
%Linealization
[num,den] = linmod('Aerogen2019',X,U)