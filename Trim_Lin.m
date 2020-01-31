Vw=10;
wn=20;
psi=0.8;
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
Y0= 50;
U0=0;
iy=1;
[X,U,Y,DX] = trim('Aerogen2019',[],U0,Y0,[],[],iy)

%Linealization
[num,den] = linmod('Aerogen2019',X,U)