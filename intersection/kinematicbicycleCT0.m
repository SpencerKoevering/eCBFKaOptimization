function dxdt = kinematicbicycleCT0(x, u)

state=x;
x = state(1);
y = state(2);
psi = state(3);
v=state(4);
f = [v*cos(psi); v*sin(psi);0;0;];
g = [0 -v*sin(psi); 0 v*cos(psi); 0 v/l_er; 1 0;];
dxdt = f+g*u;
