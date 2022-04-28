function xk1 = kinematicbicyclefcn(xk, uk, Ts)

M = 10;
delta = Ts/M;
xk1 = xk;
for ct=1:M
    xk1 = xk1 + delta*kinematicbicycleCT0(xk1,uk);
end
