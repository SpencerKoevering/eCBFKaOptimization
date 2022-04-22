function [c] = makeDetECBF(input, car, car_t,l_er, l_ef, l_mr, l_mf, alpha1, alpha2, a_m, B_m, front, rval)
x_e = car(1);
psi_e = car(3);
v_e = car(4);
a_e = input(1);
B_e = input(2);

x_m = car_t(1);
psi_m = car_t(3);
v_m = car_t(4);

if front
    extra_dist = l_ef+l_mr+rval;
else
    extra_dist = l_er+l_mf+rval;
end

h = (x_e-x_m)^2-(extra_dist)^2;

xedot = v_e*cos(psi_e)-B_e*v_e*sin(psi_e);
xedotdot = -((v_e^2*B_e)/l_er)*sin(psi_e)-((v_e^2*B_e^2)/l_er)*cos(psi_e)+a_e*cos(psi_e)-a_e*B_e*sin(psi_e);
xmdot = v_m*cos(psi_m)-B_m*v_m*sin(psi_m);
xmdotdot = -((v_m^2*B_m)/l_mr)*sin(psi_m)-((v_m^2*B_m^2)/l_mr)*cos(psi_m)+a_m*cos(psi_m)-a_m*B_m*sin(psi_m);

hdot = 2*(x_e-x_m)*(xedot-xmdot);

delta_x = x_e-x_m;
delta_dx = xedot - xmdot;
delta_ddx = xedotdot - xmdotdot;

c = 2*(delta_dx)^2 + 2*(delta_x)*(delta_ddx)+ alpha1*h + alpha2*hdot;
end