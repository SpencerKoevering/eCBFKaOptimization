function [r] = makeProbECBF_2d(input, car, car_t, l_er, l_ef, l_mr, l_mf, w_e, w_m, alpha1, alpha2, a_m, B_m, mudex, mudey, stdevdex, stdevdey, confidence, rval)
x_e = car(1);
y_e = car(2);
psi_e = car(3);
v_e = car(4);
a_e = input(1);
B_e = input(2);

x_m = car_t(1);
y_m = car_t(2);
psi_m = car_t(3);
v_m = car_t(4);
[extr, dextr, ddextr, eytr, deytr, ddeytr, extl, dextl, ddextl, eytl, deytl, ddeytl] = boundingboxderivatives(psi_e, v_e, 2*w_e, 2*l_ef, a_e, B_e);
[mxtr, dmxtr, ddmxtr, mytr, dmytr, ddmytr, mxtl, dmxtl, ddmxtl, mytl, dmytl, ddmytl] = boundingboxderivatives(psi_m, v_m, 2*w_m, 2*l_mf, a_m, B_m);
if extr > extl
    posthetae = true;
else
    posthetae = false;
end
if mxtr > mxtl
    posthetam = true;
else
    posthetam = false;
end
if posthetae
    dx = extr;
    ddx = dextr;
    dddx = ddextr;
    
    dy = eytl;
    ddy = deytl;
    dddy = ddeytl;

else
    dx = extl;
    ddx = dextl;
    dddx = ddextl;
    
    dy = eytr;
    ddy = deytr;
    dddy = ddeytr;
end
       
if posthetam
    odx = mxtr;
    oddx = dmxtr;
    odddx = ddmxtr;
    
    ody = mytl;
    oddy = dmytl;
    odddy = ddmytl;
else
    odx = mxtl;
    oddx = dmxtl;
    odddx = ddmxtl;
    
    ody = mytr;
    oddy = dmytr;
    odddy = ddmytr;
end

xedot = v_e*cos(psi_e)-B_e*v_e*sin(psi_e);
xedotdot = -((v_e^2*B_e)/l_er)*sin(psi_e)-((v_e^2*B_e^2)/l_er)*cos(psi_e)+a_e*cos(psi_e)-a_e*B_e*sin(psi_e);
xmdot = v_m*cos(psi_m)-B_m*v_m*sin(psi_m);
xmdotdot = -((v_m^2*B_m)/l_mr)*sin(psi_m)-((v_m^2*B_m^2)/l_mr)*cos(psi_m)+a_m*cos(psi_m)-a_m*B_m*sin(psi_m);

yedot = v_e*sin(psi_e)+B_e*v_e*cos(psi_e);
yedotdot = ((v_e^2*B_e)/l_er)*cos(psi_e)-((v_e^2*B_e^2)/l_er)*sin(psi_e)+a_e*sin(psi_e)+a_e*B_e*cos(psi_e);
ymdot = v_m*sin(psi_m)+B_m*v_m*cos(psi_m);
ymdotdot = ((v_m^2*B_m)/l_mr)*cos(psi_m)-((v_m^2*B_m^2)/l_mr)*sin(psi_m)+a_m*sin(psi_m)+a_m*B_m*cos(psi_m);

h = abs(x_e-x_m) - dx-odx + abs(y_e-y_m) - dy-ody-rval;
hdot = (x_e-x_m)/abs(x_e-x_m) * (xedot-xmdot) -ddx - oddx + (y_e-y_m)/abs(y_e-y_m)* (yedot-ymdot) -ddy - oddy;
hddot = (x_e-x_m)/abs(x_e-x_m) * (xedotdot-xmdotdot) -dddx - odddx + (y_e-y_m)/abs(y_e-y_m)* (yedotdot-ymdotdot) -dddy - odddy;


LHS = (-alpha1*h-hddot)/alpha2 -hdot;
r = (norminv(1-confidence) + (LHS - ((x_e-x_m)/abs(x_e-x_m) * mudex +(y_e-y_m)/abs(y_e-y_m) * mudey))/(stdevdex + stdevdey));
end