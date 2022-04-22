function [c] = makeKaConditions_prob_intersection(a_e, B_e, a_m, B_m, alpha_1, alpha_2, car, car_t, l_er, l_ef, l_mr, l_mf, w_e, w_m, rval, confidence, mudex, mudey, stdevdex, stdevdey)
    
    x_e = car(1);
    y_e = car(2);
    psi_e = car(3);
    v_e = car(4);

    x_m = car_t(1);
    y_m = car_t(2);
    psi_m = car_t(3);
    v_m = car_t(4);

    [extr, dextr, ddextr, eytr, deytr, ddeytr, extl, dextl, ddextl, eytl, deytl, ddeytl] = boundingboxderivatives(psi_e, v_e, 2*w_e, 2*l_er, a_e, B_e);
    [mxtr, dmxtr, ddmxtr, mytr, dmytr, ddmytr, mxtl, dmxtl, ddmxtl, mytl, dmytl, ddmytl] = boundingboxderivatives(psi_m, v_m, 2*w_m, 2*l_mr, a_m, B_m);

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
    
    if(x_e==x_m || y_e==y_m)
        disp("x_e-x_m or y_e-y_m undefined")
    end
    
    
    p1 = alpha_2/2 - ((alpha_2^2-4*alpha_1)^(1/2))/2;
    p2 = alpha_2/2 + ((alpha_2^2-4*alpha_1)^(1/2))/2;
    
    b1 = -hdot-p1*h;
    c1 = norminv(1-confidence)-(b1 - (x_e-x_m)/abs(x_e-x_m) * mudex -(y_e-y_m)/abs(y_e-y_m) * mudey)/(stdevdex + stdevdey);
    
    b2 = (-hddot-(p1+p2)*hdot-p1*p2*h)/(p1+p2);
    c2 = norminv(1-confidence)-(b2 - (x_e-x_m)/abs(x_e-x_m) * mudex -(y_e-y_m)/abs(y_e-y_m) * mudey)/(stdevdex + stdevdey);
    
    order1 = [-p1+.001, -p2+.001, -c1, -c2];
    
    
    q1 = alpha_2/2 - ((alpha_2^2-4*alpha_1)^(1/2))/2;
    q2 = alpha_2/2 + ((alpha_2^2-4*alpha_1)^(1/2))/2;
    
    d1 = -hdot-q1*h;
    r1 = norminv(1-confidence)-(d1 - (x_e-x_m)/abs(x_e-x_m) * mudex -(y_e-y_m)/abs(y_e-y_m) * mudey)/(stdevdex + stdevdey);
    
    d2 = (-hddot-(q1+q2)*hdot-q1*q2*h)/(q1+q2);
    r2 = norminv(1-confidence)-(d2 - (x_e-x_m)/abs(x_e-x_m) * mudex -(y_e-y_m)/abs(y_e-y_m) * mudey)/(stdevdex + stdevdey);
    
    order2 = [-q1+.001, -q2+.001, -r1, -r2];
    
    if all(order1<=0)
        c=order1;
    elseif all(order2<=0)
        c=order2;
    else
        c=order1;
    end
return
end