function [c] = makeKaConditions_det(a_e, B_e, a_m, B_m, alpha_1, alpha_2, car, car_t, front, l_er, l_ef, l_mr, l_mf, r, confidence, mean_diff, std_diff)
    
    x_e = car(1);
    psi_e = car(3);
    v_e = car(4);

    x_m = car_t(1);
    psi_m = car_t(3);
    v_m = car_t(4);

    if front
        extra_dist = l_ef+l_mr+r;
    else
        extra_dist = l_er+l_mf+r;
    end

    h = (x_e-x_m)^2-(extra_dist)^2;

    xedot = v_e*cos(psi_e)-B_e*v_e*sin(psi_e);
    xedotdot = -((v_e^2*B_e)/l_er)*sin(psi_e)-((v_e^2*B_e^2)/l_er)*cos(psi_e)+a_e*cos(psi_e)-a_e*B_e*sin(psi_e);
    xmdot = v_m*cos(psi_m)-B_m*v_m*sin(psi_m);
    xmdotdot = -((v_m^2*B_m)/l_mr)*sin(psi_m)-((v_m^2*B_m^2)/l_mr)*cos(psi_m)+a_m*cos(psi_m)-a_m*B_m*sin(psi_m);

    hdot = 2*(x_e-x_m)*(xedot-xmdot);
    hdotdot = 2*(xedot-xmdot)^2+2*(x_e-x_m)*(xedotdot-xmdotdot);
    
    x_d = x_e-x_m;
    xd_d = xedot-xmdot;
    xdd_d = xedotdot - xmdotdot;

        p1 = alpha_2/2 - ((alpha_2^2-4*alpha_1)^(1/2))/2;
        p2 = alpha_2/2 + ((alpha_2^2-4*alpha_1)^(1/2))/2;
        v1 = hdot + p1*h;
        v2 =  hdotdot + p1*hdot + p2*(v1);

        set1 = [-p1+.0000001; -p2+.0000001; -v1; -v2;];

        q2 = alpha_2/2 - ((alpha_2-4*alpha_1)^(1/2))/2;
        q1 = alpha_2/2 + ((alpha_2-4*alpha_1)^(1/2))/2;
        w1 = hdot + q1*h;
        w2 =  hdotdot + q1*hdot + q2*(w1);
        
        set2 = [-q1+.0000001; -q2+.0000001; -w1; -w2;];

        if all(isreal(set1)) && ~all(isreal(set2))
            c=set1;
        elseif all(isreal(set2)) && ~all(isreal(set1))
            c=set2;
        elseif all(set1<=0)
            c=set1;
        else
            c=set2;
        end
        
    return
end