function c = makeCBF(input, x, m, epsilon, gamma, a_m, B_m, acc_lim, front, l_m, l_e)
    a_e = input(1);
    B_e = input(2);
    
    x_e = x(1);
    psi_e = x(3);
    v_e = x(4);
    
    x_m = m(1);
    psi_m = m(3);
    v_m = m(4);
    if front
        if x(4)>=m(4)
            eqn1 = (a_m*(2*v_e - 2*v_m))/(2*acc_lim) - a_e*(epsilon + (2*v_e - 2*v_m)/(2*acc_lim) + 1) + v_e*sign(x_e - x_m)*cos(psi_e) - v_m*sign(x_e - x_m)*cos(psi_m) - B_e*v_e*sign(x_e - x_m)*sin(psi_e) + B_m*v_m*sign(x_e - x_m)*sin(psi_m) + gamma*(abs((x_e-x_m))-l_e-l_m - (1+epsilon)*v_e - (v_m-v_e)^2/(2*acc_lim));
        else
            eqn1 = (v_e*sign(x_e - x_m)*cos(psi_e) - a_e*(epsilon + 1) - v_m*sign(x_e - x_m)*cos(psi_m) - B_e*v_e*sign(x_e - x_m)*sin(psi_e) + B_m*v_m*sign(x_e - x_m)*sin(psi_m)- gamma*(abs(x_e - x_m) - l_m - l_e - v_e*(epsilon + 1)));
        end
    else
        if m(4) >= x(4)
            eqn1 = (v_e*sign(x_e - x_m)*cos(psi_e) - (a_e*(2*v_e - 2*v_m))/(2*acc_lim) - a_m*(epsilon - (2*v_e - 2*v_m)/(2*acc_lim) + 1) - v_m*sign(x_e - x_m)*cos(psi_m) - B_e*v_e*sign(x_e - x_m)*sin(psi_e) + B_m*v_m*sign(x_e - x_m)*sin(psi_m) + gamma*(abs(x_e - x_m) - l_m - l_e - (v_e - v_m)^2/(2*acc_lim) - v_m*(epsilon + 1)));
        else
            eqn1 = (v_e*sign(x_e - x_m)*cos(psi_e) - a_m*(epsilon + 1) - v_m*sign(x_e - x_m)*cos(psi_m) - B_e*v_e*sign(x_e - x_m)*sin(psi_e) + B_m*v_m*sign(x_e - x_m)*sin(psi_m) - gamma*(abs(x_e - x_m) - l_m - l_e - v_m*(epsilon + 1)));
        end
    end
    c= [eqn1];
end