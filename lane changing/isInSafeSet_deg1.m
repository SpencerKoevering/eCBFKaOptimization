function isInSafeSet = isInSafeSet_deg1(input, car, fc, ft, bt, isfc, isft, isbt, r, l_er, l_ef, l_fcr,l_ftr, l_btf, acc_lim, epsilon1, epsilon2, epsilon3)
    c = [0 0 0];
    xe = car(1);
    psie = car(3);
    ve = car(4);
    ae = input(1);
    Be = input(2);
    xedot = ve*cos(psie)-Be*ve*sin(psie);
    vedot = ae;
    
    if isfc
        if car(4)>=fc(4)
            c(1) = (xe-fc(1))-(1+epsilon1)*ve-((fc(4)-ve)^2)/(2*acc_lim) >= 0;
        else
            c(1) = (xe-fc(1))-(1+epsilon1)*ve >= 0;
        end
    else
        c(1)= 1;
    end

    if isft
        if car(4)>=ft(4)
            c(2) = (xe-ft(1))-(1+epsilon2)*ve-((ft(4)-ve)^2)/(2*acc_lim) >= 0;
        else
            c(2) = (xe-ft(1))-(1+epsilon2)*ve >= 0;
        end
    else
        c(2)= 1;
    end
    
    if isbt
        if car(4)>=ft(4)
            c(2) = (xe-bt(1))-(1+epsilon3)*bt(4)-((bt(4)-ve)^2)/(2*acc_lim) >= 0;
        else
            c(2) = (xe-bt(1))-(1+epsilon3)*bt(4) >= 0;
        end
    else
        c(2)= 1;
    end
    
    if all(c >= 0)
        isInSafeSet = 1;
    else
        isInSafeSet = 0;
    end
end