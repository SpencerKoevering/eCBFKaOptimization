function [c, ceq] = make_cbf_conditions(input, x, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, r, acc_lim, afc, Bfc, aft, Bft, abt, Bbt, isfc, isft, isbt, gamma1, gamma2, gamma3)
    epsilon1 = 2;
    epsilon2 = 2;
    epsilon3 = 2;
    
    
    c=[];
    ceq = [];
%     return;
    
    if(isfc)
        c= [c, makeCBF(input, x, fc, epsilon1, gamma1, afc, Bfc, acc_lim, true, l_fcr, l_ef)];
    else
        c= [c, 0];
    end
    return;
    
    if(isft)
        c= [c, makeCBF(input, x, ft, epsilon2, gamma2, aft, Bft, acc_lim, true, l_ftr, l_ef)];
    else
        c= [c, 0];
    end
    
    if(isbt)
        c= [c, makeCBF(input, x, bt, epsilon3, gamma3, abt, Bbt, acc_lim, false, l_btf, l_er)];
    else
        c= [c, 0];
    end
end