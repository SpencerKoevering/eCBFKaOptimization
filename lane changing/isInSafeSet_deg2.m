function isInSafeSet = isInSafeSet_deg2(car, fc, ft, bt, isfc, isft, isbt, rval, l_er, l_ef, l_fcr,l_ftr, l_btf )
    c = [0 0 0];
    if isfc
        c(1) = (car(1)-fc(1))^2-(l_ef+l_fcr+rval)^2;
    else
        c(1)= 1;
    end

    if isft
        c(2) = (car(1)-ft(1))^2-(l_ef+l_ftr+rval)^2;
    else
        c(2)= 1;
    end
    
    if isbt
        c(3) = (car(1)-bt(1))^2-(l_er+l_btf+rval)^2;
    else
        c(3)= 1;
    end
    
    if all(c >= 0)
        isInSafeSet = 1;
    else
        isInSafeSet = 0;
    end
end