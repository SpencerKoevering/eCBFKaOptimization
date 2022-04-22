function [c, ceq] = tECBF_det_optim(input, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt, rval, prevAlphafc, prevAlphaft, prevAlphabt)

c= [];

if any(isnan(input))
    disp("fmincon input NaN")
    if (usefc)
        c=[c zeros([6 1])];
    end
    if (useft)
        c=[c zeros([6 1])];
    end
    if (usebt)
        c=[c zeros([6 1])];
    end
    ceq = [];
    return;
end

if ~isempty(prevAlphafc)
    alphafc1=prevAlphafc(1);
    alphafc2=prevAlphafc(2);
else
    alphafc1=input(6);
    alphafc2=input(7);
end

if ~isempty(prevAlphaft)
    alphaft1=prevAlphaft(1);
    alphaft2=prevAlphaft(2);
else
    alphaft1=input(8);
    alphaft2=input(9);
end

if ~isempty(prevAlphabt)
    alphabt1=prevAlphabt(1);
    alphabt2=prevAlphabt(2);
else
    alphabt1=input(10);
    alphabt2=input(11);
end

if(usefc)
    [cbf] = makeDetECBF(input, car, fc, l_er, l_ef, l_fcr, l_fcf, alphafc1, alphafc2, afc, Bfc, true, rval);
    c=[c;-cbf; 0];
    
    if isempty(prevAlphafc)
        c = [c; makeKaConditions_det(input(1), input(2), afc, Bfc, alphafc1, alphafc2, car, fc, true, l_er, l_ef, l_fcr, l_fcf, rval)];
    else
        c=[c;0;0;0;0];
    end
end
if(useft)
    [cbf] = makeDetECBF(input, car, ft, l_er, l_ef, l_ftr, l_ftf, alphaft1, alphaft2, aft, Bft, true, rval); 
    c=[c;-cbf; 0];
    
    if isempty(prevAlphaft)
        c = [c; makeKaConditions_det(input(1), input(2), aft, Bft, alphaft1, alphaft2, car, ft, true, l_er, l_ef, l_ftr, l_ftf, rval)];
    else
        c=[c;0;0;0;0];
    end
end
if(usebt)
    [cbf] = makeDetECBF(input, car, bt, l_er, l_ef, l_btr, l_btf, alphabt1, alphabt2, abt, Bbt, false, rval);
    c=[c;-cbf; 0];
    
    if isempty(prevAlphabt)
        c = [c; makeKaConditions_det(input(1), input(2), abt, Bbt, alphabt1, alphabt2, car, bt, false, l_er, l_ef, l_btr, l_btf, rval)];
    else
        c=[c;0;0;0;0];
    end
end
ceq=[];
end