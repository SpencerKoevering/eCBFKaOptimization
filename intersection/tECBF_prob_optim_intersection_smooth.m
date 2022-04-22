function [c, ceq] = tECBF_prob_optim_intersection_smooth(input, car, cars, l_er, l_ef, a, B, prevAlphas, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, ego_epsilony_mean, ego_epsilony_stdev, other_epsilony_mean, other_epsilony_stdev, confidence, rval)

c= [];

mudex = ego_epsilonx_mean-other_epsilonx_mean;
stdevdex = ego_epsilonx_stdev + other_epsilonx_stdev;

mudey = ego_epsilony_mean-other_epsilony_mean;
stdevdey = ego_epsilony_stdev + other_epsilony_stdev;
for n = 1:length(cars)
    if prevAlphas(n,1) ~= 0 && prevAlphas(n,2) ~= 0
        alpha1=prevAlphas(n,1);
        alpha2=prevAlphas(n,2);
    else
        alpha1=input(3+2*(n-1));
        alpha2=input(4+2*(n-1));
    end
    
    carstate = cars(n).State;
    fcoutput = makeProbECBF_2d(input, car, carstate, l_er, l_ef, cars(n).l_r, cars(n).l_f, .5, cars(n).w, alpha1, alpha2, a(n), B(n), mudex, mudey, stdevdex, stdevdey, confidence, rval);
    c=[c;fcoutput];
    
    if prevAlphas(n,1) == 0 && prevAlphas(n,2) == 0 
        c = [c; makeKaConditions_prob_intersection(input(1), input(2), a(n), B(n), alpha1, alpha2, car, carstate, l_er, l_ef, cars(n).l_r, cars(n).l_f, .5, cars(n).w, rval, confidence, mudex, mudey, stdevdex, stdevdey)'];
    else
        c = [c;0;0];
    end
end

ceq=[];
end