function [originput,fval,exitflag,output] = doFminconwithKaChoice_intersection_smooth(objective_fun, options, car, inita, initB, cars, l_er, l_ef, a, B, prevAlphas, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, ego_epsilony_mean, ego_epsilony_stdev, other_epsilony_mean, other_epsilony_stdev, maxacc, maxB, confidence, rval, optim)  
    exitflag=0;
    slack_factor = 1;
    lb = [-slack_factor*maxacc, -slack_factor*maxB, repmat([.001 .001], 1,length(cars))];
    ub = [slack_factor*maxacc, slack_factor*maxB, repmat([Inf Inf], 1,length(cars))];
    if optim
        n=1;
        pass_alphas = zeros([length(cars), 2]);
        while exitflag ~= 1 && exitflag ~=2 && n <= length(cars)
            nonlcon=@(input) tECBF_prob_optim_intersection_smooth(input, car, cars, l_er, l_ef, a, B, pass_alphas, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, ego_epsilony_mean, ego_epsilony_stdev, other_epsilony_mean, other_epsilony_stdev, confidence, rval);
            [originput,fval,exitflag,output] = fmincon(objective_fun,[[inita initB], repmat([2 10], 1,length(cars))],[] ,[],[],[],lb,ub, nonlcon, options);
            pass_alphas(n,:) = prevAlphas(n,:);
            n=n+1;
        end
        if exitflag == 1 || exitflag == 2
            return 
        end
    end
    nonlcon=@(input) tECBF_prob_optim_intersection_smooth(input, car, cars, l_er, l_ef, a, B, prevAlphas, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, ego_epsilony_mean, ego_epsilony_stdev, other_epsilony_mean, other_epsilony_stdev, confidence, rval);
    [originput,fval,exitflag,output] = fmincon(objective_fun,[[inita initB], repmat([2 10], 1,length(cars))],[],[],[],[],lb,ub, nonlcon, options);
end