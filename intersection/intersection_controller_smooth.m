function [result] = intersection_controller_smooth(x, cars, ref, Ts, l_er, l_ef, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, ego_epsilony_mean, ego_epsilony_stdev, other_epsilony_mean, other_epsilony_stdev, r, confidence, exp_epoch, initlane, afterturnlane, turn_index, max_epochs, prob, optim, dirname, Kalphapen, aw, Bw, distance_obj, weight, wx, wy, wtheta)
    nx = 4;
    ny = 3;
    nu = 2;
    nlobj = nlmpc(nx, ny, nu);
    nlobj.Ts = Ts;
    nlobj.PredictionHorizon = 10;
    nlobj.ControlHorizon = 2;
    nlobj.Model.IsContinuousTime = false;
    nlobj.Model.OutputFcn = @(x, u, Ts) [x(1); x(2); x(3)];
    x0 = x;
    u0 = [0;0];
    nlobj.Model.NumberOfParameters = 1;
    nlobj.Model.StateFcn="kinematicbicyclefcn";
    nlobj.Optimization.ReplaceStandardCost = false;
    
    % validateFcns(nlobj,x0,u0, [], {Ts});
    
    maxacc=1;
    max_angle=pi/4;
    maxB=atan(1/2*tan(max_angle)); %max steering angle is enforced through B. this value corresponds to pi/2 max
    
    nlobj.MV(1).Min = -maxacc;      %
    nlobj.MV(1).Max = maxacc;       %
    nlobj.MV(2).Min = -maxB;   % 
    nlobj.MV(2).Max = maxB;    %
    nlobj.States(4).Min = -1;      
    nlobj.States(4).Max = 1;
    if initlane(1)==1
        nlobj.States(1).Min = initlane(2);      
        nlobj.States(1).Max = initlane(3);
    elseif initlane(1)==2
        nlobj.States(2).Min = initlane(2);      
        nlobj.States(2).Max = initlane(3);
    end
        
    nlobj.Weights.ManipulatedVariablesRate = [aw Bw];
    nlobj.Weights.OutputVariables = [wx wy wtheta];

    nloptions = nlmpcmoveopt;
    nloptions.Parameters = {Ts};




%     options =  optimset('Display','iter');
    options =  optimset('Display','off');
    dt_dyn=.2;
    mv=u0;
    ref_index = 1;
    prevAlphas = zeros([length(cars), 2]);
    alphahistory = zeros([length(cars)*2, max_epochs]);

    diagonal = [(1+zeros([2*length(cars), 1])).'];
    H = diag(diagonal);
    if ~prob
        confidence = .5;
    end

    history = zeros([4 max_epochs]);

    history(:,1) = x;
    xhistory = zeros([length(cars) max_epochs]);
    for m=1:length(cars)
        xhistory(m, 1) = cars(m).State(1);
    end
    yhistory = zeros([length(cars) max_epochs]);
    for m=1:length(cars)
        yhistory(m, 1) = cars(m).State(2);
    end
    psihistory = zeros([length(cars) max_epochs]);
    for m=1:length(cars)
        psihistory(m, 1) = cars(m).State(3);
    end

    collided=false;
    unable_to_compensate=false;
    retrying=false;
    finished=false;
    c=1;
    initialr = r;
    opt_input_hist = zeros([2 max_epochs]);
    inputhistory = zeros([2 max_epochs]);
    otherinputhistory = zeros([length(cars)*2, max_epochs]);
    while ref_index <= length(ref) && c < max_epochs
        if (~retrying)
            othera = [];
            otherB = [];
            for m=1:length(cars)
                [a, B]=cars(m).nlmpc_inc_prob_get_input();
                othera(m) = a;
                otherB(m) = B; 
                otherinputhistory(2*m-1, c) = a;
                otherinputhistory(2*m, c) = B;
            end
            history(:,c) = x;
            for m=1:length(cars)
                xhistory(m, c) = cars(m).State(1);
                yhistory(m, c) = cars(m).State(2);
                psihistory(m, c) = cars(m).State(3);
            end
        end
        % Compute optimal control moves
        yref = ref(:,ref_index)';
        nextx = zeros(1,length(cars));
        nexty = zeros(1,length(cars));
        for m=1:length(cars)
            psi = cars(m).State(3);
            v = cars(m).State(4);
            f = [v*cos(psi); v*sin(psi);0;0;];
            predicted_state = cars(m).State+dt_dyn*(f);
            nextx(m) = predicted_state(1);
            nexty(m) = predicted_state(2);
        end
        nlobj.Optimization.CustomCostFcn = @(X,U,e,data,params)  sum(vectorstep(diag(realsqrt(diag(nextx-X(2,1))^2 + diag(nexty-X(2,2))^2)), distance_obj, weight));
        [mv,nloptions] = nlmpcmove(nlobj,x,mv,yref,[],nloptions);
        % Implement first optimal control move
        psi = x(3);
        v = x(4);
        f = [v*cos(psi); v*sin(psi);0;0;];
        g = [0 -v*sin(psi) 1 0; 0 v*cos(psi) 0 1; 0 v/l_er 0 0; 1 0 0 0;];
        opt_input_hist(:,c) = mv(1:2);
        objective_fun=@(x) (x(1)-mv(1))^2*stepfun(x(1), v, 1, 20)+10*(x(2)-mv(2))^2 + Kalphapen*(x(3:length(x))*H*x(3:length(x))');
        [originput,fval,exitflag,output] = doFminconwithKaChoice_intersection_smooth(objective_fun, options, x, mv(1), mv(2), cars, l_er, l_ef, othera, otherB, prevAlphas, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, ego_epsilony_mean, ego_epsilony_stdev, other_epsilony_mean, other_epsilony_stdev, maxacc, maxB, confidence, r, optim);
        input = originput.';
        
        if (exitflag == 1 || exitflag == 2) && (optim || c==1)
            for n=1:length(cars)
                prevAlphas(n, 1) = input(3+2*(n-1));
                prevAlphas(n, 2) = input(4+2*(n-1));
                alphahistory(2*n-1, c) = input(3+2*(n-1));
                alphahistory(2*n, c) = input(3+2*(n-1));
            end
            r=initialr;
        end
        if exitflag == -2 || exitflag == -1
            if  r < .01
                unable_to_compensate = true;
                break;
            else
                r = max(0, r-.1);
                retrying=true;
                continue
            end
        else
            retrying=false;
        end
        for m=1:length(cars)
            cars(m).State =cars(m).nlmpc_inc_prob_apply_input(dt_dyn);
        end
        if collided || unable_to_compensate
            break;
        end
        inputhistory(:,c) = input(1:2);
        input = [input(1:2); normrnd(ego_epsilonx_mean, ego_epsilonx_stdev);normrnd(ego_epsilony_mean, ego_epsilony_stdev);];
        for n = 1 : length(cars)
            if isIntersection(x, cars(n).State, l_ef, l_er, cars(n).l_r, cars(n).l_f, cars(n).w, cars(n).w)
                collided=true;
                break;
            end
        end
        x = x+dt_dyn*(f+g*input);
        V = x(1:2)-yref(1:2)';
        if(sqrt(V*V') < sqrt(8))
            dims = size(ref);
            if ref_index == dims(2)
                finished=true;
                break;
            else
                ref_index = ref_index+1;
                if ref_index==turn_index
                    if afterturnlane(1)==1
                        nlobj.States(1).Min = afterturnlane(2);      
                        nlobj.States(1).Max = afterturnlane(3);
                    elseif afterturnlane(1)==2
                        nlobj.States(2).Min = afterturnlane(2);      
                        nlobj.States(2).Max = afterturnlane(3);
                    end
                end
            end
        end
%         c
        c=c+1;
    end

    if unable_to_compensate
        success=-1;
    elseif collided
        success=-2;
    elseif finished==true
        success=2;
    else
        success=1;
    end

    result = [success;c;];
    if prob && optim
        name = ['prob_optim_intersection' num2str(exp_epoch) '.mat'];
    elseif prob && ~optim
        name = ['prob_const_intersection' num2str(exp_epoch) '.mat'];
    elseif ~prob && optim
        name = ['det_optim_intersection' num2str(exp_epoch) '.mat'];
    else
        name = ['det_const_intersection' num2str(exp_epoch) '.mat'];
    end
    cd(dirname)
    save(name, 'history', 'xhistory', 'yhistory', 'psihistory', 'alphahistory', 'cars', 'opt_input_hist', 'inputhistory', 'otherinputhistory');
    cd("..")
end

function ret = stepfun(a, v, biaslow, biashigh)
    if ((a+v < 0) && (a < 0))
        ret = sigmoidf(-1*v)*biashigh;
    else
        ret = biaslow;
    end
end

function ret = sigmoidf(x)
    ret = 1/(1+exp(-x));
end

function ret = vectorstep(x, det_dist, weight)
    for n=1:length(x)
        if x(n) > det_dist
            x=zeros(size(x));
            break;
        else
            x(n) = -weight*x(n) + det_dist*weight;
        end
    end
%     for n=1:length(x)
%             x(n) = -weight*x(n);
%     end
    ret = x;
end
