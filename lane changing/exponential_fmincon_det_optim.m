function [result] = exponential_fmincon_det_optim(x, cars, merge_point, abs_target_speed, fc, ft, bt, fci, fti, bti, isfc, isft, isbt, target_y, target_merge_y, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, r, confidence, exp_epoch)
    
    options =  optimset('Display','off');
    max_epochs=1000;
    H = [10 0 0 0 0 0 0 0 0 0 0; 0 1000 0 0 0 0 0 0 0 0 0; 0 0 10000 0 0 0 0 0 0 0 0; 0 0 0 .001 0 0 0 0 0 0 0; 0 0 0 0 1000000 0 0 0 0 0 0; 0 0 0 0 0 .0001 0 0 0 0 0;0 0 0 0 0 0 .0001 0 0 0 0;0 0 0 0 0 0 0 .0001 0 0 0;0 0 0 0 0 0 0 0 .0001 0 0;0 0 0 0 0 0 0 0 0 .0001 0;0 0 0 0 0 0 0 0 0 0 .0001;];

    alpha_y=1;
    alpha_v=50;
    alpha_yaw=1;

    l_er=1;
    l_ef=1;
    l_fcr=1;
    l_fcf=1;
    l_ftr=1;
    l_ftf=1;
    l_btr=1;
    l_btf=1;

    lim_slip_angle = 15 * pi / 180;
    lim_beta = lim_slip_angle;
    lim_acc = 0.3 * 9.81;
    lim_slip_rate = 15 * pi / 180;
    dt=1;
    rval=r;

    %%QP setup

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

    historyinput = zeros([3 max_epochs]);
    historyd1 = zeros([1 max_epochs]);
    historyd2 = zeros([1 max_epochs]);
    historyd3 = zeros([1 max_epochs]);
    historyKa = zeros([2*length(cars) max_epochs]);

    merging = false;
    beta = 0;
    
    prev_alphafc = [];
    prev_alphaft = [];
    prev_alphabt = [];


    merge_complete=false;
    epoch_merge_completed=-1;
    collided=false;
    unable_to_compensate=false;
    c=1;
    epochs_since_merge_complete = 0;

    while epochs_since_merge_complete<=10 && c < max_epochs
%         disp(c);
%         xpos = x(1);
        y = x(2);
        psi = x(3);
        v = x(4);

        f = [v*cos(psi); v*sin(psi);0;0;];
        g = [0 -v*sin(psi) 1; 0 v*cos(psi) 0; 0 v/l_er 0; 1 0 0;];
        target_speed=abs_target_speed;
        if ~merging && ~merge_complete && isfc
            target_speed = min([fc.State(4)*1.25, target_speed]);
        elseif ~merging && merge_complete && isft
            target_speed = min([ft.State(4)*1.25, target_speed]);
        elseif merging
            if isfc
                target_speed = min([fc.State(4)*1.25, target_speed]);
            end
            if isft
                target_speed = min([target_speed, ft.State(4)*1.25]);
            end
            if isbt
                target_speed = max([target_speed, bt.State(4)*.8]);
            end
        end


        %% lateral position CLF
        h_y = y - target_y;
        V_y = h_y^2;
        phi0_y = 2 * h_y * (v * sin(psi)) + alpha_y * V_y;
        phi1_y = [0, 2 * h_y * v * cos(psi)];

        %% velocity CLF
        h_v = v - target_speed;
        V_v = h_v^2;
        phi0_v = alpha_v * V_v;
        phi1_v = [2 * h_v * 1, 0];

        %% yaw angle CLF
        h_yaw = psi;
        V_yaw = h_yaw^2;
        phi0_yaw = alpha_yaw * V_yaw;
        phi1_yaw = [0, 2 * h_yaw * v * l_er];


        A_u = [1, 0, 0, 0, 0,0,0,0,0,0,0; ...
            -1, 0, 0, 0, 0,0,0,0,0,0,0; ...
            0, 1, 0, 0, 0,0,0,0,0,0,0; ...
            0, -1, 0, 0, 0,0,0,0,0,0,0; ...
            cos(psi + beta), 0, 0, 0, 0,0,0,0,0,0,0; ...
            -cos(psi + beta), 0, 0, 0, 0,0,0,0,0,0,0];
        A_u0 = [0, 1, 0, 0, 0,0,0,0,0,0,0; ...
            0, -1, 0, 0, 0,0,0,0,0,0,0];
        b_u = [lim_acc; lim_acc; lim_beta; lim_beta; 0.5 * 0.9 * 9.81; 0.5 * 0.9 * 9.81];
        b_u0 = [beta + 1 * lim_slip_rate * dt; -beta + 1 * lim_slip_rate * dt];


        %%QP

        Aclf = [phi1_y, -1, 0, 0,0,0,0,0,0,0; phi1_v, 0, -1,0,0,0,0,0,0, 0;phi1_yaw, 0, 0, -1,0,0,0,0,0,0;];
        bclf = [-phi0_y;-phi0_v; -phi0_yaw];

        Constraint_A = [Aclf; A_u; A_u0];
        Constraint_b = [bclf; b_u; b_u0];


        objective_fun=@(x) x*H*x.';
        if~merge_complete
            [originput,fval,exitflag,output] = doFminconwithKaChoice_det(objective_fun,Constraint_A,Constraint_b, options, x, fc.State, ft.State, bt.State, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, 0, 0, 0, 0, 0, 0, isfc, isft, isbt, rval, prev_alphafc, prev_alphaft, prev_alphabt);
            input = originput.';
            if exitflag == 1 || 2
                if isfc
                    prev_alphafc = input(6:7);
                end
                if isft
                    prev_alphaft = input(8:9);
                end
                if isbt
                    prev_alphabt = input(10:11);
                end
            end
            if ~merging && (exitflag == 1|| exitflag == 2) && c >= merge_point && isInSafeSet_deg2(x, fc.State, ft.State, bt.State, isfc, isft, isbt, rval, 1, 1, 1, 1, 1) 
                target_y = target_merge_y;
                merging= true;
            elseif c>2 && exitflag == -2 && ~merging
                [originput,fval,exitflag,output] = doFminconwithKaChoice_det(objective_fun,Constraint_A,Constraint_b, options, x, fc.State, ft.State, bt.State, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, 0, 0, 0, 0, 0, 0, isfc, false, false, rval, prev_alphafc, prev_alphaft, prev_alphabt);
                input = originput.';
                if exitflag == 1 || 2
                    if isfc
                        prev_alphafc = input(6:7);
                    end
                    prev_alphaft = [];
                    prev_alphabt = [];
                end
            end
        else[originput,fval,exitflag,output] = doFminconwithKaChoice_det(objective_fun,Constraint_A,Constraint_b, options, x,ft.State, [], bt.State, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, 0, 0, 0, 0, 0, 0, isft, false, isbt, rval, prev_alphafc, prev_alphaft, prev_alphabt);
            input = originput.';

            if exitflag == 1 || 2
                if isft
                    prev_alphaft = input(6:7);
                end
                if isbt
                    prev_alphabt = input(10:11);
                end
                prev_alphafc = [];
            end
        end
        if abs(x(2)-target_y)<.25 && merging == true && abs(x(3)) < pi/24 
            merge_complete=true;
            epoch_merge_completed=c;
            merging=false;
        end

        if c>2 && exitflag == -2
                unable_to_compensate=true;
                break;
        end
        rval=r;


        d1 = input(3);
        d2 = input(4);
        d3 = input(5);
        input = [input(1:2); normrnd(ego_epsilonx_mean, ego_epsilonx_stdev);];

        x = x+ f+g*input;
        for m=1:length(cars)
            cars(m).State=cars(m).increment_probx(0,0, other_epsilonx_mean, other_epsilonx_stdev);
        end
        fc = cars(fci);
        ft = cars(fti);
        bt = cars(bti);
        if isIntersection(x, fc.State, l_ef, l_er, l_fcf, l_fcr, .5, .5)
          collided=true;
          break
        end
        if isIntersection(x, ft.State, l_ef, l_er, l_ftf, l_ftr, .5, .5)
          collided=true;
          break
        elseif isft && ft.State(1)-x(1) < 0
            bti=fti;
            isbt=true;
            isft=false;
            for n= 1:length(cars)
                if cars(n).State(1) > x(1) && (cars(n).State(1) < cars(fti).State(1) || cars(fti).State(1)<x(1)) && abs(cars(n).State(2)-target_merge_y) <= .4
                    fti = n;
                    isft=true;
                end
            end
        end
        if isIntersection(x, ft.State, l_ef, l_er, l_btf, l_btr, .5, .5)
            collided=true;
            break
        elseif isbt && (x(1)-bt.State(1) < 0)
            fti=bti;
            isft=true;
            isbt=false;
            for n= 1:length(cars)
                if cars(n).State(1) < x(1) && (cars(n).State(1) > cars(bti).State(1) || cars(bti).State(1)>x(1)) && abs(cars(n).State(2)-target_merge_y) <= .4
                    bti = n;
                    isbt=true;
                end
            end
        end
        if isfc
            fc = cars(fci);
        end
        if isft
            ft = cars(fti);
        end
        if isbt
            bt = cars(bti);
        end

        history(:,c) = x;


        for m=1:length(cars)
            xhistory(m, c) = cars(m).State(1);
            yhistory(m, c) = cars(m).State(2);
            psihistory(m, c) = cars(m).State(3);
        end


        historyinput(:,c) = input;
        historyd1(c) = d1;
        historyd2(c) = d2;
        historyd3(c) = d3;
        historyKa(:,c) = originput(6:11);
        beta=input(2);
        c=c+1;
        if(merge_complete)
            epochs_since_merge_complete = epochs_since_merge_complete+1;
        end
    end
    if merge_complete && ~(unable_to_compensate || collided)
        success =1;
    elseif ~merge_complete && ~(unable_to_compensate || collided)
        success = 0;
    elseif unable_to_compensate
        success=-1;
    elseif collided
        success=-2;
    end
    
    result = [success;epoch_merge_completed;];
    
    name = ['det_optim' num2str(exp_epoch) '.mat'];
    save(name, 'history', 'xhistory', 'yhistory', 'psihistory');
end

