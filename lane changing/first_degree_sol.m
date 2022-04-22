function [result] = first_degree_sol(x, cars, merge_point, abs_target_speed, fc, ft, bt, fci, fti, bti, isfc, isft, isbt, target_y, target_merge_y, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, r, exp_epoch)

    options =  optimset('Display','off');
    max_epochs=1000;
    H = [10 0 0 0 0; 0 1000 0 0 0; 0 0 10000 0 0; 0 0 0 .001 0; 0 0 0 0 1000000];
    F = [0;0;0;0;0;];
    
    alpha_y=.8;
    alpha_v=1.7;
    alpha_yaw=12;
    
    gamma_1=1;
    gamma_2=1;
    gamma_3=1;
    
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

    beta=0;

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

    merging = false;
    
    merge_complete=false;
    epoch_merge_completed=-1;
    collided=false;
    unable_to_compensate=false;
    c=1;
    epochs_since_merge_complete = 0;
    while epochs_since_merge_complete<=10 && c < max_epochs
%         disp(c);
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
        
        epsilon1 = rval;
        epsilon2 = rval;
        epsilon3 = rval;


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
        
        %% fc CBF
    a_fc=0;
    B_fc=0;
    a_ft=0;
    B_ft=0;
    a_bt=0;
    B_bt=0;
    if(isfc)
        if x(4) > fc.State(4)
            h_CBF2 = fc.State(1) - x(1) - l_ef-l_fcr - (1 + epsilon1) * x(4) - 0.5 * (fc.State(4) - x(4)) * (fc.State(4) - x(4)) / lim_acc;
            h2dot = fc.State(4) - (0.5 / lim_acc) * 2 * (fc.State(4) - x(4)) * a_fc;
            Lfh2 = [-cos(x(3)) * x(4)];
            Lgh2 = [(-(1 + epsilon1) + (fc.State(4) - x(4)) / lim_acc), x(4) * sin(x(3))];
            Acbf2 = [-Lgh2, 0, 0, 0];
            bcbf2 = [Lfh2 + gamma_1 * h_CBF2 + h2dot];
        else
            h_CBF2 = fc.State(1) - x(1) - l_ef-l_fcr - (1 + epsilon1) * x(4);
            h2dot = fc.State(4);
            Lfh2 = [-cos(x(3)) * x(4)];
            Lgh2 = [-(1 + epsilon1), x(4) * sin(x(3))];
            Acbf2 = [-Lgh2, 0, 0, 0];
            bcbf2 = [Lfh2 + gamma_1 * h_CBF2 + h2dot];
        end
    else
        Acbf2 = [0, 0, 0, 0, 0];
        bcbf2 = [0];
        h_CBF2 = [];
    end
    
    if(isft)
        if x(4) > ft.State(4)
            h_CBF4 = ft.State(1) - x(1) - l_ef-l_ftr - (1 + epsilon2) * x(4) - 0.5 * (ft.State(4) - x(4)) * (ft.State(4) - x(4)) / lim_acc;
            h4dot = ft.State(4) - (0.5 / lim_acc) * 2 * (ft.State(4) - x(4)) * a_ft;
            Lfh4 = [-cos(x(3)) * x(4)];
            Lgh4 = [(-(1 + epsilon2) + (ft.State(4) - x(4)) / lim_acc), x(4) * sin(x(3))];
            Acbf4 = [-Lgh4, 0, 0, 0];
            bcbf4 = [Lfh4 + gamma_2 * h_CBF4 + h4dot];
        else
            h_CBF4 = ft.State(1) - x(1) - l_ef-l_ftr - (1 + epsilon2) * x(4);
            h4dot = ft.State(4);
            Lfh4 = [-cos(x(3)) * x(4)];
            Lgh4 = [-(1 + epsilon2), x(4) * sin(x(3))];
            Acbf4 = [-Lgh4, 0, 0, 0];
            bcbf4 = [Lfh4 + gamma_2 * h_CBF4 + h4dot];
        end
    else
        Acbf4 = [0, 0, 0, 0, 0];
        bcbf4 = [0];
        h_CBF4 = [];
    end
    
    if ~isbt
        Acbf6 = [0, 0, 0, 0, 0];
        bcbf6 = [0];
        h_CBF6 = [];
    else
        if bt.State(4) > x(4)
            h_CBF6 = -(bt.State(1) + l_btf) + x(1) - l_er - (1 + epsilon3) * bt.State(4) - 0.5 * (bt.State(4) - x(4)) * (bt.State(4) - x(4)) / lim_acc;
            h6dot = -bt.State(4) - (1 + epsilon3) * a_bt - (1 / lim_acc) * (bt.State(4) - x(4)) * a_bt;
            Lfh6 = (cos(x(3)) * x(4));
            Lgh6 = [(x(4) - bt.State(4)) / lim_acc, -x(4) * sin(x(3))];
            Acbf6 = [-Lgh6, 0, 0, 0];
            bcbf6 = [Lfh6 + gamma_3 * h_CBF6 + h6dot];
        else
            h_CBF6 = -(bt.State(1) + l_btf) + x(1) - l_er - (1 + epsilon3) * bt.State(4);
            h6dot = -bt.State(4) - (1 + epsilon3) * a_bt;
            Lfh6 = cos(x(3)) * x(4);
            Lgh6 = [0, -x(4) * sin(x(3))];
            Acbf6 = [-Lgh6, 0, 0, 0];
            bcbf6 = [Lfh6 + gamma_3 * h_CBF6 + h6dot];
        end
    end

        A_u = [1, 0, 0, 0, 0; ...
            -1, 0, 0, 0, 0; ...
            0, 1, 0, 0, 0; ...
            0, -1, 0, 0, 0; ...
            cos(psi + beta), 0, 0, 0, 0; ...
            -cos(psi + beta), 0, 0, 0, 0];
        A_u0 = [0, 1, 0, 0, 0; ...
            0, -1, 0, 0, 0];
        b_u = [lim_acc; lim_acc; lim_beta; lim_beta; 0.5 * 0.9 * 9.81; 0.5 * 0.9 * 9.81];
        b_u0 = [beta + 1 * lim_slip_rate * dt; -beta + 1 * lim_slip_rate * dt];


        %%QP

        Aclf = [phi1_y, -1, 0, 0; phi1_v, 0, -1,0;phi1_yaw, 0, 0, -1;];
        bclf = [-phi0_y;-phi0_v; -phi0_yaw];

        Constraint_A_merg = [Aclf; Acbf2; Acbf4; Acbf6; A_u; A_u0];
        Constraint_b_merg = [bclf; bcbf2; bcbf4; bcbf6; b_u; b_u0];
        
        Constraint_A_pre = [Aclf; Acbf2;A_u; A_u0];
        Constraint_b_pre = [bclf; bcbf2;b_u; b_u0];
        
        Constraint_A_done = [Aclf; Acbf4; Acbf6; A_u; A_u0];
        Constraint_b_done = [bclf; bcbf4; bcbf6; b_u; b_u0];


        if~merge_complete
        [originput,fval,exitflag,output] = quadprog(H, F, Constraint_A_merg, Constraint_b_merg, [], [], [], [], [], options);
        input = originput.';
            if ~merging && c >= merge_point && (exitflag > 0 && ~isempty(input)) && isInSafeSet_deg1(input, x, fc.State, ft.State, bt.State, isfc, isft, isbt, rval, l_er, l_ef, l_fcr,l_ftr, l_btf, lim_acc, epsilon1, epsilon2, epsilon3) 
                target_y = target_merge_y;
                merging= true;
            elseif c>2 && (exitflag < 0||isempty(input)) && ~merging
%                 disp("infeasible high constraint QP, defaulting to front car only")    
            [originput,fval,exitflag,output] = quadprog(H, F, Constraint_A_pre, Constraint_b_pre, [], [], [], [], [], options);
            input = originput.';
            end
        else        
            [originput,fval,exitflag,output] = quadprog(H, F, Constraint_A_done, Constraint_b_done, [], [], [], [], [], options);
            input = originput.';
        end
        
        if abs(x(2)-target_y)<.25 && merging == true && abs(x(3)) < pi/24 
            merge_complete=true;
            epoch_merge_completed=c;
            merging=false;
        end

        if c>2 && (exitflag < 0||isempty(input))
            unable_to_compensate=true;
            break;
        end
        rval=r;


        d1 = input(3);
        d2 = input(4);
        d3 = input(5);
        input = [input(1:2).'; normrnd(ego_epsilonx_mean, ego_epsilonx_stdev);];

        x = x+ f+g*input;
        for m=1:length(cars)
            cars(m).State=cars(m).increment_probx(0,0, other_epsilonx_mean, other_epsilonx_stdev);
        end
        fc = cars(fci);
        ft = cars(fti);
        bt = cars(bti);
        if isIntersection(x, fc.State, l_ef, l_er, l_fcf, l_fcr, .5, .5)
          disp('collided with fc')
          collided=true;
          break
        end
        if isIntersection(x, ft.State, l_ef, l_er, l_ftf, l_ftr, .5, .5)
          disp('collided with ft')
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
            disp('collided with bt')
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
    name = ['1d' num2str(exp_epoch) '.mat'];
    save(name, 'history', 'xhistory', 'yhistory', 'psihistory');
    %%plotting
%     t = 1:max_epochs;
%     
%     figure();
%     plot(t, history(4,:));
%     xlabel('time'); ylabel('ego velocity');
% 
% 
%     figure();
%     plot(t, history(2,:));
%     xlabel('time'); ylabel('ego positiony');
% 
%     figure();
%     plot(t, history(3,:));
%     xlabel('time'); ylabel('ego psi');
%     
%     plotBehavior(history, xhistory, yhistory, psihistory);

end

