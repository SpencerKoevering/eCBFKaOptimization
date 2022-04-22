numtrials = 100;

first_order = zeros(2,numtrials); 
prob_optim = zeros(2,numtrials);
det_optim = zeros(2,numtrials);
prob_const = zeros(2,numtrials);
det_const = zeros(2,numtrials);

for c=1:numtrials
    c
    
    confidence = .99;
    v_abs = 2.*rand();
    v_bt = (.1).*rand()+v_abs;
    v_e = (.1).*rand()+v_abs;
    fc = car([(10-5).*rand() + 10, -1, 0, v_e+(.1).*rand();].', 1, 1);
    ft = car([(10-5).*rand() + 10, 1, 0, v_bt+(.1).*rand();].', 1, 1);
    bt = car([(-10+5).*rand() - 10, 1, 0, v_bt;].', 1, 1);
    x=[0 -1 0 v_e].';
    
    abs_target_speed=(2).*rand()+.25;
    merge_point=(50).*rand()+10;

%     fc = car([15, -1, 0, 1.2].', 1, 1);
%     ft = car([10, 1, 0, 1.1].', 1, 1);
%     bt = car([-10, 1, 0, 1].', 1, 1);
%     x=[0 -1 0 1].';
%     abs_target_speed=2;
%     merge_point=10;

    cars = [fc ft bt];
    fci=1;
    fti=2;
    bti=3;
    isfc=true;
    isft=true;
    isbt=true;
    
    target_y=-1;
    target_merge_y=1;
    
    ego_epsilonx_mean = 0;
    ego_epsilonx_stdev = .0001;
    
    other_epsilonx_mean = 0;
    other_epsilonx_stdev = .0001;
    rval=1;
    
    disp("      -first_order");
    first_order(:,c)=first_degree_sol(x, cars, merge_point, abs_target_speed, fc, ft, bt, fci, fti, bti, isfc, isft, isbt, target_y, target_merge_y, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, rval, c);
    disp("      -det_const");
    det_const(:,c)=exponential_fmincon_det_const(x, cars, merge_point, abs_target_speed, fc, ft, bt, fci, fti, bti, isfc, isft, isbt, target_y, target_merge_y, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, rval, confidence, c);
    disp("      -det_optim");
    det_optim(:,c) = exponential_fmincon_det_optim(x, cars, merge_point, abs_target_speed, fc, ft, bt, fci, fti, bti, isfc, isft, isbt, target_y, target_merge_y, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, rval, confidence, c);
    disp("      -prob_const");
    prob_const(:,c) = exponential_fmincon_Prob_const(x, cars, merge_point, abs_target_speed, fc, ft, bt, fci, fti, bti, isfc, isft, isbt, target_y, target_merge_y, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, rval, confidence, c);
    disp("      -prob_optim");
    prob_optim(:,c)=exponential_fmincon_Prob_optim(x, cars, merge_point, abs_target_speed, fc, ft, bt, fci, fti, bti, isfc, isft, isbt, target_y, target_merge_y, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, rval, confidence, c);
end