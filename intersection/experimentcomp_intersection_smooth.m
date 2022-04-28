numtrials = 100;

first_order = zeros(2,numtrials); 
prob_optim = zeros(2,numtrials);
det_optim = zeros(2,numtrials);
prob_const = zeros(2,numtrials);
det_const = zeros(2,numtrials);

for scenario=[1 2]
    if scenario ==1
        dirname = strjoin(["case1test_smooth_noslack" strrep(datestr(datetime), ':', '_')]);
        mkdir(dirname);
    elseif scenario==2
        dirname = strjoin(["case2test_smooth_noslack" strrep(datestr(datetime), ':', '_')]);
        mkdir(dirname);
    elseif scenario==3
        dirname = strjoin(["case3test_smooth_noslack" strrep(datestr(datetime), ':', '_')]);
        mkdir(dirname);
    elseif scenario==4
        dirname = strjoin(["case3test_smooth_noslack" strrep(datestr(datetime), ':', '_')]);
        mkdir(dirname);
    end
    for c=1:numtrials
        if(scenario==1)
            ego_epsilonx_mean = 0;
            ego_epsilonx_stdev = .15;

            other_epsilonx_mean = 0;
            other_epsilonx_stdev = .15;

            ego_epsilony_mean = 0;
            ego_epsilony_stdev = .15;

            other_epsilony_mean = 0;
            other_epsilony_stdev = .15;

            confidence = .9999;

            Ts=1;
            randval1 = .1*rand;
            randval2 = .1*rand;
            randval3 = .1*rand;
            randval4 = 10*rand-5;
            randval5 = 10*rand-5;
            ref1 = [[-100 1 pi]'];
            pa = .1;
            pB = .2;
            maxa=1;
            max_angle=pi/2;
            o1 = intersection_car_smooth([66+randval4, 1, pi, .7+randval1].', 1, 1, .5, ref1 ,Ts, other_epsilonx_mean, other_epsilony_mean, other_epsilonx_stdev, other_epsilony_stdev, [-Inf, Inf], [.5, 1.5], .3, .3, .5, pa, pB, -1, maxa, max_angle);
            x=[10+randval5 -1 0 .7+randval3].';
            ref = [[36, -1, 0]', [44, 0, pi/4]', [44, 1, pi/2]', [44, 20, pi/2]',];
            cars = [o1];
            initlane=[1, -Inf, Inf];
            afterturnlane=[];
            turn_index = -1;
            maxepochs=600;
            r=.1;
            Kalphapen = 2;
            distance_obj=0;
            weight=0;
            aw = .2;
            Bw = 1;
            wx = .3;
            wy = .3;
            wtheta = .3;
          
        elseif(scenario==2)
            ego_epsilonx_mean = 0;
            ego_epsilonx_stdev = .15;

            other_epsilonx_mean = 0;
            other_epsilonx_stdev = .15;

            ego_epsilony_mean = 0;
            ego_epsilony_stdev = .15;

            other_epsilony_mean = 0;
            other_epsilony_stdev = .15;

            confidence = .9999;
            
            Ts=1;
            randval1 = .1*rand;
            randval2 = .1*rand;
            randval3 = .1*rand;
            randval4 = (10*rand-5)/5;
            randval5 = (10*rand-5)/5;
            randval6 = (10*rand-5)/2;
            ref1 = [[44 40 pi/2]'];
            ref2 = [[42 -40 -pi/2]'];
            pa = .25;
            pB = .5;
            maxa=1;
            max_angle=pi/2;
            o1 = intersection_car_smooth([44, -20+randval4, pi/2, .7+randval1].', 1, 1, .5, ref1 ,Ts, other_epsilonx_mean, other_epsilony_mean, other_epsilonx_stdev, other_epsilony_stdev, [43, 45], [-Inf, Inf], .3, .3, .5, pa, pB, -1, maxa, max_angle);
            o2 = intersection_car_smooth([42, 20+randval5, -pi/2, .7+randval2].', 1, 1, .5, ref2 ,Ts, other_epsilonx_mean, other_epsilony_mean, other_epsilonx_stdev, other_epsilony_stdev, [41, 43], [-Inf, Inf], .3, .3, .5, pa, pB, -1, maxa, max_angle);
            x=[20+randval6 -1 0 .7+randval3].';
            ref = [[50, -1, 0]'];
            cars = [o1 o2];
            afterturnlane=[];
            initlane=[1, -Inf, Inf];
            maxepochs=600;
            distance_obj=6;
            weight=125000/2;
            turn_index = -1;
            r=0;
            Kalphapen = 1;
            aw = .2;
            Bw = 1;
            wx = .3;
            wy = .3;
            wtheta = .3;
            
        end
        exp_epoch = c;
        disp("      -det_const");
        det_const(:,c)=intersection_controller_smooth(x, cars, ref, Ts, 1, 1, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, ego_epsilony_mean, ego_epsilony_stdev, other_epsilony_mean, other_epsilony_stdev, r, confidence, exp_epoch, initlane, afterturnlane, turn_index, maxepochs, false, false, dirname, Kalphapen, aw, Bw, distance_obj, weight, wx, wy, wtheta)
        for i = 1 : length(cars)
            cars(i).reset();
        end
        disp("      -det_optim");
        det_optim(:,c)=intersection_controller_smooth(x, cars, ref, Ts, 1, 1, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, ego_epsilony_mean, ego_epsilony_stdev, other_epsilony_mean, other_epsilony_stdev, r, confidence, exp_epoch, initlane, afterturnlane, turn_index, maxepochs, false, true, dirname, Kalphapen, aw, Bw, distance_obj, weight, wx, wy, wtheta)
        for i = 1 : length(cars)
            cars(i).reset();
        end
        disp("      -prob_const");
        prob_const(:,c)=intersection_controller_smooth(x, cars, ref, Ts, 1, 1, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, ego_epsilony_mean, ego_epsilony_stdev, other_epsilony_mean, other_epsilony_stdev, r, confidence, exp_epoch, initlane, afterturnlane, turn_index, maxepochs, true, false, dirname, Kalphapen, aw, Bw, distance_obj, weight, wx, wy, wtheta)
        for i = 1 : length(cars)
            cars(i).reset();
        end
        disp("      -prob_optim");
        prob_optim(:,c)=intersection_controller_smooth(x, cars, ref, Ts, 1, 1, ego_epsilonx_mean, ego_epsilonx_stdev, other_epsilonx_mean, other_epsilonx_stdev, ego_epsilony_mean, ego_epsilony_stdev, other_epsilony_mean, other_epsilony_stdev, r, confidence, exp_epoch, initlane, afterturnlane, turn_index, maxepochs, true, true, dirname, Kalphapen, aw, Bw, distance_obj, weight, wx, wy, wtheta)
        for i = 1 : length(cars)
            cars(i).reset();
        end
    end
    cd(dirname)
    save(['experiment' num2str(c) '.mat'], 'det_const', 'det_optim', 'prob_const', 'prob_optim');
    dirname
    cd("..")
end