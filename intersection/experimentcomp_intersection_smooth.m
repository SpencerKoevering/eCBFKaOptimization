% numtrials = 50;
numtrials = 100;

first_order = zeros(2,numtrials); 
prob_optim = zeros(2,numtrials);
det_optim = zeros(2,numtrials);
prob_const = zeros(2,numtrials);
det_const = zeros(2,numtrials);

for scenario=[2]
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
%             cars = [o1 o2];
            cars = [o1 o2];
%             initlane=[1, -3, 1];
            afterturnlane=[];
            initlane=[1, -Inf, Inf];
%             afterturnlane=[2, -Inf, Inf];
            maxepochs=600;
%             r=1;
%             turn_index = -1;
%             Kalphapen = .25;
            distance_obj=6;
%             distance_obj=0;
            weight=125000/2;
%             aw = .01;
%             Bw = .005;
%             wx = 1;
%             wy = 1;
%             wtheta = 2;
            turn_index = -1;
            r=0;
            Kalphapen = 1;
            aw = .2;
            Bw = 1;
            wx = .3;
            wy = .3;
            wtheta = .3;
            
        elseif(scenario == 3)
            ego_epsilonx_mean = 0;
            ego_epsilonx_stdev = .1;

            other_epsilonx_mean = 0;
            other_epsilonx_stdev = .1;

            ego_epsilony_mean = 0;
            ego_epsilony_stdev = .1;

            other_epsilony_mean = 0;
            other_epsilony_stdev = .1;
            
            confidence = .9999;
            
            Ts=1;
            randval1 = .25*rand;
            randval2 = .25*rand;
            randval3 = .25*rand;
            randval4 = 10*rand-5;
            randval5 = 10*rand-5;
%             turnref = [[48 1 -pi]', [45 .5 -3*pi/4]'  [42 -100 -pi/2]'];
            turnref = [[48 1 -pi]', [42 -100 -pi/2]'];
            xbounds = [41.5, Inf, 41, 43];
%             ybounds =  [0, 2, -Inf, 2];
            ybounds =  [.5, 1.5, -Inf, 2];
            turnepoch = 1;
            start_speed = .7;
            pa = .05;
            pB = .01;
            o1 = intersection_car_smooth([73+randval4, 1, -pi, .7+randval1].', 1, 1, .5, turnref ,Ts, other_epsilonx_mean, other_epsilony_mean, other_epsilonx_stdev, other_epsilony_stdev, xbounds, ybounds, 1, .3, .5, pa, pB, turnepoch, maxa, max_angle);
            x=[10+randval5 -1 0 .7+randval3].';
            ref = [[60, -1, 0]'];
            cars = [o1];
%             initlane=[1, -1.5,.5];
            afterturnlane=[];
            initlane=[1, -Inf, Inf];
%             afterturnlane=[2, -Inf, Inf];
%             r = .35;
            weight=0;
            maxepochs = 400;
%             turn_index = -1;
%             distance_obj=0;
%             Kalphapen = .25;
%             aw = .1;
%             Bw = .05;
%             wx = .5;
%             wy = .5;
%             wtheta = .6;
            turn_index = -1;
            r=.25;
            Kalphapen = 5;
            distance_obj=0;
            aw = .3;
            Bw = 1;
            wx = .3;
            wy = .3;
            wtheta = .5;
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