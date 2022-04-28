function plotBehavior(ego, carsx, carsy, carspsi, name, speed, intersection)
        if ~isempty(carsx)
            scenario = drivingScenario('SampleTime', .1, 'StopTime', 1);

            angs1 = [-30:2500]';
            angs2 = [-30:2500]';

            roadCenters = [angs1, zeros([1 2531]).', zeros([1 2531]).'];
            lspec = lanespec(2,'Width',[2 2]);
            road(scenario, roadCenters, 'lanes', lspec);
            
            if intersection
                roadCenters2 = [(43+zeros([1 2531])).', angs2, zeros([1 2531]).'];
                lspec2 = lanespec(2,'Width',[2 2]);
                road(scenario, roadCenters2, 'lanes', lspec2);
            end
            
            egoCar = vehicle(scenario,'ClassID',1,'Position',[ego(1,1) ego(2,1) 0],'Yaw',ego(3,1)*180/pi, 'Length',2, 'Width',1);
            cars = [];
            xhistsize = find(carsx(1,:)==0, 1, 'first');
            numcars = size(carsx);
            for m=1:numcars
                cars = [cars vehicle(scenario,'ClassID',1,'Position',[carsx(m,1) carsy(m, 1) 0],'Yaw',carspsi(m, 1)*180/pi, 'Length',2, 'Width',1)];
            end
            plot(scenario);
            xlim([-50+egoCar.Position(1) 50+egoCar.Position(1)]);
            ylim([-20+egoCar.Position(2) 20+egoCar.Position(2)]);

            pause(.01);

            myVideo = VideoWriter(name);
            myVideo.FrameRate = 10;
            open(myVideo);

            for c=1:xhistsize

                for m=1:length(cars)
                    cars(m).Position = [carsx(m,c) carsy(m,c) 0];
                    cars(m).Yaw = carspsi(m,c)*180/pi;
                end
                egoCar.Position = [ego(1, c) ego(2, c) 0];
                egoCar.Yaw = ego(3, c)*180/pi;
                updatePlots(scenario);
                xlim([-50+egoCar.Position(1) 50+egoCar.Position(1)]);
                ylim([-50+egoCar.Position(2) 50+egoCar.Position(2)]);
                pause(.05 * 1/speed);
                frame = getframe(gcf);
                writeVideo(myVideo, frame);
            end
            close(myVideo)
        else
            scenario = drivingScenario('SampleTime', .1, 'StopTime', 1);

            angs1 = [-30:2500]';
            angs2 = [-30:2500]';

            roadCenters = [angs1, zeros([1 2531]).', zeros([1 2531]).'];
            lspec = lanespec(2,'Width',[2 2]);
            road(scenario, roadCenters, 'lanes', lspec);

            roadCenters2 = [(43+zeros([1 2531])).', angs2, zeros([1 2531]).'];
            lspec2 = lanespec(2,'Width',[2 2]);
            road(scenario, roadCenters2, 'lanes', lspec2);

            egoCar = vehicle(scenario,'ClassID',1,'Position',[ego(1,1) ego(2,1) 0],'Yaw',ego(3,1)*180/pi, 'Length',2, 'Width',1);
            plot(scenario);
            xlim([-50+egoCar.Position(1) 50+egoCar.Position(1)]);
            ylim([-20+egoCar.Position(2) 20+egoCar.Position(2)]);

            pause(.01);

            myVideo = VideoWriter(name);
            myVideo.FrameRate = 10;
            open(myVideo);

            for c=1:length(ego)

                egoCar.Position = [ego(1, c) ego(2, c) 0];
                egoCar.Yaw = ego(3, c)*180/pi;
                updatePlots(scenario);
                xlim([-50+egoCar.Position(1) 50+egoCar.Position(1)]);
                ylim([-20+egoCar.Position(2) 20+egoCar.Position(2)]);
                pause(.05 * 1/speed);
                frame = getframe(gcf);
                writeVideo(myVideo, frame);
            end
            close(myVideo)
        end
    return;
end
