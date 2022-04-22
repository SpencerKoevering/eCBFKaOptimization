function intersecting = isIntersection(car1, car2, l_1f, l_1r, l_2f, l_2r, w1, w2)

    theta1 = car1(3);
    theta2 = car2(3);
    car1points = [ [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)]*[l_1f; w1]+[car1(1);car1(2)], [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)]*[l_1f; -w1]+[car1(1);car1(2)], [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)]*[-l_1r; w1]+[car1(1);car1(2)], [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)]*[-l_1r; -w1]+[car1(1);car1(2)]];
    car2points = [ [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)]*[l_2f; w2]+[car2(1);car2(2)], [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)]*[l_2f; -w2]+[car2(1);car2(2)], [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)]*[-l_2r; w2]+[car2(1);car2(2)], [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)]*[-l_2r; -w2]+[car2(1);car2(2)]];
    
    ax11 = [car1points(1,2)-car1points(1,1); car1points(2,2)-car1points(2,1)];
    ax12 = [car1points(1,3)-car1points(1,1); car1points(2,3)-car1points(2,1)];
    
    ax21 = [car2points(1,2)-car2points(1,1); car2points(2,2)-car2points(2,1)];
    ax22 = [car2points(1,3)-car2points(1,1); car2points(2,3)-car2points(2,1)];
    
    axes = [ax11 ax12 ax21 ax22];
    
    intersecting=true;
    for c=1:length(axes)
        projpts1 = zeros(1,4);
        projpts2 = zeros(1,4);
        for n=1:length(car1points(1,:))
            a = car1points(:,n);
            b = axes(:,c); 
            projpts1(n) = dot(a,b)/norm(b);
        end
        for n=1:length(car2points(1,:))
            a = car2points(:,n);
            b = axes(:,c); 
            projpts2(n) = dot(a,b)/norm(b);
        end
        separateaxis=true;
        for m=1:length(projpts1)
            if projpts1(m) <= max(projpts2) && projpts1(m) >= min(projpts2) 
                separateaxis=false;
            end
        end
        for m=1:length(projpts2)
            if projpts2(m) <= max(projpts1) && projpts2(m) >= min(projpts1) 
                separateaxis=false;
            end
        end
        if separateaxis
            intersecting = false;
        end
    end
end