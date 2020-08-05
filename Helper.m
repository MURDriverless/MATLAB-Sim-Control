e_ref = 0;
for i=1:numel(world.Car.X)
    closest = 1;
    xr(i) = world.Car.X(i)-world.Car.Lr*cos(world.Car.Theta(i));
    yr(i) = world.Car.Y(i)-world.Car.Lr*sin(world.Car.Theta(i));
    closeDist = realmax;
    for j=1:numel(X)
        xp = X(j);
        yp = Y(j);
        dist = sqrt((xr(i)-xp)^2+(yr(i)-yp)^2);
        if dist<closeDist
            closeDist = dist;
            closest = j;
        elseif dist>closeDist
            continue;
        end
    end
    MatchingPoints(:,i) = [X(closest);Y(closest)];
    Dist(i) = closeDist;
    
    e_ref = e_ref+closeDist;
end
fprintf("Calculated Reference Error %f\n",e_ref);

figure(2)
hold on
plot(xr,yr,'b')
plot(MatchingPoints(1,:),MatchingPoints(2,:),'g')
plot(X,Y,'r-')
%plot(left_x,left_y,'bo')
%plot(right_x,right_y,'yo')
%plot(pp.CPs(1,:),pp.CPs(2,:),'go')
