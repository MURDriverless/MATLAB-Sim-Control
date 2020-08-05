classdef PathPlanner < handle
    properties
        LCs                            %Cones on Left-Side of Track
        RCs                            %Cones on Right-Side of Track
        TKCs                           %Cones for Time-Keeping
        CPs                            %'Centre Points' Geometrically Determined from Track Cones
        Path                           %Extrapolated Path from CPs
        ConesToAddL                    %Array of Cones (Unsorted,Unadded or Not-Ordered Correctly)
        ConesToAddR                    %Array of Cones (Unsorted,Unadded or Not-Ordered Correctly)
        ConesTALSorted = true;         %Boolean - Check if Cones To Add L (Already sorted)
        ConesTARSorted = true;         %Boolean - Check if Cones To Add R (Already sorted)
        HaveSetFinalPoints = false     %Boolean - Checks if final path points determined 
        HaveLeftStartZone = false      %Boolean - Checks if left starting region
        HaveReachedEndZone = false     %Boolean - Checks if reached the end zone of track (Near start position)
        FPs                            %Final Points (CPs that are used both at start and finish of circuit)
        VPs                            %Velocity Points - Estimated Velocities at each CP
        RPs                            %Radius Points - Estimated Radiuses at each CP
        CentreSpline                   %Piecewise Cubic Spline connecting CPs - Uses an imaginary path parameter 
        VelocitySpline                 %Piecewise Cubic Spline connecting VPs - Uses same imaginary path parameter
        LC_Index = 1;                  %Track previously mapped cones on Left
        RC_Index = 1;                  %Track previously mapped cones on Right
        setConstantVelocity            %Boolean - Sets Velocity Reference Type
        VMax                           %Maximum Velocity Value
        VConst                         %Constant Velocity Value
        FGain                          %Maximum Force Gain
    end
    methods
        function obj = PathPlanner(Cones,Car,isConstantVelocity,VMax,VConst,FGain)
            obj.LCs = Cone.empty;
            obj.RCs = Cone.empty;
            obj.TKCs = Cone.empty;
            obj.FPs = double.empty(2,0);
            obj.ConesToAddL = Cone.empty;
            obj.ConesToAddR = Cone.empty;
            obj.setConstantVelocity = isConstantVelocity;
            obj.VMax = VMax;
            obj.VConst = VConst;
            obj.FGain = FGain;
            
            numL = 0;
            numR = 0;
            %Prepare initial cones to be added to path planner (Can be
            %replaced and done in first iteration of update aswell)
            for i=1:numel(Cones)
                if Cones(i).C=='b'
                    numL=numL+1;
                    obj.ConesToAddL(numL)=Cone(Cones(i).XAvg,Cones(i).YAvg,'b');
                    obj.ConesTALSorted = false;
                end
                if Cones(i).C=='y'
                    numR=numR+1;
                    obj.ConesToAddR(numR)=Cone(Cones(i).XAvg,Cones(i).YAvg,'y');
                    obj.ConesTARSorted = false;
                end
                if Cones(i).C=='r'
                    obj.TKCs(numel(obj.TKCs)+1)=Cone(Cones(i).XAvg,Cones(i).YAvg,'r');
                end
            end
            %Set initial car position as starting CP
            Xr = Car.X(1)-Car.Lr*cos(Car.Theta(1));
            Yr = Car.Y(1)-Car.Lr*sin(Car.Theta(1));
            
            CarPoint = [Xr;Yr];
            obj.CPs = CarPoint;
            
            %Sort initial Cones to add by distance to Car
            obj.sortConesToAdd(CarPoint,CarPoint);
            
            %Add the first Left and Right side of track cone
            LConeFirst = obj.ConesToAddL(1);
            RConeFirst = obj.ConesToAddR(1);
            obj.LCs = Cone(LConeFirst.X,LConeFirst.Y,'b');
            obj.RCs = Cone(RConeFirst.X,RConeFirst.Y,'y');
            obj.ConesToAddL(1) = [];
            obj.ConesToAddR(1) = [];
            obj.ConesTALSorted = false;
            obj.ConesTARSorted = false;
            
            %Re-sort by distance to recently added cone
            obj.sortConesToAdd([LConeFirst.X;LConeFirst.Y],[RConeFirst.X;RConeFirst.Y]);
            obj.popConesToAdd()
            obj.addPathPairs()
        end
        
        function [X,Y,V] = update(obj,Cones,Car)
            %If haven't reached the final zone (Returned near starting point)
            if(~obj.HaveReachedEndZone)
                %Add any new cones and determine centre path
                obj.addCones(Cones)
                LConeLast = [obj.LCs(end).X;obj.LCs(end).Y];
                RConeLast = [obj.RCs(end).X;obj.RCs(end).Y]; 
                obj.sortConesToAdd(LConeLast,RConeLast);
                obj.popConesToAdd()
                obj.addPathPoints()
            end
            
            %If haven't set the centre points at the start needing to be reused again at end
            %Or If haven't seen the start/finish line
            if(~obj.HaveSetFinalPoints && numel(obj.TKCs)==2)
                deltaX = obj.TKCs(1).X-obj.TKCs(2).X;
                deltaY = obj.TKCs(1).Y-obj.TKCs(2).Y;
                %Cones perfectly vertical
                if(deltaX==0)
                    %Determine the side of the x=c line car was initially on
                    compResult = (obj.CPs(1,1)<obj.TKCs(1).X);
                    %Add to FinalPoints until the Centre Point is across the start line
                    for i=(size(obj.FPs,2)+1):size(obj.CPs,2)
                        if ((obj.CPs(1,i)<obj.TKCs(1).X)==compResult)
                            obj.FPs(:,size(obj.FPs,2)+1)=obj.CPs(:,i);
                        else
                            obj.FPs(:,size(obj.FPs,2)+1)=[obj.TKCs(1).X;(obj.TKCs(1).Y+obj.TKCs(2).Y)/2];
                            obj.FPs(:,size(obj.FPs,2)+1)=obj.CPs(:,i);
                            obj.HaveSetFinalPoints = true;
                            break;
                        end
                    end
                %Cones on a y=mx+c line
                else
                    m = (deltaY)/(deltaY);
                    c = obj.TKCs(1).Y-m*obj.TKCs(1).X;
                    %Determine the side of the y=mx+c line car was initially on
                    compResult = (obj.CPs(2,1)<(m*obj.CPs(1,1)+c));
                    %Add to FinalPoints until the Centre Point is across the start line
                    for i=(size(obj.FPs,2)+1):size(obj.CPs,2)
                        if ((obj.CPs(2,i)<(m*obj.CPs(1,i)+c))==compResult)
                            obj.FPs(:,size(obj.FPs,2)+1)=obj.CPs(:,i);
                        else
                            obj.FPs(:,size(obj.FPs,2)+1)=[(obj.TKCs(1).X+obj.TKCs(2).X)/2;(obj.TKCs(1).Y+obj.TKCs(2).Y)/2];
                            obj.FPs(:,size(obj.FPs,2)+1)=obj.CPs(:,i);
                            obj.HaveSetFinalPoints = true;
                            break;
                        end
                    end
                end
            end
            
            %If haven't reached the end zone (Near starting position of Car)
            if(~obj.HaveReachedEndZone)
                %If haven't left the start zone
                if(~obj.HaveLeftStartZone)
                    %Check whether travelled 10m away from initial position
                    xDistC = obj.CPs(1,1)-Car.X(end);
                    yDistC = obj.CPs(2,1)-Car.Y(end);
                    DistC = sqrt(xDistC^2+yDistC^2);
                    if(DistC>=10)
                        obj.HaveLeftStartZone = true;
                    end
                %If have left the starting zone
                else
                    xDistL = obj.LCs(1).X-obj.LCs(end).X;
                    yDistL = obj.LCs(1).Y-obj.LCs(end).Y;
                    DistL = sqrt(xDistL^2+yDistL^2);
                    xDistR = obj.RCs(1).X-obj.RCs(end).X;
                    yDistR = obj.RCs(1).Y-obj.RCs(end).Y;
                    DistR = sqrt(xDistR^2+yDistR^2);
                    %If most recently found cone within range of first seen cone
                    if(DistL<=5 && DistR<=5)
                        obj.HaveReachedEndZone = true;
                        for i=1:size(obj.FPs,2)
                            obj.CPs(:,size(obj.CPs,2)+1) = obj.FPs(:,i);
                        end 
                    end
                end
            end
            %Re-calculate the reference path,velocity and acceleration
            obj.addVelocityPoints()
            obj.calcSpline()
            obj.Path = ppval(obj.CentreSpline,linspace(1,size(obj.CPs,2),size(obj.CPs,2)*50));
            V = ppval(obj.VelocitySpline,linspace(1,size(obj.CPs,2),size(obj.CPs,2)*50));
            X = obj.Path(1,:);
            Y = obj.Path(2,:);
        end
        
        %Function to determine cubic spline for Centre Path, Velocity and Acceleration
        function calcSpline(obj)
           %Generate parameterised time vectors corresponding to index
           t = 1:size(obj.CPs,2);
           
           %Create x,y positions as vectors
           xy = obj.CPs;
           v = obj.VPs;
           
           %Use spline() to determine coefficients for piecewise Spline 
           obj.CentreSpline = spline(t,xy);
           obj.VelocitySpline = spline(t,v);
        end
        
        %Calculate CPs if new Cones found on either side of the track
        function addPathPoints(obj)
            %Checking all cones not mapped yet
            for i=obj.LC_Index:numel(obj.LCs)
                if(~obj.LCs(i).Mapped)
                    %Find the Closest Opposite Cone and set Centre Point
                    indexOpp = obj.findClosest(obj.LCs(i),obj.RCs);
                    if (indexOpp~=-1)
                        xCur = obj.LCs(i).X;
                        yCur = obj.LCs(i).Y;
                        xOp = obj.RCs(indexOpp).X;
                        yOp = obj.RCs(indexOpp).Y;
                        xMid = (xOp+xCur)/2;
                        yMid = (yOp+yCur)/2;
                        GP = [xMid;yMid];
                        obj.LCs(i).Mapped = true;
                        obj.LC_Index=obj.LC_Index+1;
                        if (~((GP(1)==obj.CPs(1,size(obj.CPs,2))) && (GP(2)==obj.CPs(2,size(obj.CPs,2)))))
                            obj.CPs(:,size(obj.CPs,2)+1) = GP;
                        end
                    end
                end      
            end
            %Checking all cones not mapped yet
            for i=obj.RC_Index:numel(obj.RCs)
                if(~obj.RCs(i).Mapped)
                    %Find the Closest Opposite Cone and set Centre Point
                    indexOpp = obj.findClosest(obj.RCs(i),obj.LCs);
                    if (indexOpp~=-1)
                        xCur = obj.RCs(i).X;
                        yCur = obj.RCs(i).Y;
                        xOp = obj.LCs(indexOpp).X;
                        yOp = obj.LCs(indexOpp).Y;
                        xMid = (xOp+xCur)/2;
                        yMid = (yOp+yCur)/2;
                        GP = [xMid;yMid];
                        obj.RCs(i).Mapped = true;
                        obj.RC_Index=obj.RC_Index+1;
                        if (~((GP(1)==obj.CPs(1,size(obj.CPs,2))) && (GP(2)==obj.CPs(2,size(obj.CPs,2)))))
                            obj.CPs(:,size(obj.CPs,2)+1) = GP;
                        end
                    end
                end      
            end
        end
    
        %Function taking new cones from SLAM and sorting them into track boundaries
        function addCones(obj,Cones)
            %If number of cones from SLAM has increased
            numCones = ((numel(obj.LCs))+(numel(obj.RCs))+(numel(obj.TKCs))+numel(obj.ConesToAddL)+numel(obj.ConesToAddR));
            if(numCones<numel(Cones))
                %Prepare to add cones into memory
                for i=numCones+1:numel(Cones)
                    if(Cones(i).C=='r')
                        obj.TKCs(numel(obj.TKCs)+1) = Cone(Cones(i).X,Cones(i).Y,'r');
                    elseif Cones(i).C=='b'
                        obj.ConesToAddL(numel(obj.ConesToAddL)+1) = Cone(Cones(i).X,Cones(i).Y,'b');
                        obj.ConesTALSorted = false;
                    elseif Cones(i).C=='y'
                        obj.ConesToAddR(numel(obj.ConesToAddR)+1) = Cone(Cones(i).X,Cones(i).Y,'y');
                        obj.ConesTARSorted = false;
                    end
                end
            end
        end
        
        %Function finding closest Cone on Opposite side of Track
        function index = findClosest(~,Cone,ConesOppSide)
            X = Cone.X;
            Y = Cone.Y;
            minDist = 6; %Minimum distance cones will have
            maxDist = 10;     %Maximum distance cones will have
            index = -1;        %-1 denotes no solution found
            %Pass through all cones on opposite side in reverse order
            for i=numel(ConesOppSide):-1:1
                XCone = ConesOppSide(i).X;
                YCone = ConesOppSide(i).Y;
                dist = sqrt((X-XCone)^2+(Y-YCone)^2);
                %Closer cone found
                if dist<minDist
                    index = i;
                    minDist = dist;
                %Cone too far away
                elseif dist>maxDist
                    break;  
                end
            end
        end
        
        %Function sorting cones to be added into memory to correctly order 
        function sortConesToAdd(obj,LeftClosest,RightClosest)
            %Sorting left sided cones
            n = numel(obj.ConesToAddL);
            %Bubble sort of ConesToAdd sorting by shortest distance
            if(n>1 && ~obj.ConesTALSorted)
                for i=1:n-1
                    for j=1:n-i
                        dist_j = sqrt((LeftClosest(1)-obj.ConesToAddL(j).X)^2+(LeftClosest(2)-obj.ConesToAddL(j).Y)^2);
                        dist_j1 = sqrt((LeftClosest(1)-obj.ConesToAddL(j+1).X)^2+(LeftClosest(2)-obj.ConesToAddL(j+1).Y)^2);
                        if dist_j>dist_j1
                            temp = obj.ConesToAddL(j);
                            obj.ConesToAddL(j) = obj.ConesToAddL(j+1);
                            obj.ConesToAddL(j+1) = temp;
                        end
                    end
                end
            end
            %Sorting right sided cones
            n = numel(obj.ConesToAddR);
            if(n>1 && ~obj.ConesTARSorted)
                for i=1:n-1
                    for j=1:n-i
                        dist_j = sqrt((RightClosest(1)-obj.ConesToAddR(j).X)^2+(RightClosest(2)-obj.ConesToAddR(j).Y)^2);
                        dist_j1 = sqrt((RightClosest(1)-obj.ConesToAddR(j+1).X)^2+(RightClosest(2)-obj.ConesToAddR(j+1).Y)^2);
                        if dist_j>dist_j1
                            temp = obj.ConesToAddR(j);
                            obj.ConesToAddR(j) = obj.ConesToAddR(j+1);
                            obj.ConesToAddR(j+1) = temp;
                        end
                    end
                end
            end
            obj.ConesTALSorted = true;
            obj.ConesTARSorted = true;
        end
        
        %Function putting cones into PathPlanner memory if sorted and in range
        function popConesToAdd(obj)
            needToAddMore=true;
            wasAdded = true;
            
            while needToAddMore && wasAdded
                wasAdded = false;
                nL = numel(obj.ConesToAddL);
                nR = numel(obj.ConesToAddR);
                
                if(nL>0)
                    LCDist = sqrt((obj.ConesToAddL(1).X-obj.LCs(end).X)^2+(obj.ConesToAddL(1).Y-obj.LCs(end).Y)^2);
                    if LCDist<5
                        obj.LCs(numel(obj.LCs)+1) = Cone(obj.ConesToAddL(1).X,obj.ConesToAddL(1).Y,'b');
                        obj.ConesToAddL(1) = [];
                        obj.ConesTALSorted = false;
                        wasAdded = true;
                    end
                end
                
                if(nR>0)
                    RCDist = sqrt((obj.ConesToAddR(1).X-obj.RCs(end).X)^2+(obj.ConesToAddR(1).Y-obj.RCs(end).Y)^2);
                    if RCDist<5
                        obj.RCs(numel(obj.RCs)+1) = Cone(obj.ConesToAddR(1).X,obj.ConesToAddR(1).Y,'y');
                        obj.ConesToAddR(1) = [];
                        obj.ConesTARSorted = false;
                        wasAdded = true;
                    end
                end
                
                %If more cones to be added - Sort and attempt to continue adding
                if (numel(obj.ConesToAddL)+numel(obj.ConesToAddR))>0
                    needToAddMore = true;
                    LastLC = obj.LCs(end);
                    LLC = [LastLC.X;LastLC.Y];
                    LastRC = obj.RCs(end);
                    RRC = [LastRC.X;LastRC.Y];
                    obj.sortConesToAdd(LLC,RRC);
                else
                    break;
                end
            end
        end
        
        %Adds CPs Points between Paired track cones (Used on first run only)
        function addPathPairs(obj)
            %Set number of CPs generated
            n = min(numel(obj.LCs),numel(obj.RCs));
            
            %Determine each CP
            for i=1:n
                %If Cone not previously mapped
                if(~obj.LCs(i).Mapped)
                    %Find closest opposite cone 
                    indexOpp = obj.findClosest(obj.LCs(i),obj.RCs);
                    %If a solution is found
                    if (indexOpp~=-1)
                        xCur = obj.LCs(i).X;
                        yCur = obj.LCs(i).Y;
                        xOp = obj.RCs(indexOpp).X;
                        yOp = obj.RCs(indexOpp).Y;
                        xMid = (xOp+xCur)/2;
                        yMid = (yOp+yCur)/2;
                        GP = [xMid;yMid];
                        obj.CPs(:,size(obj.CPs,2)+1) = GP;
                        obj.LCs(i).Mapped = true;
                        obj.RCs(indexOpp).Mapped = true;
                        obj.LC_Index=obj.LC_Index+1;
                    end
                end      
            end
        end
        
        %Function to determine conservative estimate for maximum cornering velocity
        function addVelocityPoints(obj)
            VMax = obj.VMax;
            VConst = obj.VConst;
            
            %Adaptive speed
            if(~obj.setConstantVelocity)
                %N is amount of Centre Points
                n = size(obj.CPs,2);

                %If enough centre points found to estimate radiuses
                if(n>=3)
                    cur_RPs = numel(obj.RPs);
                    goal_RPs = n-2;
                    %Calculate radius estimates centred on each iteration i(Uncalculated CPs)
                    for i=(cur_RPs+2):(goal_RPs+1)
                        p_1 = obj.CPs(:,i-1);
                        p_2 = obj.CPs(:,i);
                        p_3 = obj.CPs(:,i+1);
                        %Determine radius 
                        R = obj.calcCircle(p_1,p_2,p_3);
                        %Each CP(i) corresponds to RP(i-1) as no Radius for CP(1) or CP(end)
                        obj.RPs(i-1) = R;
                    end

                    %Set starting and end velocities
                    obj.VPs(1) = min(obj.FGain*sqrt(obj.RPs(1)),VMax);
                    obj.VPs(n) = 0;
                    %Special case with only 3 CentrePoints
                    if(goal_RPs==1)
                        obj.VPs(2) = min(obj.FGain*sqrt(obj.RPs(1)),VMax);
                    %Case with greater than 3 CentrePoints
                    else
                        %Set the two edge cases (Average of 2 Velocities)
                        obj.VPs(2) = min(obj.FGain*(sqrt(obj.RPs(1))+sqrt(obj.RPs(2)))/2,VMax);
                        obj.VPs(n-1) = min(obj.FGain*(sqrt(obj.RPs(end))+sqrt(obj.RPs(end-1)))/2,VMax);
                        for i=3:n-2
                           %Set normal case (Average of 3 Velocities in Curve)
                           obj.VPs(i) = min(obj.FGain*(sqrt(obj.RPs(i-2))+sqrt(obj.RPs(i-1))+sqrt(obj.RPs(i)))/3,VMax); 
                        end
                    end
                else
                    obj.VPs(1) = VConst;
                    obj.VPs(2) = VConst;
                end
            else
                obj.VPs(1) = VConst;
                for i=2:size(obj.CPs,2)-1
                    obj.VPs(i) = VConst; %#ok<*PROP>
                end
                obj.VPs(size(obj.CPs,2))=0;
            end
        end
        
        %Function to estimate a circle radius from 3 points on perimeter
        function R = calcCircle(~,A,B,C)
            %Find Midpoints
            BA_Mid = (A+B)/2;
            BC_Mid = (B+C)/2;
            AC_Mid = (A+C)/2;

            %Find gradient between 
            BA_m = (A(2)-B(2))/(A(1)-B(1));
            BC_m = (C(2)-B(2))/(C(1)-B(1));
            AC_m = (C(2)-A(2))/(C(1)-A(1));

            %Find perpendicular gradient
            BA_mp = -1/BA_m;
            BC_mp = -1/BC_m;
            AC_mp = -1/AC_m;

            %Determine perpendicular equation to points
            cBA = BA_Mid(2)-BA_mp*BA_Mid(1);
            cBC = BC_Mid(2)-BC_mp*BC_Mid(1);
            cAC = AC_Mid(2)-AC_mp*AC_Mid(1);

            %Find intersection points
            x_BA_BC = (cBC-cBA)/(BA_mp-BC_mp);
            x_BA_AC = (cAC-cBA)/(BA_mp-AC_mp);
            x_BC_AC = (cAC-cBC)/(BC_mp-AC_mp);

            y_BA_BC = BA_mp*x_BA_BC+cBA;
            y_BA_AC = BA_mp*x_BA_AC+cBA;
            y_BC_AC = BC_mp*x_BC_AC+cBC;

            x_int = (x_BA_BC+x_BA_AC+x_BC_AC)/3;
            y_int = (y_BA_BC+y_BA_AC+y_BC_AC)/3;

            rad_A = sqrt((A(2)-y_int)^2+(A(1)-x_int)^2);
            rad_B = sqrt((B(2)-y_int)^2+(B(1)-x_int)^2);
            rad_C = sqrt((C(2)-y_int)^2+(C(1)-x_int)^2);

            R = (rad_A+rad_B+rad_C)/3;  
        end
        
    end
end