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
        WeightDist                     %Weighting on sorting by Distance
        WeightAngleDiff                %Weighting on sorting by Angle between cones
        WeightTrackWidth               %Weighting on sorting by Expected Track Width
        
    end
    methods
        function obj = PathPlanner(Cones,Car,isConstantVelocity,VMax,VConst,FGain,WeightDist,WeightAngleDiff,WeightTrackWidth)
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
            obj.WeightDist = WeightDist;
            obj.WeightAngleDiff = WeightAngleDiff;
            obj.WeightTrackWidth = WeightTrackWidth;
            
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
            obj.sortConesToAdd(CarPoint,CarPoint,true);
            
            %Add the first Left and Right side of track cone
            LConeFirst = obj.ConesToAddL(1);
            RConeFirst = obj.ConesToAddR(1);
            obj.LCs = Cone(LConeFirst.X,LConeFirst.Y,'b');
            obj.RCs = Cone(RConeFirst.X,RConeFirst.Y,'y');
            obj.ConesToAddL(1) = [];
            obj.ConesToAddR(1) = [];
            obj.ConesTALSorted = false;
            obj.ConesTARSorted = false;
            
            %Re-sort with cost function compared to recently added cone
            obj.sortConesToAdd([LConeFirst.X;LConeFirst.Y],[RConeFirst.X;RConeFirst.Y],false);
            %Add any suitable cones into memory
            obj.popConesToAdd()
            %Generate CentrePoints
            obj.addPathPairs()
        end
        
        function [X,Y,V] = update(obj,Cones,Car)
            %If haven't reached the final zone (Returned near starting point)
            if(~obj.HaveReachedEndZone)
                %Add any new cones and determine centre path
                obj.addCones(Cones)
                LConeLast = [obj.LCs(end).X;obj.LCs(end).Y];
                RConeLast = [obj.RCs(end).X;obj.RCs(end).Y];
                %Sort cones by cost function compared to recently added cones
                obj.sortConesToAdd(LConeLast,RConeLast,false);
                %Add any suitable cones into memory
                obj.popConesToAdd()
                %Generate CentrePoints
                obj.addPathPoints()
            end
            
            %If haven't determined the centre points at the start needing to be reused again at end (finishing lap)
            %Or If haven't seen the start/finish line
            if(~obj.HaveSetFinalPoints && numel(obj.TKCs)==2)
                deltaX = obj.TKCs(1).X-obj.TKCs(2).X;
                deltaY = obj.TKCs(1).Y-obj.TKCs(2).Y;
                %Time Keeping Cones perfectly vertical
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
                    m = (deltaY)/(deltaX);
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
            i = obj.LC_Index;
            j = obj.RC_Index;
            
            %Loop through unmapped left and right side cones
            while ((i<=numel(obj.LCs))||(j<=numel(obj.RCs)))
                if((i<=numel(obj.LCs))&&(~obj.LCs(i).Mapped))
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
                        %Check CP wasn't found last iteration (Avoids two cones pairing with each other twice)
                        if ~((GP(1)==obj.CPs(1,size(obj.CPs,2)))&&(GP(2)==obj.CPs(2,size(obj.CPs,2))))
                            obj.LCs(i).Mapped = true;
                            obj.CPs(:,size(obj.CPs,2)+1) = GP;
                            %Update start of search index (Improve
                            %efficiency and ignores cones with no suitable
                            %partner to generate CP)
                            obj.LC_Index = i;
                        end
                    end
                end
                i=i+1;
                
                if((j<=numel(obj.RCs))&&(~obj.RCs(j).Mapped))
                    %Find the Closest Opposite Cone and set Centre Point
                    indexOpp = obj.findClosest(obj.RCs(j),obj.LCs);
                    if (indexOpp~=-1)
                        xCur = obj.RCs(j).X;
                        yCur = obj.RCs(j).Y;
                        xOp = obj.LCs(indexOpp).X;
                        yOp = obj.LCs(indexOpp).Y;
                        xMid = (xOp+xCur)/2;
                        yMid = (yOp+yCur)/2;
                        GP = [xMid;yMid];
                        %Check CP wasn't found last iteration (Avoids two cones pairing with each other twice)
                        if ~((GP(1)==obj.CPs(1,size(obj.CPs,2)))&&(GP(2)==obj.CPs(2,size(obj.CPs,2))))
                            obj.RCs(j).Mapped = true;
                            obj.CPs(:,size(obj.CPs,2)+1) = GP;
                            %Update start of search index (Improve
                            %efficiency and ignores cones with no suitable
                            %partner to generate CP)
                            obj.RC_Index = j;
                        end
                    end
                end
                j=j+1;
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
            maxGoal = 7;      %Maximum distance cones can be apart without goal point
            maxDist = 10;       %Maximum distance cones will have (Causes exit)
            index = -1;        %-1 denotes no solution found
            %Pass through all cones on opposite side in reverse order
            for i=numel(ConesOppSide):-1:1
                XCone = ConesOppSide(i).X;
                YCone = ConesOppSide(i).Y;
                dist = sqrt((X-XCone)^2+(Y-YCone)^2);
                %Closer cone found
                if dist<maxGoal
                    index = i;
                    maxGoal = dist;
                %Cone too far away
                elseif dist>maxDist
                    break;  
                end
            end
        end
        
        %Function sorting cones to be added into memory to correctly order 
        function sortConesToAdd(obj,LeftClosest,RightClosest,sortDistOnly)
            %Sorting left sided cones
            n = numel(obj.ConesToAddL);
            %Bubble sort of ConesToAdd sorting by shortest distance
            if(n>1 && ~obj.ConesTALSorted)
                for i=1:n-1
                    for j=1:n-i
                        %Sorting by distance between desired cone and cones
                        %to be added
                        if sortDistOnly
                            dist_j = sqrt((LeftClosest(1)-obj.ConesToAddL(j).X)^2+(LeftClosest(2)-obj.ConesToAddL(j).Y)^2);
                            dist_j1 = sqrt((LeftClosest(1)-obj.ConesToAddL(j+1).X)^2+(LeftClosest(2)-obj.ConesToAddL(j+1).Y)^2);
                            if dist_j>dist_j1
                                temp = obj.ConesToAddL(j);
                                obj.ConesToAddL(j) = obj.ConesToAddL(j+1);
                                obj.ConesToAddL(j+1) = temp;
                            end
                        %Sorting with cost function by distance between cones, angle change
                        %and estimated track width for cones to be added
                        else
                            %Determine cost for two cones to be compared
                            e_j = obj.calcCost(LeftClosest,obj.ConesToAddL(j),RightClosest);
                            e_j1 = obj.calcCost(LeftClosest,obj.ConesToAddL(j+1),RightClosest);
                            
                            if (e_j>e_j1)
                                temp = obj.ConesToAddL(j);
                                obj.ConesToAddL(j) = obj.ConesToAddL(j+1);
                                obj.ConesToAddL(j+1) = temp;
                            end    
                        end
                    end
                end
            end
            %Sorting right sided cones
            n = numel(obj.ConesToAddR);
            if(n>1 && ~obj.ConesTARSorted)
                for i=1:n-1
                    for j=1:n-i
                        %Sorting with cost function by distance between cones, angle change
                        %and estimated track width for cones to be added
                        if sortDistOnly
                        dist_j = sqrt((RightClosest(1)-obj.ConesToAddR(j).X)^2+(RightClosest(2)-obj.ConesToAddR(j).Y)^2);
                        dist_j1 = sqrt((RightClosest(1)-obj.ConesToAddR(j+1).X)^2+(RightClosest(2)-obj.ConesToAddR(j+1).Y)^2);
                            if dist_j>dist_j1
                                temp = obj.ConesToAddR(j);
                                obj.ConesToAddR(j) = obj.ConesToAddR(j+1);
                                obj.ConesToAddR(j+1) = temp;
                            end
                        %Sorting by distance between cones, angle change
                        %and estimated track width for cones to be added
                        else
                            %Determine cost for two cones to be compared
                            e_j = obj.calcCost(RightClosest,obj.ConesToAddR(j),LeftClosest);
                            e_j1 = obj.calcCost(RightClosest,obj.ConesToAddR(j+1),LeftClosest);
                                                  
                            if (e_j>e_j1)
                                temp = obj.ConesToAddR(j);
                                obj.ConesToAddR(j) = obj.ConesToAddR(j+1);
                                obj.ConesToAddR(j+1) = temp;
                            end 
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
            
            %Loop until cones no longer being added 
            while needToAddMore && wasAdded
                wasAdded = false;
                
                nL = numel(obj.ConesToAddL);              
                nR = numel(obj.ConesToAddR);
                
                %Most recently added cones on Left and Right side
                LeftLast = [obj.LCs(end).X,obj.LCs(end).Y];
                RightLast = [obj.RCs(end).X,obj.RCs(end).Y];
                
                %Calculate cost of adding cone and key measures that
                %shouldn't be exceeded
                if nL>0
                    [eL,dL,dOppL,angleDiffL] = obj.calcCost(LeftLast,obj.ConesToAddL(1),RightLast);
                end
                
                if nR>0
                    [eR,dR,dOppR,angleDiffR] = obj.calcCost(RightLast,obj.ConesToAddR(1),LeftLast);
                end
                
                if(nL>0)
                    %Left Cone must be within 5m of last, have an angle
                    %change to track curvature less than 45deg and within 7m of last Right Cone
                    %These parameters will likely need further optimisation
                    if (dL<5 && (angleDiffL<(pi/4)) && dOppL<7)
                        %If passes guards add to ordered cone list
                        obj.LCs(numel(obj.LCs)+1) = Cone(obj.ConesToAddL(1).X,obj.ConesToAddL(1).Y,'b');
                        obj.ConesToAddL(1) = [];
                        obj.ConesTALSorted = false;
                        wasAdded = true;
                    end
                end
                
                if(nR>0)
                    %Rightt Cone must be within 5m of last, have an angle
                    %change to track curvature less than 45deg and within 7m of last Left Cone
                    %These parameters will likely need further optimisation
                    if (dR<5 && (angleDiffR<(pi/4)) && dOppR<7)
                        %If passes guards add to ordered cone list
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
                    obj.sortConesToAdd(LLC,RRC,false);
                else
                    break;
                end
            end
        end
        
        %Adds CPs Points between Paired track cones (Used on initialisation only)
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
        
        %Function to determine sorting cost of a proposed cone
        function [e,d,dOpp,angleDiff] = calcCost(obj,PreviousCone,ConeOfInterest,OppPreviousCone)
            %Set weightings of cost function
            w_dist = obj.WeightDist;
            w_ang = obj.WeightAngleDiff;
            w_track_width = obj.WeightTrackWidth;
            
            %Determine candidate cone distances (Between same side and
            %potential track width)
            d = sqrt((PreviousCone(1)-ConeOfInterest.X)^2+(PreviousCone(2)-ConeOfInterest.Y)^2);
            dOpp = sqrt((OppPreviousCone(1)-ConeOfInterest.X)^2+(OppPreviousCone(2)-ConeOfInterest.Y)^2);
            
            %Determine change in path angle - Compared to average of past cones
            if ((numel(obj.LCs)>1)&&(numel(obj.RCs)>1))
                %Angles from 2nd last cone to last cone in range [0,2pi]
                anglePrevL = obj.myatan((obj.LCs(end).Y-obj.LCs(end-1).Y),(obj.LCs(end).X-obj.LCs(end-1).X));         
                anglePrevR = obj.myatan((obj.RCs(end).Y-obj.RCs(end-1).Y),(obj.RCs(end).X-obj.RCs(end-1).X));
                %Determine which of the two angles between is correct (Two
                %solutions for an angle between 'anglePrevL' and 'anglePrevR'
                if abs(anglePrevL-anglePrevR)<(2*pi-abs(anglePrevL-anglePrevR))
                    prevDelta = abs(anglePrevL-anglePrevR);
                    anglePrevMid = min(anglePrevL,anglePrevR)+prevDelta/2;
                else
                    prevDelta = 2*pi-abs(anglePrevL-anglePrevR);
                    anglePrevMid = max(anglePrevL,anglePrevR)+prevDelta/2;
                end
                angleCurrent = obj.myatan((ConeOfInterest.Y-PreviousCone(2)),(ConeOfInterest.X-PreviousCone(1)));
                angleDiff = min(abs(angleCurrent-anglePrevMid),2*pi-abs(angleCurrent-anglePrevMid));
                %Normalise change in angle back to [0,2pi]
                angleDiff = atan(tan(angleDiff));
            else
                angleDiff = 0;
            end
            
            %Calculate cost value
            e = w_dist*(d^2)+w_ang*(angleDiff^2)+w_track_width*(dOpp^2);     
        end
        
        %Function to calculate arctan in range [0,2pi]
        function v = myatan(~,y,x)
            if nargin==1 %just in case the user only gives the value of y myatan(y)
                x=1;
            end
            v=nan;
            if x>0
                v=atan(y/x);
            end
            if y>=0 && x<0
                v=pi+atan(y/x);
            end
            if y<0 && x<0
                v=-pi+atan(y/x);
            end
            if y>0 && x==0
                v=pi/2;
            end
            if y<0 && x==0
                v=-pi/2;
            end
            if v<0
                v=v+2*pi;
            end
        end
    end
end