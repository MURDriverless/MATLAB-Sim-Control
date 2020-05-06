classdef Controller < handle
    properties
        Ld
        Kdd
        LdMod
        leftStartZone
        isComplete
        SampleTime
        GP
        Index
    end
    methods
       function obj = Controller(Ld,Kdd,SampleTime)
           obj.Ld = Ld;
           obj.Kdd = Kdd;
           obj.isComplete = false;
           obj.SampleTime = SampleTime;
           obj.leftStartZone = false;
           obj.GP = [0,0];
           obj.Index = 1;
       end
       
       function updateLd(obj,V)
           obj.LdMod = obj.Kdd*V;
       end
       
       function [V,DeltaDot] = update(obj,V,Xpath,Ypath,Car)
           step = obj.SampleTime;
           X = Car.X(end);
           Y = Car.Y(end);
           FP = [Xpath(end),Ypath(end)];
           Theta = Car.Theta(end);
           L = Car.L;
           DeltaPrev = Car.Delta(end);
           obj.GP = obj.determineGP(Xpath,Ypath,X,Y,obj.Index);
           DeltaDot = obj.determineDelta(obj.GP,X,Y,Theta,L,step,DeltaPrev);
           if ~obj.leftStartZone
                hasLeft = obj.checkLeftZone(FP,X,Y,0.05);
                if hasLeft
                    obj.leftStartZone = true;
                end
           else
                isCompleted = obj.checkComplete(FP,X,Y,0.05);
                if isCompleted
                    obj.isComplete = true;
                    V = 0;
                end
           end
       end
       
       %Calculate GoalPoint for Current Path
       function [GP,GP_index] = determineGP(obj,Xpath,Ypath,X,Y,index)
           last = index-1;
           %For each element on path
           for i = index:numel(Xpath)
               %Find distance between
               xDelta = Xpath(i)-X;
               yDelta = Ypath(i)-Y;
               D = sqrt(xDelta^2+yDelta^2);
               %If Goal within lookahead - Skip to next
               if D < obj.Ld
                   last = last+1;
               else
                   break;
               end
           end
           if (last>numel(Xpath))
               fprintf("SETTING FINAL GOAL\n")
               GP = [Xpath(end),Ypath(end)];
               obj.Index = numel(Xpath);
           else
               %Goal Point
               GP = [Xpath(last),Ypath(last)];
               GP_index = last;
               obj.Index = last;
               fprintf("GOAL POINT:")
               disp(GP)
           end
       end
       
       function DeltaDot = determineDelta(obj,GP,X,Y,Theta,L,step,DeltaPrev)
           Alpha = atan2((GP(2)-Y),(GP(1)-X))-Theta;
           Delta = atan2((2*L*sin(Alpha)),obj.Ld);
           DeltaDot = (Delta-DeltaPrev)/step;
       end
       
       function isComplete = checkComplete(~,FP,X,Y,R)
           xDist = FP(1)-X;
           yDist = FP(2)-Y;
           Dist = sqrt(xDist^2+yDist^2);
           if Dist<R
               isComplete = true;
           else
               isComplete = false;
           end
       end
       
       function hasLeft = checkLeftZone(~,SP,X,Y,R)
           xDist = SP(1)-X;
           yDist = SP(2)-Y;
           Dist = sqrt(xDist^2+yDist^2);
           if Dist<R
               hasLeft = false;
           else
               hasLeft = true;
           end
       end
    end
end