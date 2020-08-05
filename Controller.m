classdef Controller < handle
    properties
        Kdd                %Gain setting Lookahead distance from V
        T                  %Sampling Time
        GP = [0,0]         %Current Goal Point
        pathIndex = 1      %Path Index - Where to start searching from
        LdMod              %Current Lookahead Distance
        PIDCounter         %Counter to seperate PID frequency vs Controller frequency
        V_Ref              %Velocity Reference
        D_Ref              %Delta Reference
        V_E = 0            %Velocity Error
        V_E_Int = 0        %Velocity Error Integral
        D_E = 0            %Delta Error
        D_E_Int = 0        %Delta Error Integral
        Kp_V = 10          %Proportional Control - Velocity
        Ki_V = 0.5         %Integral Control - Velocity
        Kd_V = 0.01        %Derivative Control - Velocity
        Kp_D = 5           %Proportional Control - Delta
        Ki_D = 0.5         %Integral Control - Delta
        Kd_D = 0.01        %Derivative Control - Delta
        aPrev = 0;         %Previous acceleration control
        dDotPrev = 0;      %Previous steering angle rate control
        minLd              %Minimum look ahead distance
        maxLd              %Maximum look ahead distance
        FGain              %Maximum Centripetal Force (Proportional to Sqrt-Radius)
        
    end
    methods
       function obj = Controller(Kdd,T,minLd,maxLd,FGain)
           obj.Kdd = Kdd;
           obj.T = T;
           obj.PIDCounter = 1;
           obj.minLd = minLd;
           obj.maxLd = maxLd;
           obj.FGain = FGain;
       end
       
       function updateLd(obj,V)
           obj.LdMod = obj.minLd + obj.Kdd*V;
            if obj.LdMod>obj.maxLd        %Maximum look ahead on unknown
               obj.LdMod = obj.maxLd;
           end
       end
       
       function [a,DeltaDot] = update(obj,VRef,Xpath,Ypath,Car)
           X = Car.X(end);
           Y = Car.Y(end);
           V = sign(Car.xDot(end))*sqrt(Car.XDot(end)^2+Car.YDot(end)^2);
           D = Car.Delta(end);
           
           if(obj.PIDCounter==1)
               obj.updateLd(V);
               Theta = Car.Theta(end);
               L = Car.L;
               Lr = Car.Lr;  
               Xr = X-Lr*cos(Theta);
               Yr = Y-Lr*sin(Theta);
           
               d = realmax;
               for i=obj.pathIndex:numel(Xpath)
                   dist = sqrt((Xr-Xpath(i))^2+(Yr-Ypath(i))^2);
                   if dist>obj.minLd
                       break;
                   end
                   if dist<d
                       d = dist;
                       obj.pathIndex = i;
                   end
               end
           
               obj.GP = obj.determineGP(Xpath,Ypath,Xr,Yr,obj.pathIndex);
               [obj.D_Ref,V_Max] = obj.determineDelta(obj.GP,Xr,Yr,Theta,L);
               obj.V_Ref = min(VRef(obj.pathIndex),V_Max);
           end
           
           V_E_Prev = obj.V_E;
           D_E_Prev = obj.D_E;
           obj.V_E = obj.V_Ref-V;
           obj.D_E = obj.D_Ref-D;
           if obj.aPrev<10 && obj.aPrev>-15
                obj.V_E_Int = obj.V_E_Int+obj.V_E*obj.T;
           end
           if obj.dDotPrev<pi && obj.dDotPrev>-pi
                obj.D_E_Int = obj.D_E_Int+obj.D_E*obj.T;
           end
           
           a = obj.Kp_V*obj.V_E + obj.Ki_V*(obj.V_E_Int) + obj.Kd_V*(obj.V_E-V_E_Prev)/obj.T;
           DeltaDot = obj.Kp_D*obj.D_E + obj.Ki_D*(obj.D_E_Int) + obj.Kd_D*(obj.D_E-D_E_Prev)/obj.T;
           
           if(obj.GP(1)==Xpath(end) && obj.GP(2)==Ypath(end))
               DeltaDot = 0;
           end
               
           
           a = obj.saturateValue(a,-15,10);
           DeltaDot = obj.saturateValue(DeltaDot,-pi,pi);
           
           obj.PIDCounter = obj.PIDCounter+1;
           if obj.PIDCounter == 11
               obj.PIDCounter = 1;
           end           
           
           obj.aPrev = a;
           obj.dDotPrev = DeltaDot;
       end
       
       %Calculate GoalPoint for Current Path
       function GP = determineGP(obj,Xpath,Ypath,X,Y,index)
           last = index-1;
           %For each element on path
           for j = index:numel(Xpath)
               %Find distance between
               xDelta = Xpath(j)-X;
               yDelta = Ypath(j)-Y;
               D = sqrt(xDelta^2+yDelta^2);
               %If Goal within lookahead - Skip to next
               if D < obj.LdMod
                   last = last+1;
               else
                   break;
               end
           end
           
           %If no goal within the set Lookahead distance - Reached final goal
           if (last>numel(Xpath))
               GP = [Xpath(end),Ypath(end)];
           else
               %Goal Point
               GP = [Xpath(last),Ypath(last)];
           end
       end
       
       function [Delta,VMax] = determineDelta(obj,GP,X,Y,Theta,L)
           Alpha = atan2((GP(2)-Y),(GP(1)-X))-Theta;
           Delta = atan2((2*L*sin(Alpha)),obj.LdMod);
           VMax = 3*sqrt(obj.LdMod/(2*abs(sin(Alpha))));
       end
       
       function sat = saturateValue(~,val,min,max)
           if val>max
               sat=max;
           elseif val<min
               sat=min;
           else
               sat=val;
           end
       end
    end
end