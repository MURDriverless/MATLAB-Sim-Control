classdef SLAM < handle
    properties
        Car_N               %Car Measurements  - Noise
        Cones_N             %Cone Measurements - Noise
        k                   %Current Iteration
        t                   %Time Vector
        R                   %Radius of SLAM Perception
        VelocityBasedStdDev %Set StdDev based on Velocity (Cone Tracking)
        Cone_StdDev         %Noise Value (StdDev)
        Car_StdDev          %Noise Value (StdDev) [16x1]
        T                   %Sample/Step Time
        Cone_Index = 1      %Speeds up searching cones
        LeftConeIndex = 1   %Speeds up SLAM 
        RightConeIndex = 1  %Speeds up SLAM
        TKConeIndex = 1     %Speeds up SLAM
        
    end
    methods
        function obj = SLAM(Cone_StdDev,Car_StdDev,R,T,VelBased)
            obj.k = 1;
            obj.t = 0;
            obj.Car_N = Car(0,0,0);
            obj.Cones_N = Cone.empty;
            obj.Cone_StdDev = Cone_StdDev;
            obj.Car_StdDev = Car_StdDev;
            obj.R = R;
            obj.T = T;
            obj.VelocityBasedStdDev = VelBased;
            
        end
        
        function update(obj,LCs,RCs,TKCs,Car)
            %Update Iteration and Time
            obj.k = obj.k+1;
            obj.t(obj.k) = obj.t(obj.k-1) + obj.T;
            
            %Update Car State
            obj.setCarNoise(Car)
            
            %Update Cone States
            obj.LeftConeIndex = obj.addCones(LCs,Car,obj.LeftConeIndex);
            obj.RightConeIndex = obj.addCones(RCs,Car,obj.RightConeIndex);
            obj.TKConeIndex = obj.addCones(TKCs,Car,obj.TKConeIndex);  
            obj.setConeNoise(Car)
        end
        
        %Function to Recognise Cones in Slam
        function newStartIndex = addCones(obj,Cones,Car,startIndex)
            inRow = true;
            newStartIndex = startIndex;
            %Check each cone in World
            for i=startIndex:numel(Cones)
                %If Cone hasn't been 'Seen'
                if (~Cones(i).Seen)
                    Car_x = Car.X(end);
                    Car_y = Car.Y(end);
                    Cone_x = Cones(i).X;
                    Cone_y = Cones(i).Y;    
                    dist = sqrt((Car_x-Cone_x)^2+(Car_y-Cone_y)^2);
                    %Check within sensor Range
                    if(dist<obj.R)
                        Theta = Car.Theta(end);
                        %Check in front of vehicle (180deg Field Of View)
                        if (Car_x*cos(Theta)+Car_y*sin(Theta))<(Cone_x*cos(Theta)+Cone_y*sin(Theta))
                            %Add new detected cone into SLAM
                            Col = Cones(i).C;
                            Cones(i).Seen = true;
                            obj.Cones_N(numel(obj.Cones_N)+1) = Cone(Cone_x,Cone_y,Col);
                            if inRow
                                newStartIndex = newStartIndex+1;
                            end
                        else
                            inRow = false;
                        end
                    else
                        inRow = false;
                    end
                end
            end
        end
        
        %Function to improve cone position and add sensor noise
        function setConeNoise(obj,Car)
            %Velocity based SLAM Noise - Determines StdDev
            if obj.VelocityBasedStdDev
                V = sqrt(Car.xDot(end)^2+Car.yDot(end)^2);
                obj.Cone_StdDev = 0.1*V/22;
            end
            
            %Set starting cone to update noise/position from (Speeds searching process) 
            index = obj.Cone_Index;
            indexOriginal = index;
            Car_x = Car.X(end);
            Car_y = Car.Y(end);
            Cone_x = obj.Cones_N(index).X;
            Cone_y = obj.Cones_N(index).Y;
            
            %Search for first cone in range of sensors
            while sqrt((Car_x-Cone_x)^2+(Car_y-Cone_y)^2)>obj.R
                if index<numel(obj.Cones_N)
                    obj.Cone_Index=obj.Cone_Index+1;
                    index = index+1;
                else
                    obj.Cone_Index=1;
                    index=1;
                end
                %Full search with none in range
                if index==indexOriginal
                    break;
                end
                %Update for next Cone
                Cone_x = obj.Cones_N(index).X;
                Cone_y = obj.Cones_N(index).Y;
            end
            
            counter = 0;
            %For each cone seen by SLAM (Starting from first in range)
            for i=[index:numel(obj.Cones_N),1:index]
                counter=counter+1;
                Cone_x = obj.Cones_N(i).X;
                Cone_y = obj.Cones_N(i).Y;
                %Once cone outside of range again OR Every cone was measured
                if(sqrt((Car_x-Cone_x)^2+(Car_y-Cone_y)^2)>obj.R || counter>numel(obj.Cones_N))
                    break;
                else
                    %Update Cone Noise and Average Position
                    noise = randn(2,1).*obj.Cone_StdDev;
                    obj.Cones_N(i).numSeen = obj.Cones_N(i).numSeen+1;
                    obj.Cones_N(i).XAvg = (obj.Cones_N(i).XAvg*(obj.Cones_N(i).numSeen-1)+obj.Cones_N(i).X+noise(1))/obj.Cones_N(i).numSeen;
                    obj.Cones_N(i).YAvg = (obj.Cones_N(i).YAvg*(obj.Cones_N(i).numSeen-1)+obj.Cones_N(i).Y+noise(2))/obj.Cones_N(i).numSeen;
                end
            end
        end
        
        %Set noise of car states (Currently not used - Need StdDev of each
        %measurement and what is measurable)
        function setCarNoise(obj,Car)
            noise = randn(16,1).*obj.Car_StdDev;
            i = Car.k;
            obj.Car_N.X(i) = Car.X(end) + noise(1);
            obj.Car_N.Y(i) = Car.Y(end) + noise(2);
            obj.Car_N.Theta(i) = Car.Theta(end) + noise(3);
            obj.Car_N.Delta(i) = Car.Delta(end) + noise(4);
            obj.Car_N.a(i) = Car.a(end) + noise(5);
            obj.Car_N.DDot(i) = Car.DDot(end) + noise(6);
            obj.Car_N.xDot(i) = Car.xDot(end) + noise(7);
            obj.Car_N.yDot(i) = Car.yDot(end) + noise(8);
            obj.Car_N.XDot(i) = Car.XDot(end) + noise(9);
            obj.Car_N.YDot(i) = Car.YDot(end) + noise(10);
            obj.Car_N.xDDot(i) = Car.xDDot(end) + noise(11);
            obj.Car_N.yDDot(i) = Car.yDDot(end) + noise(12);
            obj.Car_N.XDDot(i) = Car.XDDot(end) + noise(13);
            obj.Car_N.YDDot(i) = Car.YDDot(end) + noise(14);
            obj.Car_N.TDot(i) = Car.TDot(end) + noise(15);
            obj.Car_N.TDDot(i) = Car.TDDot(end) + noise(16);
        end
    end
end