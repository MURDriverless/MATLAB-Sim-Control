classdef SLAM < handle
    properties
        Car_N               %Car Measurements  - Noise
        Car_R               %Car Measurements - Real
        Cones_N             %Cone Measurements - Noise
        Cones_R             %Cone Measurements - Real
        CurIter             %Current Iteration
        CurTime             %Current Time
        Radius              %Radius of SLAM Perception
        Noise_Type          %Uniform/Normal Distribution
        Cone_N_Val          %Noise Value (Range,StdDev)
        Car_N_Val           %Noise Value (Range,StdDev) [5x1]
        SampleTime          %Sample/Step Time
        %SHAPE?             %Potential change shape of SLAM perception
        
        
    end
    methods
        function obj = SLAM(Noise_Type,Cone_N_Val,Car_N_Val,Radius,StepTime)
            obj.CurIter = 0;
            obj.CurTime = 0;
            obj.Car_N = Car(0,0,0);
            obj.Car_R = Car(0,0,0);
            obj.Cones_N = Cone.empty;
            obj.Cones_R = Cone.empty;
            obj.Noise_Type = Noise_Type;
            obj.Cone_N_Val = Cone_N_Val;
            obj.Car_N_Val = Car_N_Val;
            obj.Radius = Radius;
            obj.SampleTime = StepTime;
            
        end
        
        function update(obj,Cones_L,Cones_R,Cones_Tk,Car)
            %Update Iteration and Time
            obj.CurIter = obj.CurIter+1;
            obj.CurTime = obj.CurTime+obj.SampleTime;
            
            %Update Car State
            obj.Car_R.setVals(Car);
            
            %Update Cone States
            obj.addCones(Cones_L);
            obj.addCones(Cones_R);
            obj.addCones(Cones_Tk);
           
            if(strcmp(obj.Noise_Type,'uniform'))
                obj.setUniformNoise(obj.Car_N_Val,obj.Cone_N_Val);
            else
                obj.setNormalNoise(obj.Car_N_Val,obj.Cone_N_Val);
            end
            
        end
        
        function addCones(obj,Cones)
            for i=1:numel(Cones)
                isConeIn = obj.isConeIn(Cones(i));
                
                if(~isConeIn)
                    Car_x = obj.Car_R.X(end);
                    Car_y = obj.Car_R.Y(end);
                    Cone_x = Cones(i).X;
                    Cone_y = Cones(i).Y;
                    dist = sqrt((Car_x-Cone_x)^2+(Car_y-Cone_y)^2);
                    if(dist<obj.Radius)
                        Col = Cones(i).Colour;
                        obj.Cones_N(numel(obj.Cones_N)+1) = Cone(Cone_x,Cone_y,Col);
                        obj.Cones_R(numel(obj.Cones_R)+1) = Cone(Cone_x,Cone_y,Col);
                    end
                end
            end
        end
        
        function result = isConeIn(obj,Cone)
            for i = 1:numel(obj.Cones_R)
                if(obj.Cones_R(i).X==Cone.X && obj.Cones_R(i).Y==Cone.Y)
                    result = true;
                    return
                end 
            end
            result = false;
            return
        end
        
        function setUniformNoise(obj,range_car,range_cone)
            noise_cones = (rand(2,numel(obj.Cones_R))-0.5).*range_cone;
            noise_car = (rand(1,numel(range_car))-0.5).*range_car;
            obj.setNoise(noise_cones,noise_car);
          
        end
        
        function setNormalNoise(obj,stddev_car,stddev_cone)
           noise_cones = (randn(2,numel(obj.Cones_R)).*stddev_cone);
           noise_car = (randn(1,numel(stddev_car))).*stddev_car;
           obj.setNoise(noise_cones,noise_car);
        end
        
        function setNoise(obj,noise_cones,noise_car)
             for i = 1:numel(obj.Cones_R)
                X = obj.Cones_R(i).X + noise_cones(1,i);
                Y = obj.Cones_R(i).Y + noise_cones(2,i);
                obj.Cones_N(i).setPosition(X,Y);
             end
            obj.Car_N.setNoise(noise_car,obj.Car_R);
        end 
    end
end