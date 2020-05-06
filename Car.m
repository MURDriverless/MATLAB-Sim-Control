classdef Car < handle
    properties
        %Main State Values
        X
        Y
        Theta
        Delta
        %Input States
        V
        Delta_Dot
        %Inbetween States (Derivatives)
        X_Dot
        Y_Dot
        Theta_Dot
        Cur_Iter
        Cur_Time
        L = 2.951;
        Lf = 2.951/2;
        Lr = 2.951/2;
        W = 1.420;
    end
    methods
        function obj = Car(x_init,y_init,theta_init)
            obj.X = x_init;
            obj.X_Dot = 0;
            obj.Y = y_init;
            obj.Y_Dot = 0;
            obj.Theta = theta_init;
            obj.Theta_Dot = 0;
            obj.V = 0;
            obj.Delta = 0;
            obj.Delta_Dot = 0;
            obj.Cur_Iter = 1;
            obj.Cur_Time = 0;
        end
        
        function setVals(obj,Car)
            obj.Cur_Iter = Car.Cur_Iter;
            i = obj.Cur_Iter;
            obj.X(i) = Car.X(end);
            obj.X_Dot(i) = Car.X_Dot(end);
            obj.Y(i) = Car.Y(end);
            obj.Y_Dot(i) = Car.Y_Dot(end);
            obj.Theta(i) = Car.Theta(end);
            obj.Theta_Dot(i) = Car.Theta_Dot(end);
            obj.V(i) = Car.V(end);
            obj.Delta(i) = Car.Delta(end);
            obj.Delta_Dot(i) = Car.Delta_Dot(end);
            obj.Cur_Time = Car.Cur_Time;
        end
        
        function setNoise(obj,CarNoise,Car_R)
            obj.Cur_Iter = Car_R.Cur_Iter;
            obj.Cur_Time = Car_R.Cur_Time;
            i = obj.Cur_Iter;
            obj.X(i) = Car_R.X(end) + CarNoise(1);
            obj.Y(i) = Car_R.Y(end) + CarNoise(2);
            obj.Theta(i) = Car_R.Theta(end) + CarNoise(3);
            obj.Delta(i) = Car_R.Delta(end) + CarNoise(4);
            obj.V(i) = Car_R.V(end) + CarNoise(5);
            obj.Delta_Dot(i) = Car_R.Delta_Dot(end) + CarNoise(6);
            obj.X_Dot(i) = Car_R.X_Dot(end) + CarNoise(7);
            obj.Y_Dot(i) = Car_R.Y_Dot(end) + CarNoise(8);
            obj.Theta_Dot(i) = Car_R.Theta_Dot(end) + CarNoise(9);
            
        end
        
        function update(obj,steptime,V,Delta_Dot)
            obj.Cur_Iter = obj.Cur_Iter+1;
            i = obj.Cur_Iter;
            obj.Cur_Time = obj.Cur_Time+steptime;
            obj.Delta_Dot(i) = Delta_Dot;
            obj.V(i) = V;
            obj.Delta(i) = obj.Delta(i-1) + (obj.Delta_Dot(i)+obj.Delta_Dot(i-1))*steptime/2;
            obj.Theta_Dot(i) = obj.V(i)*tan(obj.Delta(i))/obj.L;
            obj.Theta(i) = obj.Theta(i-1) + (obj.Theta_Dot(i)+obj.Theta_Dot(i-1))*steptime/2;
            obj.X_Dot(i) = obj.V(i)*cos(obj.Theta(i));
            obj.X(i) = obj.X(i-1) + (obj.X_Dot(i)+obj.X_Dot(i-1))*steptime/2;
            obj.Y_Dot(i) = obj.V(i)*sin(obj.Theta(i));
            obj.Y(i) = obj.Y(i-1) + (obj.Y_Dot(i)+obj.Y_Dot(i-1))*steptime/2;         
        end
    end
end