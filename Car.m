classdef Car < handle
    properties
        %Main State Values
        X                     %Global X Position (m)
        Y                     %Global Y Position (m)
        Theta                 %Car Angle Relative to X-Axis (rad)
        Delta = 0;            %Steering Angle (Front Wheel) (rad)
        
        %Input States
        a = 0;                %CoG Acceleration (m/s^2)
        DDot = 0;             %Steering Rate    (rad/s)
        
        %Inbetween States (Derivatives)
        xDot = 0;             %Longitudinal Speed (m/s)
        yDot = 0;             %Lateral Speed      (m/s)
        XDot = 0;             %Global X-Axis Speed     (m/s)
        YDot = 0;             %Global Y-Axis Speed     (m/s)
        xDDot = 0;            %Longitudinal Acceleration  (m/s^2)
        yDDot = 0;            %Lateral Acceleration       (m/s^2)
        XDDot = 0;            %Global X-Axis Acceleration (m/s^2)
        YDDot = 0;            %Global Y-Axis Acceleration (m/s^2)
        TDot = 0;             %Car Anglular Velocity     (rad/s)
        TDDot = 0;            %Car Angular Acceleration (rad/s^2)
        
        %Lateral Friction States
        Fc_r = 0;             %Rear Wheel Lateral Friction  (N)
        Fc_f = 0;             %Front Wheel Lateral Friction (N)
        s_r = 0;              %Rear Wheel Slip Angle  (Rad)
        s_f = 0;              %Front Wheel Slip Angle (Rad)
        
        %Car Properties
        L = 2.951;          %Car Length (Wheel-Wheel) (m)
        Lf = 2.951/2;       %CoG -> Front Wheel       (m)
        Lr = 2.951/2;       %CoG -> Rear Wheel        (m)
        m = 250;            %Mass of Car              (Kg)
        Izz = 169.6;        %Yaw Moment of Inertia  (Kg m^2)
        C = 22154.36;       %Cornering Stiffness (Linear) (N/Rad)
        Fc_Max = 2900;      %Lateral Friction Limit   (N)
        
        %Simulation properties
        k = 1;              %Simulation Iteration
        t = 0;              %Time 
    end
    methods
        %Initialise Car
        function obj = Car(X,Y,Theta)
            obj.X = X;
            obj.Y = Y;
            obj.Theta = Theta;
        end
        
        %Set Current State of Car
        function setState(obj,Car)
            obj.k = Car.k;
            i = obj.k;
            obj.X(i) = Car.X(end);
            obj.xDot(i) = Car.xDot(end);
            obj.Y(i) = Car.Y(end);
            obj.yDot(i) = Car.yDot(end);
            obj.XDot(i) = Car.XDot(end);
            obj.YDot(i) = Car.YDot(end);
            obj.xDDot(i) = Car.xDDot(end);
            obj.yDDot(i) = Car.yDDot(end);
            obj.XDDot(i) = Car.XDDot(end);
            obj.YDDot(i) = Car.YDDot(end);
            obj.Theta(i) = Car.Theta(end);
            obj.TDot(i) = Car.TDot(end);
            obj.TDDot(i) = Car.TDDot(end);
            obj.Delta(i) = Car.Delta(end);
            obj.DDot(i) = Car.DDot(end);
            obj.a(i) = Car.a(end);
            obj.t = Car.t;
        end
        
        %Physics of the Car (Dynamic Model - Linear Friction)
        %Uses 2nd Order Taylor Series if Possible -> Otherwise 1st Order
        function update(obj,T,a,DDot)
            obj.k = obj.k + 1;
            i = obj.k;
            obj.t = obj.t+T;
            obj.DDot(i) = DDot;
            obj.a(i) = a;
            obj.Delta(i) = obj.Delta(i-1) + obj.DDot(i)*T;  %Backwards Euler - Makes Steering more responsive
            
            obj.X(i) = obj.X(i-1) + obj.XDot(i-1)*T + 1/2*obj.XDDot(i-1)*T^2;
            obj.Y(i) = obj.Y(i-1) + obj.YDot(i-1)*T + 1/2*obj.YDDot(i-1)*T^2;
            obj.Theta(i) = obj.Theta(i-1) + obj.TDot(i-1)*T + 1/2*obj.TDDot(i-1)*T^2;
            obj.xDot(i) = obj.xDot(i-1) + obj.xDDot(i-1)*T;
            obj.yDot(i) = obj.yDot(i-1) + obj.yDDot(i-1)*T;
            obj.TDot(i) = obj.TDot(i-1) + obj.TDDot(i-1)*T;
            obj.XDot(i) = obj.XDot(i-1) + obj.XDDot(i-1)*T;
            obj.YDot(i) = obj.YDot(i-1) + obj.YDDot(i-1)*T;
            
            obj.Fc_r(i) = obj.Fcr();
            obj.Fc_f(i) = obj.Fcf();
            obj.s_r(i) = atan2((obj.yDot(i)+obj.Lr*obj.TDot(i)),(obj.xDot(i)));
            obj.s_f(i) = obj.Delta(i)-atan2((obj.yDot(i)-obj.Lr*obj.TDot(i)),(obj.xDot(i)));
            
            obj.xDDot(i) = obj.TDot(i)*obj.yDot(i)+obj.a(i);
            obj.yDDot(i) = -obj.TDot(i)*obj.xDot(i)+1/obj.m*(obj.Fc_r(i)+cos(obj.Delta(i))*obj.Fc_f(i));
            obj.TDDot(i) = 1/obj.Izz*(obj.Lf*obj.Fc_f(i)*cos(obj.Delta(i))-obj.Lr*obj.Fc_r(i));
            obj.XDDot(i) = (obj.xDDot(i)-obj.yDot(i)*obj.TDot(i))*cos(obj.Theta(i))-(obj.yDDot(i)+obj.xDot(i)*obj.TDot(i))*sin(obj.Theta(i));
            obj.YDDot(i) = (obj.xDDot(i)-obj.yDot(i)*obj.TDot(i))*sin(obj.Theta(i))+(obj.yDDot(i)+obj.xDot(i)*obj.TDot(i))*cos(obj.Theta(i));
        end
        
        %Calculate Lateral Friction at Front Wheel
        function Fc = Fcf(obj)
            i = obj.k;
            %Slip angle (Front)
            s = obj.Delta(i)-atan2((obj.yDot(i)+obj.Lf*obj.TDot(i)),(obj.xDot(i)));
            
            %Saturation Non-Linearity with Slip vs Friction
            if s>deg2rad(7.5)
                Fc = obj.Fc_Max;
            elseif s<-deg2rad(7.5)
                Fc = -obj.Fc_Max;
            else
                Fc = obj.C*s;
            end
        end
        
        %Calculate Lateral Friction at Rear Wheel
        function Fc = Fcr(obj)
            i = obj.k;
            %Slip Angle (Rear)
            s = -atan2((obj.yDot(i)-obj.Lr*obj.TDot(i)),(obj.xDot(i)));
            
            %Saturation Non-Linearity with Slip vs Friction
            if s>deg2rad(7.5)
                Fc = obj.Fc_Max;
            elseif s<-deg2rad(7.5)
                Fc = -obj.Fc_Max;
            else
                Fc = obj.C*s;
            end
        end
  
    end
end