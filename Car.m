classdef Car < handle
    properties
        X
        X_Dot
        Y
        Y_Dot
        Theta
        Theta_Dot
        V
        Beta
        Gamma_F
        Gamma_F_Dot
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
            obj.Beta = 0;
            obj.Gamma_F = 0;
            obj.Gamma_F_Dot = 0;
            obj.Cur_Iter = 1;
            obj.Cur_Time = 0;
        end
        
        function setVals(obj,Car_Vec)
            obj.X = Car_Vec(1);
            obj.Y = Car_Vec(2);
            obj.Theta = Car_Vec(3);
            obj.V = Car_Vec(4);
            obj.Theta_Dot = Car_Vec(5);
        end
        
        function state = update(obj,steptime,V,Gamma_F_Dot)
            obj.Cur_Iter = obj.Cur_Iter+1;
            i = obj.Cur_Iter;
            obj.Cur_Time = obj.Cur_Time+steptime;
   
            obj.Gamma_F_Dot(i) = Gamma_F_Dot;
            obj.V(i) = V;
            obj.Gamma_F(i) = obj.Gamma_F(i-1) + (obj.Gamma_F_Dot(i)+obj.Gamma_F_Dot(i-1))*steptime/2;
            obj.Beta(i) = atan(obj.Lr*tan(obj.Gamma_F(i))/obj.L);
            obj.Theta_Dot(i) = obj.V(i)*sin(obj.Beta(i))/obj.Lr;
            obj.Theta(i) = obj.Theta(i-1) + (obj.Theta_Dot(i)+obj.Theta_Dot(i-1))*steptime/2;
            obj.X_Dot(i) = obj.V(i)*cos(obj.Theta(i)+obj.Beta(i));
            obj.X(i) = obj.X(i-1) + (obj.X_Dot(i)+obj.X_Dot(i-1))*steptime/2;
            obj.Y_Dot(i) = obj.V(i)*sin(obj.Theta(i)+obj.Beta(i));
            obj.Y(i) = obj.Y(i-1) + (obj.Y_Dot(i)+obj.Y_Dot(i-1))*steptime/2;

            state = [obj.X(i),obj.X_Dot(i),obj.Y(i),obj.Y_Dot(i),obj.Theta(i),...
                obj.Theta_Dot(i),obj.V(i),obj.Beta(i),obj.Gamma_F(i),...
                obj.Gamma_F_Dot(i)];
            
            
        end
    end
end