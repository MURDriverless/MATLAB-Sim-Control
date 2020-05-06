classdef World < handle
   properties
       Cones_L           %Cones Left-Side of Track (Blue)
       Cones_R           %Cones Right-Side of Track (Yellow)
       Num_CL            %Total Number of Left-Side Cones
       Num_CR            %Total Number of Right-Side Cones
       Cones_Tk          %Time Keeping Cones (Red/Orange)
       Cur_Iter          %Current Simulation Iteration
       Cur_Time          %Current Time of Simulation
       Step_Time         %Step Time of Simulation
       Car               %Car Object
       Left_Spline       %Struct describing spline between cones for Left
       Right_Spline      %Struct describing spline between cones for Right
   end
   methods
       %Constructor for World - Takes track and car inputs
       function obj = World(lx,ly,rx,ry,tk_x,tk_y,c_init_x,c_init_y,c_init_theta,step)
           obj.Num_CL = length(lx);
           obj.Num_CR = length(rx);
           obj.Cones_L = Cone.empty;
           obj.Cones_R = Cone.empty;
           obj.Cones_Tk = Cone.empty;
           obj.Cur_Iter = 0;
           obj.Cur_Time = 0;
           obj.Car = Car(c_init_x,c_init_y,c_init_theta);
           obj.Step_Time = step;
           
           for i=1:obj.Num_CL
               obj.Cones_L(i) = Cone(lx(i),ly(i),'b');
           end
           
           for i=1:obj.Num_CR
               obj.Cones_R(i) = Cone(rx(i),ry(i),'y');
           end
           
           for i=1:2
               obj.Cones_Tk(i) = Cone(tk_x(i),tk_y(i),'r');
           end
           
           %Generate parameterised time vectors corresponding to index
           t_l = 1:numel(lx);
           t_r = 1:numel(rx);
           
           %Create x,y positions as vectors
           xy_left = [lx;ly];
           xy_right = [rx;ry];
           
           %Use spline() to determine coefficients for piecewise Spline 
           obj.Left_Spline = spline(t_l,xy_left);
           obj.Right_Spline = spline(t_r,xy_right);
       end
       
       %Function to Update state of world given controller inputs
       function update(obj,V,Delta_Dot)
           obj.Cur_Iter = obj.Cur_Iter+1;
           obj.Cur_Time = obj.Cur_Time + obj.Step_Time;
           obj.Car.update(obj.Step_Time,V,Delta_Dot);
       end
      
       %Plot L+R Cones and Tk Cones
       function plotTrack(obj,lx,ly,rx,ry,tk_x,tk_y)
           RESOLUTION = 1000;
           
           %Discrete track plot
           plot(lx,ly,'bo')
           hold on
           plot(rx,ry,'yo')
           plot(tk_x,tk_y,'r-s')
           
           %Determine finer resolution points with Spline
           left_track = ppval(obj.Left_Spline,linspace(1,numel(lx),RESOLUTION));
           right_track = ppval(obj.Right_Spline,linspace(1,numel(rx),RESOLUTION));
           
           plot(left_track(1,:),left_track(2,:),'b')
           plot(right_track(1,:),right_track(2,:),'y')
           xlabel("X(m)")
           ylabel("Y(m)")
           title("FSG - Driverless Track")
      
       end
   end
end
