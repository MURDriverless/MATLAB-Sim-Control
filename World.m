classdef World < handle
   properties
       LCs               %Cones Left-Side of Track  (Blue)
       RCs               %Cones Right-Side of Track (Yellow)
       nL                %Number of Left Cones
       nR                %Number of Right Cones
       TKCs              %Time Keeping Cones (Red) 
       k                 %Current Simulation Iteration
       t                 %Time Vector of Simulation
       T                 %Step Time of Simulation
       Car               %Car Object
   end
   methods
       %Constructor for World - Takes track and car inputs
       function obj = World(lx,ly,rx,ry,tk_x,tk_y,cx0,cy0,cTheta0,T)
           obj.nL = length(lx);
           obj.nR = length(rx);
           obj.LCs = Cone.empty;
           obj.RCs = Cone.empty;
           obj.TKCs = Cone.empty;
           obj.k = 1;
           obj.t = 0;
           obj.Car = Car(cx0,cy0,cTheta0);
           obj.T = T;
           
           for i=1:obj.nL
               obj.LCs(i) = Cone(lx(i),ly(i),'b');
           end
           
           for i=1:obj.nR
               obj.RCs(i) = Cone(rx(i),ry(i),'y');
           end
           
           for i=1:2
               obj.TKCs(i) = Cone(tk_x(i),tk_y(i),'r');
           end
       end
       
       %Function to Update state of world given controller inputs
       function update(obj,a,DDot)
           obj.k = obj.k+1;
           obj.t(obj.k) = obj.t(obj.k-1) + obj.T;
           obj.Car.update(obj.T,a,DDot);
       end
   end
end
