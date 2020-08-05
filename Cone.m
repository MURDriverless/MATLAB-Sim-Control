classdef Cone < handle
   properties
      X               %True X Position of Cone
      Y               %True Y Position of Cone
      W = 0.228;      %Default Width in m
      C               %Colour of Cone
      XAvg            %Average X Position
      YAvg            %Average Y Position
      numSeen = 0;    %Amount of Times Scanned by SLAM
      Mapped = false; %Mapped Geometrically by PathPlanner
      Seen = false;   %Seen and Recorded at least once by SLAM
      
   end
   methods
       function obj = Cone(X,Y,colour)
           obj.X = X;
           obj.Y = Y;
           obj.C = colour;
           obj.XAvg = X;
           obj.YAvg = Y; 
       end
   end
end