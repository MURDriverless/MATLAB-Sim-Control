classdef Cone < handle
   properties
      X               %True X Position of Cone
      Y               %True Y Position of Cone
      W = 0.228;      %Default Width in m
      C               %Colour of Cone
      XAvg            %Average X Position (Measured multiple times)
      YAvg            %Average Y Position (Measured multiple times)
      numSeen = 0;    %Amount of Times Scanned by SLAM
      Mapped = false; %Mapped Geometrically by PathPlanner (Used to set GP)
      Seen = false;   %Seen and Recorded at least once by SLAM - Speeds up SLAM emulation
      
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