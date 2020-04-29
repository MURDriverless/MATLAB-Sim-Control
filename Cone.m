classdef Cone < handle
   properties
      X               %X Position of Cone
      Y               %Y Position of Cone
      R = 0.228/2;    %Default radius in m
      Colour
      
   end
   methods
       function obj = Cone(X,Y,colour)
           obj.X = X;
           obj.Y = Y;
           obj.Colour = colour;  
       end
       
       function setPosition(obj,X,Y)
           obj.X = X;
           obj.Y = Y;
       end
           
   end
end