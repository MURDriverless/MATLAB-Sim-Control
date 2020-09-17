classdef CentrePoint < handle
    properties
        X
        Y
        LConeIndex
        RConeIndex
        isConstant
        type
    end
    methods
        function obj = CentrePoint(LCIndex,RCIndex,LCs,RCs,isConstant,type)
            obj.isConstant = isConstant;
            if(~isConstant)
                obj.LConeIndex = LCIndex;
                obj.RConeIndex = RCIndex;
                obj.X = (LCs(LCIndex).X+RCs(RCIndex).X)/2;
                obj.Y = (LCs(LCIndex).Y+RCs(RCIndex).Y)/2;
                obj.type = type;
            else
                obj.X = LCIndex;
                obj.Y = RCIndex;
            end
        end
        
        function update(obj,LCs,RCs)
            if(~obj.isConstant)
                obj.X = (LCs(obj.LConeIndex).X+RCs(obj.RConeIndex).X)/2;
                obj.Y = (LCs(obj.LConeIndex).Y+RCs(obj.RConeIndex).Y)/2;
            end
        end
    end
end