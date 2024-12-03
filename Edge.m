classdef Edge < handle
    properties
        position
        width
        length
        maxLength
        direction
        GridLocation
        inclination
        BendingAmount
        IsActive
        IsOverridden
        k1
        k2
        k
        UIElement
    end

    methods
        function obj = Edge(width,length,position,direction,GridLocation,maxLength)
            % Constructor
            obj.width=width;
            obj.length=length;
            obj.maxLength=maxLength;
            obj.position= position;
            obj.direction=direction;
            obj.GridLocation=GridLocation;
            obj.IsActive=false;
            obj.IsOverridden=false;
            obj.UpdateStiffness();
            obj.inclination=0;
            obj.BendingAmount=0;
            obj.UIElement.created=false;
            obj.k=0.98;
        end

        function obj = UpdateBending(obj, BendingAmountDegree)
            % Update
            obj.BendingAmount=BendingAmountDegree*pi/180;
            obj.inclination = ComputeInclination(obj.BendingAmount);
        end

        function obj = UpdateEdgeSize(obj, width,length)
            obj.width=width;
            obj.length=length;
        end

        function obj = Activate(obj)
            obj.IsActive=true;
        end


        % Functions to be used if the UIElements are created in this class
        function obj = CreateUIElement(obj, fig, position , tag)
            if(~obj.UIElement.created)
                obj.UIElement.button=uibutton(fig);
                obj.UIElement.label=uilabel(fig);
            end
            obj.UIElement.button.Position = position;
            obj.UIElement.button.Tag=tag;
            obj.UIElement.button.ButtonPushedFcn =@(btn,evt)ChangeStiffness(obj,btn,evt);
            obj.UIElement.button.BackgroundColor  = [0 0.5 0.5];

            obj.UIElement.label.Position = [position(1)+position(4)/5 position(2)+position(3)/20 30 20];
            obj.UIElement.label.Text=tag;
            obj.UIElement.created=true;
        end


        function obj=UpdateStiffness(obj)
            obj.k1=1-(obj.length/obj.maxLength)*0.8;
            obj.k2=obj.width;
        end
    end
end

function Inclination = ComputeInclination(BendingAmount)
Inclination=BendingAmount;
% Take from Kinematic Eqns folder, needs i and j data or odd or
% even information for the direction of the inclination
end