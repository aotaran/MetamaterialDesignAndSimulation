classdef Node < handle
    properties
        Position    % Particle position
        InitialPosition
        Rw
        R0
        RBend
        Rcheck
        NodeSize
        h
        Corners
        DrawingCorners
        CornerLocs
        FinalOrientation
        Deformation
        direction
        w
        q
        ptc
        RotAxis
        AddRot
        InitVertices
        NeighborExists
        a
        GridL
        SlimMode
        UIElement
    end

    methods
        function obj = Node(position, NodeSize, h, NeighborExists,L,direction)
            % Constructor
            obj.Position = position;
            obj.Deformation=0;
            obj.direction=direction;
            obj.InitialPosition = position;
            obj.GridL=L;
            obj.SlimMode = false;
            obj.NodeSize= NodeSize;
            obj.R0=eye(3);
%             obj.q=quaternion(1,0,0,0);
            obj.q=[1;0;0;0];
            obj.Rw=eye(3);
            obj.Rcheck=eye(3);
            obj.RBend=eye(3);
            obj.NeighborExists=NeighborExists;
            obj.UIElement.created=false;
          
            obj.Corners=[-obj.GridL 0 0;0 obj.GridL 0;obj.GridL 0 0; 0 -obj.GridL 0]';
            %             fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];

            if(obj.SlimMode)
                endPoints=NodeSize.*obj.NeighborExists+4*(1-obj.NeighborExists);
                invEp=(obj.GridL-endPoints).*NeighborExists+endPoints.*(1-obj.NeighborExists);
            else
                endPoints=NodeSize.*obj.NeighborExists+max(NodeSize)*(1-obj.NeighborExists);
                invEp=(obj.GridL-endPoints);
            end
                            obj.DrawingCorners=[-endPoints(1) -invEp(1) 0; -endPoints(1) invEp(1) 0;
                    -invEp(2) endPoints(2) 0; invEp(2) endPoints(2) 0;
                    endPoints(3) invEp(3) 0; endPoints(3) -invEp(3) 0;
                    invEp(4) -endPoints(4) 0;-invEp(4) -endPoints(4) 0;...
                    -1 -1 0;-1 1 0;1 1 0;1 -1 0;]';

            obj.CornerLocs=obj.Position+obj.R0*obj.Corners;
            obj.h=h;

            obj.InitVertices=[(obj.DrawingCorners+[0; 0; -h]) (obj.DrawingCorners+[0; 0; h])];
            obj.w=zeros(3,1);
            obj.RotAxis=[0;0;1];
            obj.AddRot=zeros(3,1);
        end

        function obj = UpdateDeformation(obj, deformation)
            % Update
            obj.Deformation=deformation;
        end

        function obj= UpdateFullOrientation(obj)
            % Compute the corresponding Rotation matrix
            R1=QuaternionToRotationMatrix(obj.q);
            wz=(pi/4*obj.Deformation)*obj.direction*[0;0;1]; % Rotate around z axis
            R2=angleAxisToRotationMatrix(wz);
            obj.R0=R1*R2;
            obj.CornerLocs=obj.Position+obj.R0*obj.Corners;
        end

        function obj= UpdateOrientation(obj,dOrientation)
            tempQ=obj.q+dOrientation;
            if norm(tempQ)>0
                obj.q=tempQ/norm(tempQ);
            else
                obj.q=[1;0;0;0];
                disp("Quaternion computed as zero");
            end
        end

        function obj = UpdatePosition(obj, dPos)
            % Update
            obj.Position=obj.Position+dPos;
        end

        function obj = UpdateAxis(obj,x)
            obj.RotAxis=[0;0;1]+x;
            obj.AddRot=x;
        end

        function obj=DrawNode(obj)
            c=1;
            NodeVerts = obj.Position + obj.R0*(obj.InitVertices*c);
            %             fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
            sides=[1 2 14 13;2 3 15 14; 3 4 16 15; 4 5 17 16; 5 6 18 17; 6 7 19 18; 7 8 20 19; 8 1 13 20];
            %             bottom=[2 3 4 11; 4 5 6 12; 6 7 8 9; 8 1 2 10;9 10 11 12];
            bottom=[1 2 10 9; 2 3 10 10; 3 4 11 10; 4 5 11 11; 5 6 12 11; 6 7 12 12; 7 8 9 12; 8 1 9 9; 9 10 11 12];
            top=12*ones(size(bottom))+bottom;
            fac = [sides;bottom;top];
            L=length(fac(:,1));
            FCD=0.5*ones(L,3);
            obj.ptc=patch('Vertices',NodeVerts','Faces',fac,...
                'FaceColor','flat','FaceVertexCData',FCD);
        end

        function UpdatePlot(obj)
            NodeVerts = obj.Position + obj.R0*obj.InitVertices;
            obj.ptc.Vertices=NodeVerts';
        end

        function SlimModeUpdate(obj) % Updates the node appearance to clip nodes according to the existence of neighbors
            if(obj.SlimMode)
                endPoints=obj.NodeSize.*obj.NeighborExists+1*(1-obj.NeighborExists);
                invEp=(obj.GridL-endPoints).*obj.NeighborExists+endPoints.*(1-obj.NeighborExists); 
            else
                endPoints=obj.NodeSize.*obj.NeighborExists+max(obj.NodeSize)*(1-obj.NeighborExists);
                invEp=(obj.GridL-endPoints);
            end
                    obj.DrawingCorners=[-endPoints(1) -invEp(1) 0; -endPoints(1) invEp(1) 0;...
                    -invEp(2) endPoints(2) 0; invEp(2) endPoints(2) 0;...
                    endPoints(3) invEp(3) 0; endPoints(3) -invEp(3) 0;...
                    invEp(4) -endPoints(4) 0;-invEp(4) -endPoints(4) 0;...
                    -1 -1 0;-1 1 0;1 1 0;1 -1 0;]';

            obj.CornerLocs=obj.Position+obj.R0*obj.Corners;
            obj.InitVertices=[(obj.DrawingCorners+[0; 0; -obj.h]) (obj.DrawingCorners+[0; 0; obj.h])];
            obj.UpdatePlot();
        end
    end
end


function R = angleAxisToRotationMatrix(angleAxis)
% % Normalize the input angle-axis vector
% angleAxis = angleAxis / norm(angleAxis);

% Extract the angle and axis components
angle = norm(angleAxis);
if angle>0
    axis = angleAxis / angle;
else
    axis = zeros(1,3);
end

K=[0 -axis(3) axis(2); axis(3) 0 -axis(1); -axis(2) axis(1) 0];
R=eye(3) + sin(angle)*K + (1-cos(angle))*K*K;

end

function R=QuaternionToRotationMatrix(Q)
 % Extract the values from Q
    q0 = Q(1);
    q1 = Q(2);
    q2 = Q(3);
    q3 = Q(4);
    
    % First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1;
    r01 = 2 * (q1 * q2 - q0 * q3);
    r02 = 2 * (q1 * q3 + q0 * q2);
    
    % Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3);
    r11 = 2 * (q0 * q0 + q2 * q2) - 1;
    r12 = 2 * (q2 * q3 - q0 * q1);
    
    % Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2);
    r21 = 2 * (q2 * q3 + q0 * q1);
    r22 = 2 * (q0 * q0 + q3 * q3) - 1;
    
    % 3x3 rotation matrix
    R = [r00, r01, r02;...
        r10, r11, r12;...
        r20, r21, r22];
end