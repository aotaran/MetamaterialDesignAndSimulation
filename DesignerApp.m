classdef DesignerApp < handle
    properties
        % Structural properties
        L            % rest length of cell sides
        nRow
        nCol
        CellMatrix
        NodeMatrix
        H_EdgeMatrix
        V_EdgeMatrix
        Nodes
        HEdges
        VEdges

        DefaultEdgeWidth
        DefaultEdgeLength

        Joints

        EdgeL
        cDef

        Def         % deformation matrix
        MotorLocs   % motor locations
        TempEdgeLoc
        top
        Motors
        NodeThickness

        % UI related

        Mode % 1: Building 2:Selecting Joints 3: Moving Joints 4: Check deformation 5: Add Motors 6: Control Motors
        ModeLabels

        GlobalDeformation
        MouseLoc
        GroundedCellLoc

        % Active Node deformation
        NumActiveNodes
        ActiveNodeNums
        ActiveNodeLocs
        DeformationRate
        DeformGradient
        DeformHessian
        new_Deformations

        CDGradient % Corner distance gradient
        CDHessian % Corner distance hessian

        QOGradient % Quaternion based orientation Gradient
        QOHessian % Quaternion based orientation Hessian
        OrientationThreshold

        % UI Elements
        SetStructureButton
        LayJointButton
        BackButton
        ConfirmButton
        SaveButton
        LoadButton
        PlaceMotorButton
        SimulateMotorButton

        SimulationWidgets
        SimulationSetButton
        SimulationRunButton

        DeformAmps
        DeformAmps2

        uifig
        fig
        Buttons
        SlimModeCheckBox
        LastCellButtonLog
        LinkAdjustment

        UpperPanel
        LowerPanel

        Palette
    end

    methods
        function obj = DesignerApp()
            % Constructor
            obj.L = 20;            % rest length of cell sides
            obj.EdgeL = obj.L*sqrt(2);
            obj.cDef=1;
            obj.nRow=8;
            obj.nCol=8;
            obj.Mode=1;
            obj.Nodes=cell(obj.nRow+1,obj.nCol+1);
            obj.HEdges=cell(obj.nRow+1,obj.nCol);
            obj.VEdges=cell(obj.nRow,obj.nCol+1);
            obj.DefaultEdgeWidth=0.8;
            obj.DefaultEdgeLength=2;
            obj.LastCellButtonLog=0;
            obj.GroundedCellLoc=[1 1];
            obj.LinkAdjustment.created=false;
            % Further setup
            obj.ZeroStructure();
            obj.OrientationThreshold=1e-3;


            obj.Palette.orange=[1 0.6 0];
            obj.Palette.lightgray=[0.95 0.95 0.95];
            obj.Palette.gray=[0.85 0.85 0.85];
            obj.Palette.cyan=[0 0.5 0.5];
            obj.Palette.blue=[.1 .1 1];
            obj.Palette.green=[.1 1 .1];
            obj.Palette.lightblue=[.9 .9 1];
            % obj.Palette.lightblue=[.9 .9 1];

            obj.top=800;
            obj.uifig = uifigure('Name','User Interface');
            obj.NodeThickness=5;
            obj.uifig.Position = [1000 50 550 obj.top+60];
            obj.fig=figure('Name','Metamaterial Designer','Position',[0 50 1000 obj.top+10]);
            hold on;
            axis equal off    % make the axes equal and invisible

            obj.Def=ones(obj.nRow+1,obj.nCol+1);


            %% Initializing UI Elements

            % Slider to be generated
            obj.CreateModeLabels();
            obj.SetMode(1);

            obj.GlobalDeformation.slider = uislider(obj.uifig, 'Value', 0, 'Limits', [-1 1],...
                "ValueChangedFcn",@(src,event)updateParam(obj,src,event));
            obj.GlobalDeformation.slider.Position=[50 270 200 3];
            obj.GlobalDeformation.slider.Enable=false;
            obj.GlobalDeformation.label = uilabel(obj.uifig,"Position",[50 280 150 32]);
            obj.GlobalDeformation.label.Text = "Global deformation";
            obj.GlobalDeformation.value=0;

            % Set Structure Button

            obj.SetStructureButton=uibutton(obj.uifig, 'Position', [225 80 120 50],...
                "Text","Set Structure", ...
                "ButtonPushedFcn", @(btn,event) SetStructure(obj,btn,event));
            for i=1:obj.nRow
                for j=1:obj.nCol
                    obj.Buttons{i,j} = uibutton(obj.uifig,'state',...
                        'Text','' ,...
                        'Value',false, ...
                        'Position', [20*j 20*i 20 20], ...
                        'FontSize', 16, ...
                        'FontWeight','bold','Tag',['C' num2str(i) num2str(j)]);
                    obj.Buttons{i,j}.ValueChangedFcn = @(btn,evt)stateButtonPressed(obj,btn,evt);
                end
            end

            obj.LinkAdjustment.Button=uibutton(obj.uifig,'state','Position', [150 200 80 30],...
                "Text","Links");
            obj.LinkAdjustment.Button.ValueChangedFcn= @(btn,event) AdjustLinks(obj,btn,event);
            obj.LinkAdjustment.Button.Enable=false;
            obj.LinkAdjustment.PanelVisible=false;

            % Lay Rotating Joint Button
            obj.LayJointButton=uibutton(obj.uifig, 'Position', [225 140 120 50],...
                "Text","Select Rotary Joint", ...
                "ButtonPushedFcn", @(btn,event) SelectJoint(obj,btn,event));
            obj.LayJointButton.Enable=false;

            % Place Motors Button
            obj.PlaceMotorButton=uibutton(obj.uifig, 'Position', [350 140 100 50],...
                "Text","Place Motor", ...
                "ButtonPushedFcn", @(btn,event) MotorPlacement(obj,btn,event));
            obj.PlaceMotorButton.Enable=false;

            % Confirm Button
            obj.ConfirmButton=uibutton(obj.uifig, 'Position', [455 140 80 50],...
                "Text","Confirm", ...
                "ButtonPushedFcn", @(btn,event) ConfirmFunc(obj,btn,event));
            obj.ConfirmButton.Enable=false;

            % Save Button
            obj.SaveButton=uibutton(obj.uifig, 'Position', [455 80 80 50],...
                "Text","Save Config", ...
                "ButtonPushedFcn", @(btn,event) SaveButtonFunction(obj,btn,event));
            obj.SaveButton.Enable=false;

            % Load Button
            obj.LoadButton=uibutton(obj.uifig, 'Position', [455 20 80 50],...
                "Text","Load Config", ...
                "ButtonPushedFcn", @(btn,event) LoadButtonFunction(obj,btn,event));
            obj.LoadButton.Enable=true;

            % Back Button
            obj.BackButton=uibutton(obj.uifig, 'Position', [225 20 120 50],...
                "Text","Back", ...
                "ButtonPushedFcn", @(btn,event) BackButtonPressed(obj,btn,event));

            % Simulate Motor signals Button
            obj.SimulateMotorButton=uibutton(obj.uifig, 'Position', [350 80 100 50],...
                "Text","Simulate Signals", ...
                "ButtonPushedFcn", @(btn,event) SimulateSignals(obj,btn,event));
            obj.SimulateMotorButton.Enable=false;

            obj.SlimModeCheckBox=uicheckbox(obj.uifig,'Position', [50 205 80 20], ...
                'Text','Slim Mode','Value',false, ...
                'ValueChangedFcn',@(src,event)SlimModeFunction(obj,src,event));

        end


        %% Button or slider callbacks

        function ZeroStructure(obj)

            obj.CellMatrix=zeros(obj.nRow,obj.nCol);
            obj.NodeMatrix=zeros(obj.nRow+1,obj.nCol+1);
            obj.H_EdgeMatrix=zeros(obj.nRow+1,obj.nCol);
            obj.V_EdgeMatrix=zeros(obj.nRow,obj.nCol+1);
            obj.Nodes=cell(obj.nRow+1,obj.nCol+1);
            obj.HEdges=cell(obj.nRow+1,obj.nCol);
            obj.VEdges=cell(obj.nRow,obj.nCol+1);
            obj.Joints={};
            obj.Motors={};
            obj.ActiveNodeLocs=[];
            obj.ActiveNodeNums=zeros(obj.nRow+1,obj.nCol+1);
            obj.NumActiveNodes=0;
        end

        function SetStructure(obj, btnHandle, event)
            % Calls the initialization functions for the structure
            obj.SetMode(1);
            obj.ZeroStructure();
            for i=1:obj.nRow
                for j=1:obj.nCol
                    if(obj.Buttons{i,j}.Value)
                        obj.CellMatrix(i,j)=1;
                    else
                        obj.CellMatrix(i,j)=0;
                    end
                end
            end
            obj.GetRigidMat();
            obj.InitializeStructure();
            obj.GenerateLinkAdjustmentPanel();
            xlim([-1 obj.nCol+1]*obj.L*2);
            ylim([-1 obj.nRow+1]*obj.L*2);

            zlim([-1 1]*obj.nCol*obj.L);
            view(30,30)
            obj.LayJointButton.Enable=true;
            obj.GlobalDeformation.slider.Enable=true;
            obj.ConfirmButton.Enable=true;
            obj.LinkAdjustment.Button.Enable=true;
        end

        % Link adjustment panel functions
        %                     % create link buttons and patches for each buttons and nodes
        %                     obj.LinkAdjustment.Button
        %                 end

        function GenerateLinkAdjustmentPanel(obj)
            % obj.AdjustLinks=cell(obj.nRow+1,obj.nCol+1);
            if(obj.LinkAdjustment.created)
                obj.CleanLinkAdjustmentPanel();
                obj.LinkAdjustment.created=false;
            else
                obj.LinkAdjustment.WidthControl.slider = uislider(obj.uifig, 'Value', 0.8, 'Limits', [0.39 1.21],...
                    "ValueChangedFcn",@(src,event)UpdateEdgeWidth(obj,src,event));
                obj.LinkAdjustment.WidthControl.slider.Position=[300 280 200 3];
                obj.LinkAdjustment.WidthControl.label = uilabel(obj.uifig,"Position",[300 280 50 30]);
                obj.LinkAdjustment.WidthControl.label.Text = "Width";
                obj.LinkAdjustment.WidthControl.slider.MajorTicks=[0.4 0.8 1.2];
                obj.LinkAdjustment.WidthControl.slider.MinorTicks=[];
                obj.LinkAdjustment.WidthControl.slider.Visible=false;
                obj.LinkAdjustment.WidthControl.label.Visible=false;

                obj.LinkAdjustment.LengthControl.slider = uislider(obj.uifig, 'Value', 2, 'Limits', [-0.01 8.01],...
                    "ValueChangedFcn",@(src,event)UpdateEdgeLength(obj,src,event));
                obj.LinkAdjustment.LengthControl.slider.Position=[300 225 200 3];
                obj.LinkAdjustment.LengthControl.label = uilabel(obj.uifig,"Position",[300 225 50 30]);
                obj.LinkAdjustment.LengthControl.label.Text = "Length";
                obj.LinkAdjustment.LengthControl.slider.MajorTicks=0:0.5:8;
                obj.LinkAdjustment.LengthControl.slider.MinorTicks=[];
                obj.LinkAdjustment.LengthControl.slider.Visible=false;
                obj.LinkAdjustment.LengthControl.label.Visible=false;

                obj.LinkAdjustment.SelectedEdge.label = uilabel(obj.uifig,"Position",[320 300 150 20]);
                obj.LinkAdjustment.SelectedEdge.label.Interpreter='latex';
            end


            obj.LinkAdjustment.NodeUI=cell(obj.nRow+1,obj.nCol+1);
            obj.LinkAdjustment.HEdgeUI=cell(obj.nRow+1,obj.nCol);
            obj.LinkAdjustment.VEdgeUI=cell(obj.nRow,obj.nCol+1);

            for i = 1:size(obj.H_EdgeMatrix,1)
                for j = 1:size(obj.H_EdgeMatrix,2)
                    if(obj.H_EdgeMatrix(i,j)==1)
                        obj.CreateEdgeUIElement(i,j,'h');
                    end
                end
            end

            %Set up vertical flexible links

            for i = 1:size(obj.V_EdgeMatrix,1)
                for j = 1:size(obj.V_EdgeMatrix,2)
                    if(obj.V_EdgeMatrix(i,j)==1)
                        obj.CreateEdgeUIElement(i,j,'v');
                    end
                end
            end

            for i = 1:obj.nRow+1
                for j = 1:obj.nCol+1
                    if(obj.NodeMatrix(i,j)==1)
                        obj.CreateNodeUIElement(i,j);
                    end
                end
            end
            Loc=obj.ActiveNodeLocs(1,:);
            obj.LinkAdjustment.SelectedEdge.Loc = Loc;
            obj.LinkAdjustment.SelectedEdge.dir = 'h';
            EdgeName=['$' upper(obj.LinkAdjustment.SelectedEdge.dir) '_{' num2str(Loc(1)) ',' num2str(Loc(2)) '}$'];
            obj.LinkAdjustment.SelectedEdge.label.Text = ['Selected edge: ' EdgeName];
            obj.LinkAdjustment.SelectedEdge.label.Visible=false;
            obj.LinkAdjustment.HEdgeUI{Loc(1),Loc(2)}.button.BackgroundColor=obj.Palette.orange;
            obj.LinkAdjustment.created=true;
        end

        function CleanLinkAdjustmentPanel(obj)
            for i = 1:size(obj.H_EdgeMatrix,1)
                for j = 1:size(obj.H_EdgeMatrix,2)
                    obj.DeleteEdgeUIElement(i,j,'h');
                end
            end

            for i = 1:size(obj.V_EdgeMatrix,1)
                for j = 1:size(obj.V_EdgeMatrix,2)
                    obj.DeleteEdgeUIElement(i,j,'v');
                end
            end

            for i = 1:obj.nRow+1
                for j = 1:obj.nCol+1
                    obj.DeleteNodeUIElement(i,j);
                end
            end
        end

        function CreateNodeUIElement(obj,i,j)
            ImagePosition=[j*60-40 i*60+260 35 35];
            ImageSource="NodeImage.png";
            TagName=['N' num2str(i) num2str(j)];
            LabelName=sprintf('$N_{%i,%i}$',i,j);
            if(isempty(obj.LinkAdjustment.NodeUI{i,j}))
                obj.LinkAdjustment.NodeUI{i,j}.image=uiimage(obj.uifig);
                obj.LinkAdjustment.NodeUI{i,j}.label=uilabel(obj.uifig);
            end
            obj.LinkAdjustment.NodeUI{i,j}.image.Position = ImagePosition;
            obj.LinkAdjustment.NodeUI{i,j}.image.ImageSource = ImageSource;
            obj.LinkAdjustment.NodeUI{i,j}.image.Tag=TagName;
            obj.LinkAdjustment.NodeUI{i,j}.label.Position = ImagePosition+[5 0 0 0];
            obj.LinkAdjustment.NodeUI{i,j}.label.Text=LabelName;
            obj.LinkAdjustment.NodeUI{i,j}.label.Interpreter='latex';
            obj.LinkAdjustment.NodeUI{i,j}.image.Visible=false;
            obj.LinkAdjustment.NodeUI{i,j}.label.Visible=false;
        end

        function DeleteNodeUIElement(obj,i,j)
            if(~isempty(obj.LinkAdjustment.NodeUI{i,j}))
                delete(obj.LinkAdjustment.NodeUI{i,j}.image);
                delete(obj.LinkAdjustment.NodeUI{i,j}.label);
            end
        end

        function CreateEdgeUIElement(obj,i,j,dir)
            if(dir=='v')
                UIPosition=[j*60-27 i*60+295 10 25];
                LabelPosition=[UIPosition(1)+8 UIPosition(2)+2 30 20];
                TagName=['V' num2str(i) num2str(j)];
                LabelName=sprintf('$V_{%i,%i}$',i,j);
                ThisEdgeUI=obj.LinkAdjustment.VEdgeUI{i,j};
            else
                UIPosition=[j*60-5 i*60+273 25 10];
                LabelPosition=[UIPosition(1) UIPosition(2)+6 30 20];
                TagName=['H' num2str(i) num2str(j)];
                LabelName=sprintf('$H_{%i,%i}$',i,j);
                ThisEdgeUI=obj.LinkAdjustment.HEdgeUI{i,j};
            end

            if(isempty(ThisEdgeUI))
                ThisEdgeUI.button=uibutton(obj.uifig);
                ThisEdgeUI.label=uilabel(obj.uifig);
            end

            ThisEdgeUI.button;
            ThisEdgeUI.button.Text='';
            ThisEdgeUI.button.Position = UIPosition;
            ThisEdgeUI.button.Tag=TagName;
            ThisEdgeUI.button.ButtonPushedFcn =@(btn,evt)SelectEdge(obj,btn,evt);
            ThisEdgeUI.button.BackgroundColor  = obj.Palette.cyan;

            ThisEdgeUI.label.Position = LabelPosition;
            ThisEdgeUI.label.Text=LabelName;
            ThisEdgeUI.label.Interpreter='latex';
            ThisEdgeUI.created=true;
            ThisEdgeUI.button.Visible=false;
            ThisEdgeUI.label.Visible=false;

            if(dir=='v')
                obj.LinkAdjustment.VEdgeUI{i,j}=ThisEdgeUI;

            else
                obj.LinkAdjustment.HEdgeUI{i,j}=ThisEdgeUI;
            end

        end

        function DeleteEdgeUIElement(obj,i,j,dir)
            if(dir=='v')
                ThisEdgeUI=obj.LinkAdjustment.VEdgeUI{i,j};

            else
                ThisEdgeUI=obj.LinkAdjustment.HEdgeUI{i,j};
            end

            if(~isempty(ThisEdgeUI))
                delete(ThisEdgeUI.button);
                delete(ThisEdgeUI.label);
            end
        end

        function ToggleLinkAdjustmentPanel(obj,ToggleOnOff)
            if (ToggleOnOff)
                obj.HideDeformSliders();
                obj.HideMotorSliders();
                obj.LinkAdjustment.Button.Value=true;
                obj.LinkAdjustment.Button.BackgroundColor = obj.Palette.lightblue;
            else
                obj.LinkAdjustment.Button.Value=false;
                obj.LinkAdjustment.Button.BackgroundColor = obj.Palette.lightgray;
                if(obj.Mode>=2 && obj.Mode<5)
                    obj.ShowDeformSliders();
                elseif(obj.Mode>4)
                    obj.ShowMotorSliders();
                end
            end
            obj.LinkAdjustment.WidthControl.label.Visible=ToggleOnOff;
            obj.LinkAdjustment.WidthControl.slider.Visible=ToggleOnOff;
            obj.LinkAdjustment.LengthControl.label.Visible=ToggleOnOff;
            obj.LinkAdjustment.LengthControl.slider.Visible=ToggleOnOff;
            obj.LinkAdjustment.SelectedEdge.label.Visible=ToggleOnOff;

            for i = 1:size(obj.H_EdgeMatrix,1)
                for j = 1:size(obj.H_EdgeMatrix,2)
                    if(obj.H_EdgeMatrix(i,j)==1)
                        obj.LinkAdjustment.HEdgeUI{i,j}.button.Visible=ToggleOnOff;
                        obj.LinkAdjustment.HEdgeUI{i,j}.label.Visible=ToggleOnOff;
                        obj.LinkAdjustment.HEdgeUI{i,j}.button.Enable=ToggleOnOff;
                    end
                end
            end

            for i = 1:size(obj.V_EdgeMatrix,1)
                for j = 1:size(obj.V_EdgeMatrix,2)
                    if(obj.V_EdgeMatrix(i,j)==1)
                        obj.LinkAdjustment.VEdgeUI{i,j}.button.Visible=ToggleOnOff;
                        obj.LinkAdjustment.VEdgeUI{i,j}.label.Visible=ToggleOnOff;
                        obj.LinkAdjustment.VEdgeUI{i,j}.button.Enable=ToggleOnOff;
                    end
                end
            end

            for i = 1:obj.nRow+1
                for j = 1:obj.nCol+1
                    if(obj.NodeMatrix(i,j)==1)
                        obj.LinkAdjustment.NodeUI{i,j}.image.Visible=ToggleOnOff;
                        obj.LinkAdjustment.NodeUI{i,j}.label.Visible=ToggleOnOff;
                    end
                end
            end

        end

        function AdjustLinks(obj, btnHandle, event)
            % If there are axes, make them disabled and invisible and show
            % the Link adjustment panel
            obj.ToggleLinkAdjustmentPanel(btnHandle.Value);
        end

        function SelectJoint(obj, btnHandle, event)
            obj.SetMode(2);
            obj.ToggleLinkAdjustmentPanel(false);
            obj.GlobalDeformation.slider.Enable=false;
            obj.SetStructureButton.Enable=false;
            obj.MoveActiveNodes(1);
            MoveCameraToDesired(0,90); % To view from above

            obj.MouseLoc=ginput(1); % Receive mouse input
            SelectedPoint=(obj.MouseLoc+obj.EdgeL/2)/obj.EdgeL;
            Lside=obj.L*sqrt(2);

            dif=abs(SelectedPoint-round(SelectedPoint)); % Check if the selected point is on vertical or horizontal direction
            if(dif(1)<dif(2))
                SelectedDirection='v';
            else
                SelectedDirection='h';
            end
            nJoints=length(obj.Joints);
            MakeJoint=false;

            % Prepare joint
            if(SelectedDirection=='v')
                index_j=ceil(SelectedPoint(2));
                index_i=round(SelectedPoint(1));
                InStructure=false;
                if(index_i>0 && index_j>0 && index_i<obj.nRow && index_j<obj.nCol)
                    InStructure=obj.H_EdgeMatrix(index_j,index_i)==1;
                end
                % Check if the selected point in the structure
                if(~InStructure)
                    disp("Selected point is out of structure.")
                else
                    indexDec=index_j-1;
                    while(indexDec>0 && obj.H_EdgeMatrix(indexDec,index_i)==1 && obj.CellMatrix(indexDec,index_i)==1) %Extending inwards
                        indexDec=indexDec-1;
                    end
                    indexInc=index_j+1;
                    while(indexInc<obj.nRow && obj.H_EdgeMatrix(indexInc,index_i)==1 && obj.CellMatrix(indexInc-1,index_i)==1) %Extending outwards
                        indexInc=indexInc+1;
                    end
                    indexInc=indexInc-1;
                    indexDec=indexDec+1;
                    minY=(indexDec-1.5)*Lside;
                    maxY=(indexInc-0.5)*Lside;
                    Xval=(index_i-0.5)*Lside;

                    p1=[Xval minY 0];
                    p2=[Xval maxY 0];
                    [X,Y,Z] = cylinder;
                    R_cylinder=obj.NodeThickness+1;

                    JointLevel=index_i;
                    JointSpan=[indexDec indexInc];
                    % Check if selected joint is overlapping with a previous one:
                    % either on top of the joint or crossing
                    ValidJoint=true;
                    for i=1:nJoints
                        ith_Joint=obj.Joints{i};
                        if (ith_Joint.direction==SelectedDirection)
                            if(ith_Joint.Level==JointLevel && ith_Joint.span(1) == JointSpan(1))
                                ValidJoint = false;
                            end
                        else
                            if(ith_Joint.Level >= JointSpan(1) && ith_Joint.Level < JointSpan(2) && JointLevel >= ith_Joint.span(1) && JointLevel < ith_Joint.span(2))
                                ValidJoint = false;
                            end
                        end
                    end

                    if(~ValidJoint)
                        disp("Selected vertical joint is invalid. Overlapping with another joint");
                    else
                        disp("Vertical joint created");
                        MakeJoint=true;
                    end
                end

            else % If the joint is horizontal
                index_i=ceil(SelectedPoint(1));
                index_j=round(SelectedPoint(2));
                InStructure=false;
                if(index_i>0 && index_j>0 && index_i<obj.nRow && index_j<obj.nCol)
                    InStructure=obj.V_EdgeMatrix(index_j,index_i)==1;
                end
                if(~InStructure)
                    disp("Selected point is out of structure.")
                else
                    indexDec=index_i-1;
                    while(indexDec>0 && obj.V_EdgeMatrix(index_j,indexDec)==1 && obj.CellMatrix(index_j,indexDec)==1)
                        indexDec=indexDec-1;
                    end
                    indexInc=index_i+1;
                    while(indexInc<obj.nCol && obj.V_EdgeMatrix(index_j,indexInc)==1 && obj.CellMatrix(index_j,indexInc-1)==1)
                        indexInc=indexInc+1;
                    end
                    indexInc=indexInc-1;
                    indexDec=indexDec+1;
                    minX=(indexDec-1.5)*Lside;
                    maxX=(indexInc-0.5)*Lside;
                    Yval=(index_j-0.5)*Lside;

                    p1=[minX Yval 0];
                    p2=[maxX Yval 0];
                    [X,Y,Z] = cylinder;
                    R_cylinder=obj.NodeThickness+1;

                    JointLevel=index_j;
                    JointSpan=[indexDec indexInc];
                    % Check if selected joint is overlapping with a previous one:
                    % either on top of the joint or crossing
                    ValidJoint=true;
                    for i=1:nJoints
                        ith_Joint=obj.Joints{i};
                        if (ith_Joint.direction==SelectedDirection)
                            if(ith_Joint.Level==JointLevel && ith_Joint.span(1) == JointSpan(1))
                                ValidJoint = false;
                            end
                        else
                            if(ith_Joint.Level >= JointSpan(1) && ith_Joint.Level < JointSpan(2) && JointLevel >= ith_Joint.span(1) && JointLevel < ith_Joint.span(2))
                                ValidJoint = false;
                            end
                        end
                    end
                    if(~ValidJoint)
                        disp("Selected horizontal joint is invalid. Overlapping with another joint");
                    else
                        MakeJoint=true;
                        disp("Horizontal joint selected");
                    end
                end
            end
            if(MakeJoint)
                if(SelectedDirection=='v')
                    obj.Joints{nJoints+1}.cylinder=surf(X*R_cylinder+Xval,Z*(maxY-minY)+minY,Y*R_cylinder,'FaceColor','b');
                else
                    obj.Joints{nJoints+1}.cylinder=surf(Z*(maxX-minX)+minX,X*R_cylinder+Yval,Y*R_cylinder,'FaceColor','r');
                end
                obj.Joints{nJoints+1}.span=JointSpan;
                obj.Joints{nJoints+1}.Level=JointLevel;
                obj.Joints{nJoints+1}.direction=SelectedDirection;
                obj.Joints{nJoints+1}.BendingAngle=0;
                SliderPosX=20+250*floor((nJoints)/8);
                SliderPosY=obj.top-65*floor(mod(nJoints,8));
                obj.Joints{nJoints+1}.DeformSlider=uislider(obj.uifig, 'Value', 0, 'Limits', [-90 90],...
                    'Position',[SliderPosX SliderPosY 200 3],'Tag',['J' num2str(nJoints+1)],...
                    "ValueChangedFcn",@(src,event)UpdateJoints(obj,src,event));
                obj.Joints{nJoints+1}.AxisLabel=uilabel(obj.uifig,'Text',['Axis ' num2str(nJoints+1)],...
                    'Position',[SliderPosX SliderPosY+10 200 20]);
                obj.Joints{nJoints+1}.DeformSlider.MajorTicks =-90:30:90;
                obj.Joints{nJoints+1}.DeformSlider.Enable=false;
            end
        end

        function MotorPlacement(obj, btnHandle, event)
            if(obj.Mode<5)
                obj.GlobalDeformation.slider.Enable=false;
                obj.SetStructureButton.Enable=false;
                obj.SetMode(5);
                obj.HideDeformSliders();
            end
            if(obj.Mode==6)
                obj.SetMode(5);
                if(~isempty(obj.Motors))
                    for i=1:length(obj.Motors)
                        obj.Motors{i}.ActSlider.Enable=false;
                    end
                end
            end
            obj.MoveActiveNodes(0);
            MoveCameraToDesired(0,90);
            obj.MouseLoc=ginput(1); %
            temp=round(obj.MouseLoc/obj.L)/2;
            i=temp(2)+1; j=temp(1)+1;
            if(obj.NodeMatrix(floor(i),floor(j))>0 && obj.NodeMatrix(ceil(i),ceil(j))>0)
                nMotors=length(obj.Motors);
                obj.Motors{nMotors+1}.s=scatter3(temp(1)*2*obj.L,temp(2)*2*obj.L,0,'red','filled');
                obj.Motors{nMotors+1}.s.SizeData=200;
                obj.Motors{nMotors+1}.Loc=[i j];
                obj.Motors{nMotors+1}.ORNode1=floor([i j]);
                obj.Motors{nMotors+1}.ORNode2=ceil([i j]);
                if(floor(i)==ceil(i))
                    obj.Motors{nMotors+1}.direction='h';
                else
                    obj.Motors{nMotors+1}.direction='v';
                end

                SliderPosX=20+250*floor((nMotors)/8);
                SliderPosY=obj.top-65*floor(mod(nMotors,8));
                obj.Motors{nMotors+1}.ActSlider=uislider(obj.uifig, 'Value', 0, 'Limits', [-90 90],'Tag',['M' num2str(nMotors+1)],...
                    "ValueChangingFcn",@(src,event)UpdateMotors(obj,src,event));
                obj.Motors{nMotors+1}.AxisLabel=uilabel(obj.uifig,'Text',['Actuator ' num2str(nMotors+1)]);
                obj.Motors{nMotors+1}.AxisLabel.Position=[SliderPosX SliderPosY+10 200 20];
                obj.Motors{nMotors+1}.ActSlider.Position=[SliderPosX SliderPosY 200 3];
                obj.Motors{nMotors+1}.ActSlider.MajorTicks =-90:30:90;
                obj.Motors{nMotors+1}.ActSlider.Enable=false;
                obj.Motors{nMotors+1}.ActValue=0;
            end
        end

        function SimulateSignals(obj, btnHandle, event)
            if(~isempty(obj.SimulationWidgets))
                for i=1:length(obj.SimulationWidgets)
                    delete(obj.SimulationWidgets{i}.Axis);
                    delete(obj.SimulationWidgets{i}.AmplitudeSlider);
                    delete(obj.SimulationWidgets{i}.PhaseSlider);
                    delete(obj.SimulationWidgets{i}.AmpLabel);
                    delete(obj.SimulationWidgets{i}.PhaseLabel);
                end
                delete(obj.SimulationSetButton);
                delete(obj.SimulationRunButton);
            end
            X=0:0.1:10;
            for i=1:length(obj.Motors)
                obj.SimulationWidgets{i}.Axis = uiaxes(obj.uifig,'Position',[180 obj.top-80-140*i 300 100],'Box','on');
                obj.SimulationWidgets{i}.SignalX=X;
                obj.SimulationWidgets{i}.SignalY=zeros(size(X));
                obj.SimulationWidgets{i}.Signalplot = plot(obj.SimulationWidgets{i}.Axis,obj.SimulationWidgets{i}.SignalX,obj.SimulationWidgets{i}.SignalY,'Color','b');

                obj.SimulationWidgets{i}.AmpLabel=uilabel(obj.uifig,'Position',[20 obj.top-10-140*i 70 20],'Text',['Amplitude ' num2str(i)]);
                obj.SimulationWidgets{i}.AmplitudeSlider = uislider(obj.uifig,'Position',[100 obj.top-140*i 60 3], 'Value', 0, 'Limits', [0 1]);

                obj.SimulationWidgets{i}.PhaseLabel=uilabel(obj.uifig,'Position',[20 obj.top-50-140*i 70 20],'Text',['Phase ' num2str(i)]);
                obj.SimulationWidgets{i}.PhaseSlider = uislider(obj.uifig,'Position',[100 obj.top-40-140*i 60 3],'Value', 0, 'Limits', [0 2*pi]);
            end
            obj.SimulationSetButton = uibutton(obj.uifig, 'Position',[20 obj.top-80-140*i 50 20],...
                "Text","Set", "ButtonPushedFcn", @(btn,event) SetButtonFunc(obj,btn,event));
            obj.SimulationRunButton = uibutton(obj.uifig, 'Position',[80 obj.top-80-140*i 50 20],...
                "Text","Run", "ButtonPushedFcn", @(btn,event) RunButtonFunc(obj,btn,event));
        end

        function ConfirmFunc(obj, btnHandle, event)
            % Add to joints
            obj.ToggleLinkAdjustmentPanel(false);
            if obj.Mode==5
                obj.SetMode(6);
                for i=1:length(obj.Motors)
                    obj.Motors{i}.ActSlider.Enable=true;
                    obj.SimulateMotorButton.Enable=true;
                end
            elseif obj.Mode==3
                obj.SetMode(4);
                for i=1:length(obj.Joints)
                    obj.Joints{i}.DeformSlider.Enable=false;
                    obj.GlobalDeformation.slider.Enable=true;
                    obj.PlaceMotorButton.Enable=true;
                    obj.SaveButton.Enable=true;
                    obj.Joints{i}.cylinder.Visible=false;
                    % delete(obj.Joints{i}.cylinder)
                end
            elseif obj.Mode==2
                obj.SetMode(3);
                for i=1:length(obj.Joints)
                    obj.Joints{i}.DeformSlider.Enable=true;
                    obj.Joints{i}.cylinder.Visible=false;
                end
            end
        end

        function SaveButtonFunction(obj,btnHandle, event)
            disp("Function currently under construction.");
        end

        function LoadButtonFunction(obj,btnHandle, event)
            disp("Function currently under construction.");
        end

        function SetButtonFunc(obj, btnHandle, event)
            X=0:0.1:10;
            for i=1:length(obj.Motors)
                A=obj.SimulationWidgets{i}.AmplitudeSlider.Value;
                Phase=obj.SimulationWidgets{i}.PhaseSlider.Value;
                Y=A*sin(X+Phase);
                obj.SimulationWidgets{i}.Signalplot.YData=Y;
            end

        end

        function RunButtonFunc(obj, btnHandle, event)
            for i=1:length(obj.SimulationWidgets{1}.Signalplot.YData)
                obj.DeformationEstimation();
                obj.MoveActiveNodes(obj.DeformationRate);
            end
        end

        function BackButtonPressed(obj, btnHandle, event)
            if(obj.Mode>1)
                obj.SetMode(1);
                obj.GlobalDeformation.slider.Enable=true;
                obj.SetStructureButton.Enable=true;
                obj.LayJointButton.Enable=false;
                obj.PlaceMotorButton.Enable=false;
                obj.SimulateMotorButton.Enable=false;
                % Reset Joints
                if (~isempty(obj.Joints))
                    for i=1:length(obj.Joints)
                        delete(obj.Joints{i}.DeformSlider);
                        delete(obj.Joints{i}.AxisLabel);
                        delete(obj.Joints{i}.cylinder);
                    end
                    obj.Joints={};
                end

                % Reset Actuators
                if (~isempty(obj.Motors))
                    for i=1:length(obj.Motors)
                        delete(obj.Motors{i}.ActSlider);
                        delete(obj.Motors{i}.AxisLabel);
                        delete(obj.Motors{i}.s);
                    end
                    obj.Motors={};
                end
            end
        end

        function stateButtonPressed(obj, btnHandle, event)
            ClickDelay=now-obj.LastCellButtonLog;
            if event.Value
                % Button is TRUE: set color to default
                btnHandle.BackgroundColor = obj.Palette.blue;
                %btnHandle.BackgroundColor = [.96 .96 .96]; % default color
            else
                if(ClickDelay<3e-6) % Double clicked
                    btnHandle.Value=true;
                    PreviousGroundedButton = obj.Buttons{obj.GroundedCellLoc(1),obj.GroundedCellLoc(2)};
                    if(PreviousGroundedButton.Value)
                        PreviousGroundedButton.BackgroundColor = obj.Palette.blue;
                    end
                    obj.GroundedCellLoc=[str2num(btnHandle.Tag(2)) str2num(btnHandle.Tag(3))];
                    btnHandle.BackgroundColor = obj.Palette.green;
                else
                    % Button is FALSE
                    btnHandle.BackgroundColor = obj.Palette.lightgray;
                end
            end
            obj.LastCellButtonLog=now;
        end

        function updateParam(obj,src,event)
            PreviousValue=obj.GlobalDeformation.value;
            NewValue=event.Value;
            obj.GlobalDeformation.value = event.Value;
            obj.GlobalDeformation.slider.Enable=false;
            pause(0.001);
            % dValue=(NewValue-PreviousValue)/10;
            dValue=0.05*sign(NewValue-PreviousValue);

            obj.OrientationThreshold=1e-3;
            for Value=PreviousValue+dValue:dValue:NewValue
                obj.MoveActiveNodes(Value);
            end
            % obj.OrientationThreshold=1e-4;
            % obj.MoveActiveNodes(Value);

            % for T=2:0.4:5
            % obj.OrientationThreshold=10^(-T);
            % obj.MoveActiveNodes(Value);
            % end

            obj.GlobalDeformation.slider.Enable=true;
            pause(0.01);
            obj.GlobalDeformation.value=NewValue;
        end

        function UpdateEdgeWidth(obj,src,event)
            % MajorTicks=0.4:0.4:1.2;
            SelectedValue=round(event.Value/0.4)*0.4;
            src.Value=SelectedValue;

            dir=obj.LinkAdjustment.SelectedEdge.dir;
            Loc=obj.LinkAdjustment.SelectedEdge.Loc;
            if(dir=='v')
                obj.VEdges{Loc(1),Loc(2)}.width=SelectedValue;
                obj.VEdges{Loc(1),Loc(2)}.UpdateStiffness();
            else
                obj.HEdges{Loc(1),Loc(2)}.width=SelectedValue;
                obj.HEdges{Loc(1),Loc(2)}.UpdateStiffness();
            end
        end

        function UpdateEdgeLength(obj,src,event)
            MajorTicks=0.5:0.5:4;
            SelectedValue=round(event.Value/0.5)*0.5;
            src.Value=SelectedValue;

            dir=obj.LinkAdjustment.SelectedEdge.dir;
            Loc=obj.LinkAdjustment.SelectedEdge.Loc;
            if(dir=='v')
                obj.VEdges{Loc(1),Loc(2)}.length=SelectedValue;
                obj.VEdges{Loc(1),Loc(2)}.UpdateStiffness();
            else
                obj.HEdges{Loc(1),Loc(2)}.length=SelectedValue;
                obj.HEdges{Loc(1),Loc(2)}.UpdateStiffness();
            end
        end

        function obj=SelectEdge(obj,btn,evt)
            dir=obj.LinkAdjustment.SelectedEdge.dir;
            Loc=obj.LinkAdjustment.SelectedEdge.Loc;
            if(dir=='v')
                obj.LinkAdjustment.VEdgeUI{Loc(1),Loc(2)}.button.BackgroundColor=obj.Palette.cyan;
            else
                obj.LinkAdjustment.HEdgeUI{Loc(1),Loc(2)}.button.BackgroundColor=obj.Palette.cyan;
            end
            btn.BackgroundColor=obj.Palette.orange;
            NewDir=lower(btn.Tag(1));
            NewLoc=[str2num(btn.Tag(2)) str2num(btn.Tag(3))];

            if(NewDir=='v')
                ThisEdge=obj.VEdges{NewLoc(1),NewLoc(2)};
            else
                ThisEdge=obj.HEdges{NewLoc(1),NewLoc(2)};
            end

            obj.LinkAdjustment.SelectedEdge.dir=NewDir;
            obj.LinkAdjustment.SelectedEdge.Loc=NewLoc;
            obj.LinkAdjustment.LengthControl.slider.Value=ThisEdge.length;
            obj.LinkAdjustment.WidthControl.slider.Value=ThisEdge.width;

            EdgeName=['$' upper(NewDir) '_{' num2str(NewLoc(1)) ',' num2str(NewLoc(2)) '}$'];
            obj.LinkAdjustment.SelectedEdge.label.Text = ['Selected edge: ' EdgeName];
        end

        function obj=UpdateJoints(obj,src,event)
            [X,Y,Z]=cylinder;
            % ** Update the bending amount of the joint **
            % Currently going through all of the joints but could add a tag
            % on the joint or use the name of the joint
            JointNum=str2num(src.Tag(2:end));

            obj.DisableJointSliders();
            pause(0.01);
            % disp("Joints Disabled")

            %             for JointNum=1:length(obj.Joints)
            ThisJoint=obj.Joints{JointNum};
            PreviousAngle=ThisJoint.BendingAngle;
            NewBendingAngle=ThisJoint.DeformSlider.Value;
            span= ThisJoint.span;
            Level= ThisJoint.Level;
            dAngle=(NewBendingAngle-PreviousAngle)/5;
            for BendingAngleDegree=PreviousAngle+dAngle:dAngle:NewBendingAngle

                if(ThisJoint.direction=='v')
                    for i=span(1):span(2)
                        obj.HEdges{i,Level}.UpdateBending(BendingAngleDegree);
                    end
                else
                    for i=span(1):span(2)
                        obj.VEdges{Level,i}.UpdateBending(BendingAngleDegree);
                    end
                end

                % Change and run the Node Movements

                obj.MoveActiveNodes(1);
            end
            obj.Joints{JointNum}.BendingAngle=NewBendingAngle;

            obj.EnableJointSliders();
            pause(0.01);
            % disp("Joints Enabled")

        end

        function obj=UpdateMotors(obj,src,event)
            MotorNum=str2num(src.Tag(2:end));
            PreviousValue=obj.Motors{MotorNum}.ActValue;
            NewValue=event.Value/90;
            dValue=(NewValue-PreviousValue)/10;
            obj.DisableMotorSliders();
            pause(0.01);

            for Value=PreviousValue+dValue:dValue:NewValue
                obj.Motors{MotorNum}.ActValue=Value;
                obj.DeformationEstimation();

                obj.MoveActiveNodes(obj.DeformationRate);
                % Update the Motor plots
                obj.UpdateMotorDrawing();
            end
            obj.EnableMotorSliders();
            pause(0.01);
        end

        function UpdateMotorDrawing(obj)
            for k=1:length(obj.Motors)
                Node1=obj.Nodes{obj.Motors{k}.ORNode1(1),obj.Motors{k}.ORNode1(2)};
                Node2=obj.Nodes{obj.Motors{k}.ORNode2(1),obj.Motors{k}.ORNode2(2)};
                if(obj.Motors{k}.direction == 'v')
                    Node1TopCorner = Node1.Position + Node1.R0*Node1.Corners(:,2);
                    Node2BottomCorner = Node2.Position + Node2.R0*Node2.Corners(:,4);
                    MotorPos=(Node1TopCorner+Node2BottomCorner)/2;
                else
                    Node1RightCorner = Node1.Position + Node1.R0*Node1.Corners(:,3);
                    Node2LeftCorner = Node2.Position + Node2.R0*Node2.Corners(:,1);
                    MotorPos= (Node1RightCorner + Node2LeftCorner)/2;
                end
                obj.Motors{k}.s.XData=MotorPos(1);obj.Motors{k}.s.YData=MotorPos(2);obj.Motors{k}.s.ZData=MotorPos(3);
            end
        end

        function obj=DisableJointSliders(obj)
            for JointNum=1:length(obj.Joints)
                obj.Joints{JointNum}.DeformSlider.Enable=false;
            end
        end

        function obj=EnableJointSliders(obj)
            for JointNum=1:length(obj.Joints)
                obj.Joints{JointNum}.DeformSlider.Enable=true;
            end
        end

        function obj=DisableMotorSliders(obj)
            for MotorNum=1:length(obj.Motors)
                obj.Motors{MotorNum}.ActSlider.Enable=false;
            end
        end

        function obj=EnableMotorSliders(obj)
            for MotorNum=1:length(obj.Motors)
                obj.Motors{MotorNum}.ActSlider.Enable=true;
            end
        end

        function obj=HideDeformSliders(obj)
            if(~isempty(obj.Joints))
                for i=1:length(obj.Joints)
                    % obj.Joints{i}.DeformSlider.Enable=false;
                    obj.Joints{i}.DeformSlider.Visible=false;
                    obj.Joints{i}.AxisLabel.Visible=false;
                end
            end
        end

        function obj=ShowDeformSliders(obj)
            if(~isempty(obj.Joints))
                for i=1:length(obj.Joints)
                    % obj.Joints{i}.DeformSlider.Enable=true;
                    obj.Joints{i}.DeformSlider.Visible=true;
                    obj.Joints{i}.AxisLabel.Visible=true;
                end
            end
        end

        function obj=HideMotorSliders(obj)
            if(~isempty(obj.Motors))
                for i=1:length(obj.Motors)
                    % obj.Motors{i}.ActSlider.Enable=false;
                    obj.Motors{i}.ActSlider.Visible=false;
                    obj.Motors{i}.AxisLabel.Visible=false;
                end
            end
        end

        function obj=ShowMotorSliders(obj)
            if(~isempty(obj.Motors))
                for i=1:length(obj.Motors)
                    % obj.Motors{i}.ActSlider.Enable=true;
                    obj.Motors{i}.ActSlider.Visible=true;
                    obj.Motors{i}.AxisLabel.Visible=true;
                end
            end
        end

        function SlimModeFunction(obj,src,event)
            % disp(event.Value);
            for i = 1:obj.nRow+1
                for j = 1:obj.nCol+1
                    if(obj.NodeMatrix(i,j)==1)
                        if(event.Value==1)
                            obj.Nodes{i,j}.SlimMode=true;
                        else
                            obj.Nodes{i,j}.SlimMode=false;
                        end
                        obj.Nodes{i,j}.SlimModeUpdate();
                    end
                end
            end
        end


        %% Initialization methods

        function InitializeStructure(obj)
            % Initialize whole structure
            [X, Y, Z] = meshgrid(0:obj.nCol, 0:obj.nRow, 0);
            set(0, 'currentfigure', obj.fig);
            clf;hold on;

            %Set up horizontal flexible links
            for i = 1:size(obj.H_EdgeMatrix,1)
                for j = 1:size(obj.H_EdgeMatrix,2)
                    position = [X(i, j) Y(i, j) Z(i,j)]*2*obj.L+[X(i, j) 0 0]*obj.L;
                    obj.HEdges{i,j}=Edge(obj.DefaultEdgeWidth,obj.DefaultEdgeLength,position,'h',[i j],obj.L*2);
                    if(obj.H_EdgeMatrix(i,j)==1)
                        obj.HEdges{i,j}.Activate();
                        %                         UIPosition=[j*60-5 i*60+275 25 5];
                        %                         obj.HEdges{i,j}.CreateUIElement(obj.uifig,UIPosition,['H' num2str(i) num2str(j)]);
                    end
                end
            end

            %Set up vertical flexible links

            for i = 1:size(obj.V_EdgeMatrix,1)
                for j = 1:size(obj.V_EdgeMatrix,2)
                    %Horizontal edges go from Node(i,j) to Node(i+1,j)
                    position = [X(i, j) Y(i, j) Z(i,j)]*2*obj.L+[0 Y(i, j) 0]*obj.L;
                    obj.VEdges{i,j}=Edge(obj.DefaultEdgeWidth,obj.DefaultEdgeLength,position,'h',[i j],obj.L*2);
                    if(obj.V_EdgeMatrix(i,j)==1)
                        obj.VEdges{i,j}.Activate();
                        %                         UIPosition=[j*60-25 i*60+295 5 25];
                        %                         obj.VEdges{i,j}.CreateUIElement(obj.uifig,UIPosition,['V' num2str(i) num2str(j)]);
                    end
                end
            end

            for i = 1:obj.nRow+1
                for j = 1:obj.nCol+1
                    position = [X(i, j); Y(i, j); Z(i,j)]*2*obj.L;
                    % Initialize rigid nodes
                    [NodeSize, NeighborExists]=GetNeighborInfo(obj,i,j);
                    obj.Nodes{i, j} = Node(position, NodeSize,obj.NodeThickness,NeighborExists,obj.L,SetDir(i+j));
                    set(0, 'currentfigure', obj.fig);
                    if(obj.NodeMatrix(i,j)==1)
                        obj.Nodes{i,j}.DrawNode();
                        obj.NumActiveNodes=obj.NumActiveNodes+1;
                        obj.ActiveNodeLocs(obj.NumActiveNodes,:)=[i j];
                        obj.ActiveNodeNums(i,j)=obj.NumActiveNodes;
                        %                         UIPosition=[j*60-40 i*60+260 35 35];
                        %                         obj.Nodes{i,j}.CreateUIElement(obj.uifig,UIPosition,"NodeImage.png",['N' num2str(i) num2str(j)]);
                    end
                end
            end
            obj.DeformationRate=zeros(obj.NumActiveNodes,1);
        end

        function CreateModeLabels(obj)
            % 1: Building 2:Selecting Joints 3: Moving Joints 4: Check deformation 5: Add Motors 6: Control Motors
            Labels={'1: Building cell structure', '2:Selecting Joints', '3: Moving Joints', '4: Check deformation', '5: Add Motors', '6: Control Motors'};
            for i=1:6
                obj.ModeLabels{i}.label= uilabel(obj.uifig,'Position',[i*80-60, obj.top+30 70 30],'Text',Labels{i});
                obj.ModeLabels{i}.label.WordWrap = "on";
                obj.ModeLabels{i}.label.BackgroundColor = obj.Palette.gray;
            end
        end

        function GetRigidMat(obj)
            for i=1:obj.nRow
                for j=1:obj.nCol
                    if(obj.CellMatrix(i,j)==1)
                        obj.NodeMatrix(i:i+1,j:j+1)=ones(2);
                        obj.H_EdgeMatrix(i:i+1,j)=ones(2,1);
                        obj.V_EdgeMatrix(i,j:j+1)=ones(1,2);
                    end
                end
            end
        end

        function [NodeSize, NeighborExists]=GetNeighborInfo(obj,i,j)
            NeighborExists=[0 0 0 0];
            NodeSize=[0 0 0 0];
            if(i>1)
                if(obj.V_EdgeMatrix(i-1,j)==1)
                    NeighborExists(4)=1;
                    NodeSize(4)= obj.L-obj.VEdges{i-1,j}.length/2;
                end
            end
            if(i<=obj.nRow)
                if(obj.V_EdgeMatrix(i,j)==1)
                    NeighborExists(2)=1;
                    NodeSize(2)=obj.L-obj.VEdges{i,j}.length/2;
                end
            end
            if(j>1)
                if(obj.H_EdgeMatrix(i,j-1)==1)
                    NeighborExists(1)=1;
                    NodeSize(1)=obj.L-obj.HEdges{i,j-1}.length/2;
                end
            end
            if(j<=obj.nCol)
                if(obj.H_EdgeMatrix(i,j)==1)
                    NeighborExists(3)=1;
                    NodeSize(3)=obj.L-obj.HEdges{i,j}.length/2;
                end
            end
        end

        %% Simulation functions
        function SetMode(obj,x)
            obj.ModeLabels{obj.Mode}.label.BackgroundColor=obj.Palette.gray;
            obj.Mode=x;
            obj.ModeLabels{x}.label.BackgroundColor=obj.Palette.orange;
        end

        function DeformationEstimation(obj)
            % Compute Deformation Gradient and Hessian to find the next
            % deformations
            obj.DeformationRate=zeros(obj.NumActiveNodes,1);
            OverRiddenNodes=zeros();
            for k=1:length(obj.Motors)
                N1i=obj.Motors{k}.ORNode1(1);
                N1j=obj.Motors{k}.ORNode1(2);
                N2i=obj.Motors{k}.ORNode2(1);
                N2j=obj.Motors{k}.ORNode2(2);

                ORNodeNum1=obj.ActiveNodeNums(N1i,N1j);
                ORNodeNum2=obj.ActiveNodeNums(N2i,N2j);

                obj.DeformationRate(ORNodeNum1)=obj.Motors{k}.ActValue;
                obj.DeformationRate(ORNodeNum2)=obj.Motors{k}.ActValue;

                OverRiddenNodes((k-1)*2+1)=ORNodeNum1;
                OverRiddenNodes((k-1)*2+2)=ORNodeNum2;
            end

            NonOverridenNodes=1:obj.NumActiveNodes;
            NonOverridenNodes(OverRiddenNodes)=[];

            % Refreshing the gradient and hessian
            obj.DeformGradient=zeros(obj.NumActiveNodes,1);
            obj.DeformHessian=zeros(obj.NumActiveNodes,obj.NumActiveNodes);
            %Set up horizontal flexible links
            for i = 1:size(obj.H_EdgeMatrix,1)
                for j = 1:size(obj.H_EdgeMatrix,2)
                    %Horizontal edges go from Node(i,j) to Node(i,j+1)
                    if(obj.H_EdgeMatrix(i,j)==1)
                        Edge= obj.HEdges{i,j};
                        N1= obj.ActiveNodeNums(i,j);
                        N2= obj.ActiveNodeNums(i,j+1);
                        obj.DeformGradientHessianCalc(N1,N2,Edge);
                    end
                end
            end

            %Set up vertical flexible links
            for i = 1:size(obj.V_EdgeMatrix,1)
                for j = 1:size(obj.V_EdgeMatrix,2)
                    %Horizontal edges go from Node(i,j) to Node(i+1,j)
                    if(obj.V_EdgeMatrix(i,j)==1)
                        Edge= obj.VEdges{i,j};
                        N1= obj.ActiveNodeNums(i,j);
                        N2= obj.ActiveNodeNums(i+1,j);
                        obj.DeformGradientHessianCalc(N1,N2,Edge);
                    end
                end
            end

            dDefNodes=-obj.DeformHessian(NonOverridenNodes,NonOverridenNodes)\obj.DeformGradient(NonOverridenNodes);
            obj.DeformationRate(NonOverridenNodes)=dDefNodes;
            %             obj.DeformationRate=obj.DeformHessian\obj.DeformGradient;
            obj.DeformAmps2=zeros(obj.nRow,obj.nCol);
            for k=1:obj.NumActiveNodes
                Loc=obj.ActiveNodeLocs(k,:);
                obj.DeformAmps2(Loc(1),Loc(2))=obj.DeformationRate(k);
            end
        end

        function DeformGradientHessianCalc(obj,N1,N2,Edge)

            defN1=obj.DeformationRate(N1);
            defN2=obj.DeformationRate(N2);

            k1=Edge.k1;
            k2=Edge.k2;

            dGN1=(defN1+defN2*(1-2*k1))*k2;
            dGN2=(defN2+defN1*(1-2*k1))*k2;

            % Gradient Calculation
            obj.DeformGradient(N1)=obj.DeformGradient(N1)+dGN1;
            obj.DeformGradient(N2)=obj.DeformGradient(N2)+dGN2;
            % Hessian Calculation
            dHN1N1=k2;
            dHN1N2=(1-2*k1)*k2;
            dHN2N2=k2;

            obj.DeformHessian(N1,N1)=obj.DeformHessian(N1,N1)+dHN1N1;
            obj.DeformHessian(N1,N2)=dHN1N2;
            obj.DeformHessian(N2,N1)=dHN1N2;
            obj.DeformHessian(N2,N2)=obj.DeformHessian(N2,N2)+dHN2N2;
        end

        % Orientation Estimation
        function NodeOrientationEstimation(obj)
            % Per edge compute the angle between connected nodes
            Converged=false;
            iteration=0;
            while(~Converged)

                % Refreshing the gradient and hessian
                obj.QOGradient=zeros(obj.NumActiveNodes*4,1);
                obj.QOHessian=zeros(obj.NumActiveNodes*4,obj.NumActiveNodes*4);

                % Set up horizontal flexible links
                for i = 1:size(obj.H_EdgeMatrix,1)
                    for j = 1:size(obj.H_EdgeMatrix,2)
                        %Horizontal edges go from Node(i,j) to Node(i,j+1)
                        if(obj.H_EdgeMatrix(i,j)==1)
                            N1= obj.ActiveNodeNums(i,j);
                            N2= obj.ActiveNodeNums(i,j+1);
                            obj.QuaternionGradientHessianCalculation(N1,N2,'h');
                        end
                    end
                end

                %Set up vertical flexible links
                for i = 1:size(obj.V_EdgeMatrix,1)
                    for j = 1:size(obj.V_EdgeMatrix,2)
                        %Horizontal edges go from Node(i,j) to Node(i+1,j)
                        if(obj.V_EdgeMatrix(i,j)==1)
                            N1= obj.ActiveNodeNums(i,j);
                            N2= obj.ActiveNodeNums(i+1,j);
                            obj.QuaternionGradientHessianCalculation(N1,N2,'v');
                        end
                    end
                end

                NumGroundedNode=obj.ActiveNodeNums(obj.GroundedCellLoc(1),obj.GroundedCellLoc(2));
                GroundedNodeIndices=NumGroundedNode*4+(-3:0);
                NonGroundedNodeIndices=1:4*obj.NumActiveNodes;
                NonGroundedNodeIndices(GroundedNodeIndices)=[];
                %             else
                %                 NonGroundedNodeIndices=1:3*obj.NumActiveNodes;
                %             end
                dOrientation=zeros(4*obj.NumActiveNodes,1);

                Hessian=obj.QOHessian(NonGroundedNodeIndices,NonGroundedNodeIndices);
                Gradient=obj.QOGradient(NonGroundedNodeIndices);
                DetHessian=det(Hessian);
                if(norm(DetHessian)>1e-10)
                dOrientation(NonGroundedNodeIndices)=-Hessian\Gradient;
                else
                dOrientation(NonGroundedNodeIndices)=-.12*Gradient;
                end
                obj.AssignNewOrientation2Nodes(dOrientation);
                iteration=iteration+1;

                if(max(abs(dOrientation))< obj.OrientationThreshold)
                    Converged=true;
                    % disp(['Orientation converged in ' num2str(iteration) ' iterations.'])
                end
                if (iteration>5000)
                    Converged=true;
                    disp(['Did not converge in ' num2str(iteration) ' iterations.'])
                end
            end
        end

        function QuaternionGradientHessianCalculation(obj,N1,N2,direction)
            % disp("All Operations")
            % tic
            Node1Loc=obj.ActiveNodeLocs(N1,:);
            Node1=obj.Nodes{Node1Loc(1),Node1Loc(2)};
            Node2Loc=obj.ActiveNodeLocs(N2,:);
            Node2=obj.Nodes{Node2Loc(1),Node2Loc(2)};
            I4=eye(4);

            if(direction == 'v')
                RotAxis=[1 0 0];
                BendingAngle=obj.VEdges{Node1Loc(1),Node1Loc(2)}.BendingAmount;
            else
                RotAxis=[0 1 0];
                BendingAngle=obj.HEdges{Node1Loc(1),Node1Loc(2)}.BendingAmount;
            end
            qFrame1=Node1.q;
            qFrame2=Node2.q;

            qComp1=MakeQuat(BendingAngle*Node1.Deformation/2*RotAxis);
            qComp2=MakeQuat(-BendingAngle*Node2.Deformation/2*RotAxis);
            qA=QuatProduct(qFrame1,qComp1);
            qB=QuatProduct(qFrame2,qComp2);
            % E=-(qA'*qB)^2;
            dpqAqB=qA'*qB;

            tpqAqA=qA*qA';
            tpqBqB=qB*qB';

            [dqAu_dqA,d2qAu_dqA2]=UnitVectorGradHessian(qA);
            [dqBu_dqB,d2qBu_dqB2]=UnitVectorGradHessian(qB);
            
            dE_dqAu= -2*dpqAqB*qB';
            dE_dqBu= -2*dpqAqB*qA';

            dE_dqA=dE_dqAu*dqAu_dqA;
            dE_dqB=dE_dqBu*dqBu_dqB;

            dqA_dqf1=QuatSkewSymMat(qComp1,'L');
            dqB_dqf2=QuatSkewSymMat(qComp2,'L');

            dE_dqf1=dE_dqA*dqA_dqf1;
            dE_dqf2=dE_dqB*dqB_dqf2;

            %% Hessian calculation

            d2E_dqAu2= -2*tpqBqB;
            d2E_dqBu2= -2*tpqAqA;
            d2E_dqAudqBu= -2*(qB*qA'+ dpqAqB*I4);
            % d2E_dqBudqAu= -2*qA*qB';

            d2E_dqA2= dqAu_dqA'*d2E_dqAu2*dqAu_dqA + tensorprod(d2qAu_dqA2,dE_dqAu,3,2);
            d2E_dqB2= dqBu_dqB'*d2E_dqBu2*dqBu_dqB + tensorprod(d2qBu_dqB2,dE_dqBu,3,2);

            d2E_dqAdqB= dqAu_dqA'*d2E_dqAudqBu*dqBu_dqB;
            % d2E_dqBdqA= dqBu_dqB'*d2E_dqBudqAu*dqAu_dqA;

            d2E_dqf12=dqA_dqf1'*d2E_dqA2*dqA_dqf1;
            d2E_dqf22=dqB_dqf2'*d2E_dqB2*dqB_dqf2;
            d2E_dqf1dqf2=dqA_dqf1'*d2E_dqAdqB*dqB_dqf2;
            % dE2_dqf2dqf1=dqB_dqf2'*d2E_dqBdqA'*dqA_dqf1;
            %
            N1Indices=N1*4+(-3:0);
            N2Indices=N2*4+(-3:0);
            % Gradient Calculation
            obj.QOGradient(N1Indices)=obj.QOGradient(N1Indices)+dE_dqf1';
            obj.QOGradient(N2Indices)=obj.QOGradient(N2Indices)+dE_dqf2';
            % Hessian Calculation
            obj.QOHessian(N1Indices,N1Indices)=obj.QOHessian(N1Indices,N1Indices)+d2E_dqf12;
            obj.QOHessian(N1Indices,N2Indices)=d2E_dqf1dqf2;
            obj.QOHessian(N2Indices,N1Indices)=d2E_dqf1dqf2';
            obj.QOHessian(N2Indices,N2Indices)=obj.QOHessian(N2Indices,N2Indices)+d2E_dqf22;
            % toc
        end

        function AssignNewOrientation2Nodes(obj,dOrientation)
            for NodeNum=1:obj.NumActiveNodes
                NodeLoc=obj.ActiveNodeLocs(NodeNum,:);
                obj.Nodes{NodeLoc(1),NodeLoc(2)}.UpdateOrientation(dOrientation(4*NodeNum+(-3:0)));
            end
        end

        % Position Estimation
        function NodePositionEstimation(obj)
            % Per edge compute the distance between connected nodes

            Converged=false;
            iteration=0;
            while(~Converged)


            % Refreshing the gradient and hessian
            obj.CDGradient=zeros(obj.NumActiveNodes*3,1);
            obj.CDHessian=zeros(obj.NumActiveNodes*3,obj.NumActiveNodes*3);

            if(obj.Mode==2 || obj.Mode==4)
                GroundNode=obj.Nodes{obj.GroundedCellLoc(1),obj.GroundedCellLoc(2)};
                GroundNode.Position=GroundNode.InitialPosition/sqrt(2);
            end

            if(obj.Mode==5)
                GroundNode=obj.Nodes{obj.GroundedCellLoc(1),obj.GroundedCellLoc(2)};
                GroundNode.Position=GroundNode.InitialPosition;
            end
            
            % Set up horizontal flexible links
            for i = 1:size(obj.H_EdgeMatrix,1)
                for j = 1:size(obj.H_EdgeMatrix,2)
                    %Horizontal edges go from Node(i,j) to Node(i,j+1)
                    if(obj.H_EdgeMatrix(i,j)==1)
                        N1= obj.ActiveNodeNums(i,j);
                        N2= obj.ActiveNodeNums(i,j+1);
                        obj.CornerDistanceGradientHessianCalculation(N1,N2,'h');
                    end
                end
            end

            %Set up vertical flexible links
            for i = 1:size(obj.V_EdgeMatrix,1)
                for j = 1:size(obj.V_EdgeMatrix,2)
                    %Horizontal edges go from Node(i,j) to Node(i+1,j)
                    if(obj.V_EdgeMatrix(i,j)==1)
                        N1= obj.ActiveNodeNums(i,j);
                        N2= obj.ActiveNodeNums(i+1,j);
                        obj.CornerDistanceGradientHessianCalculation(N1,N2,'v');
                    end
                end
            end
       


            NumGroundedNode=obj.ActiveNodeNums(obj.GroundedCellLoc(1),obj.GroundedCellLoc(2));
            GroundedNodeIndices=NumGroundedNode*3+(-2:0);
            NonGroundedNodeIndices=1:3*obj.NumActiveNodes;
            NonGroundedNodeIndices(GroundedNodeIndices)=[];

            dPosNodes=zeros(3*obj.NumActiveNodes,1);
            dPosNodes(NonGroundedNodeIndices)=-obj.CDHessian(NonGroundedNodeIndices,NonGroundedNodeIndices)\obj.CDGradient(NonGroundedNodeIndices);

            obj.AssignNewPosition2Nodes(dPosNodes);
            

            iteration=iteration+1;

                if(max(abs(dPosNodes))< 1e-2 )%obj.PositionThreshold)
                    Converged=true;
                    % disp(['Position converged in ' num2str(iteration) ' iterations.'])
                end
                if (iteration>5000)
                    Converged=true;
                    disp(['Position did not converge in ' num2str(iteration) ' iterations.'])
                end
            end

        end

        function CornerDistanceGradientHessianCalculation(obj,N1,N2,direction)
            k=1;
            L0=0;
            Node1Loc=obj.ActiveNodeLocs(N1,:);
            Node1=obj.Nodes{Node1Loc(1),Node1Loc(2)};
            Node2Loc=obj.ActiveNodeLocs(N2,:);
            Node2=obj.Nodes{Node2Loc(1),Node2Loc(2)};
            if(direction == 'v')
                Node1TopCorner = Node1.Position + Node1.R0*Node1.Corners(:,2);
                Node2BottomCorner = Node2.Position + Node2.R0*Node2.Corners(:,4);
                EdgeVector= Node1TopCorner-Node2BottomCorner;
            else
                Node1RightCorner = Node1.Position + Node1.R0*Node1.Corners(:,3);
                Node2LeftCorner = Node2.Position + Node2.R0*Node2.Corners(:,1);
                EdgeVector= Node1RightCorner - Node2LeftCorner;
            end
            EdgeLength=norm(EdgeVector);
            if(EdgeLength>1e-6)
                EdgeTangent=EdgeVector/EdgeLength;
            else
                EdgeTangent=[1e-3 0 0]';
                EdgeLength=1e-3;
            end
            dE_dv=k*(norm(EdgeVector) - L0)*EdgeTangent;

            dtde=(eye(3) - EdgeTangent'*EdgeTangent)/EdgeLength;
            d2E_dv2=k*(EdgeTangent'*EdgeTangent+(EdgeLength-L0)*dtde);

            dE_dp1=dE_dv;
            dE_dp2=-dE_dv;
            dE2_dp1dp1= d2E_dv2;
            dE2_dp1dp2 = -d2E_dv2;
            dE2_dp2dp1 = dE2_dp1dp2';
            dE2_dp2dp2 = d2E_dv2;

            N1Indices=N1*3+(-2:0);
            N2Indices=N2*3+(-2:0);
            % Gradient Calculation
            obj.CDGradient(N1Indices)=obj.CDGradient(N1Indices)+dE_dp1;
            obj.CDGradient(N2Indices)=obj.CDGradient(N2Indices)+dE_dp2;
            % Hessian Calculation
            obj.CDHessian(N1Indices,N1Indices)=obj.CDHessian(N1Indices,N1Indices)+dE2_dp1dp1;
            obj.CDHessian(N1Indices,N2Indices)=dE2_dp1dp2;
            obj.CDHessian(N2Indices,N1Indices)=dE2_dp2dp1;
            obj.CDHessian(N2Indices,N2Indices)=obj.CDHessian(N2Indices,N2Indices)+dE2_dp2dp2;
        end

        function AssignNewPosition2Nodes(obj,dPosNodes)
            for NodeNum=1:obj.NumActiveNodes
                NodeLoc=obj.ActiveNodeLocs(NodeNum,:);
                obj.Nodes{NodeLoc(1),NodeLoc(2)}.UpdatePosition(dPosNodes(3*NodeNum+(-2:0)));
                obj.Nodes{NodeLoc(1),NodeLoc(2)}.UpdatePlot();
            end
            % tic
            % % drawnow;
            pause(0.001);
            % toc
        end

        function MoveActiveNodes(obj,val)

            obj.UpdateDeformations(val);
            
            % First orientation update - Per edge orientation energy optimization
            obj.NodeOrientationEstimation();

            % Second orientation update - Turn around z by deformation
            obj.FullOrientationUpdate();


            % Position update - Per edge stretch energy optimization
            NodePositionEstimation(obj);
        end

        function obj=FullOrientationUpdate(obj)
            for k=1:obj.NumActiveNodes
                NodeLoc=obj.ActiveNodeLocs(k,:);
                obj.Nodes{NodeLoc(1),NodeLoc(2)}.UpdateFullOrientation();
            end
        end

        function obj=UpdateDeformations(obj,val)
            for k=1:obj.NumActiveNodes
                NodeLoc=obj.ActiveNodeLocs(k,:);
                if(isscalar(val)) % If all nodes are deformed the same amount
                    DeformationAmount=val;
                else               % If separate deformations are applied to each node
                    DeformationAmount=val(k);
                end
                obj.Nodes{NodeLoc(1),NodeLoc(2)}.UpdateDeformation(DeformationAmount);
            end
        end


    end
end

function y=SetDir(x)
if(rem(x, 2) == 0)
    y=1;
else
    y=-1;
end

end

function MoveCameraToDesired(DesiredCaz,DesiredCel)
[caz,cel] = view;
cazArr=linspace(caz,DesiredCaz,20);
celArr=linspace(cel,DesiredCel,20);
for i=2:20
    view(cazArr(i),celArr(i));
    pause(0.01);
end
end

function Q=MakeQuat(angleAxis)
% Extract the angle and axis components
angle = norm(angleAxis);
if angle>0
    axis = angleAxis / angle;
else
    axis = zeros(1,3);
end
ijk=axis*sin(angle/2);
% Q = quaternion(cos(angle/2), ijk(1), ijk(2), ijk(3));
Q=[cos(angle/2); ijk(1); ijk(2); ijk(3)];
end

function q3=QuatProduct(q1,q2)
%     q3=q2*q1;
q1ML=QuatSkewSymMat(q1,'L');
q3=q1ML*q2;
end


function M4=QuatSkewSymMat(x,dir)
if(dir=='L')
    M4=[x(1) -x(2) -x(3) -x(4);...
        x(2) x(1) -x(4) x(3);...
        x(3) x(4) x(1) -x(2);...
        x(4) -x(3) x(2) x(1)];
else
    M4=[x(1) -x(2) -x(3) -x(4);...
        x(2) x(1) x(4) -x(3);...
        x(3) -x(4) x(1) x(2);...
        x(4) x(3) -x(2) x(1)];
end

end

function [dqAu_dqA,d2qAu_dqA2]=UnitVectorGradHessian(qA)

I4=eye(4);

%Gradient
tpqAqA=qA*qA';
dqAu_dqA = I4-tpqAqA';

%Hessian
PartA1=-tensorprod(qA,I4,2,3);
PartA2=permute(PartA1,[2 1 3]);
PartA3=3*tensorprod((tpqAqA),qA,3,2);
PartA4=-tensorprod(I4,qA,3,2);

d2qAu_dqA2= PartA1 + PartA2 + PartA3 + PartA4;

end