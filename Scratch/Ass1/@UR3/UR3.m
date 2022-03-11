classdef UR3 < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 -0.3 2];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;
        base;
        maximumReachAndVolume;
        
    end
    
    methods%% Class for UR3 robot simulation
        function self = UR3(base)
            if nargin < 1
                base = transl(0, 0, 0);
            end
            %self.useGripper = useGripper;

        %> Define the boundaries of the workspace
    
        self.base = base;
        % robot = 
        self.GetUR3Robot();
        self.PlotAndColourRobot();
        
        %robot params =
%         self.maximumReachAndVolume = self.getMaximumReachAndVolume; 
        end

        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetUR3Robot(self)
        %     if nargin < 1
                % Create a unique name (ms timestamp after 1ms pause)
                pause(0.001);
                name = ['UR_3_', datestr(now, 'yyyymmddTHHMMSSFFF')];
        %     end
            L(1) = Link('d', 0.1519, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'qlim', [-2*pi 2*pi]);
            L(2) = Link('d', 0, 'a', -0.24365, 'alpha', -pi,'offset', pi/2, 'qlim', [-pi/2,pi/2]);
            L(3) = Link('d', 0, 'a', -0.21325, 'alpha', pi, 'offset', 0, 'qlim', [-170*pi/180,170*pi/180]);
            L(4) = Link('d', 0.11235, 'a', 0, 'alpha', -pi/2, 'offset', pi/2, 'qlim', [-2*pi 2*pi]);
            L(5) = Link('d', 0.08535, 'a', 0, 'alpha', pi/2, 'offset', pi, 'qlim', [-2*pi 2*pi]);
            L(6) = Link('d', 0.0819, 'a', 0, 'alpha', 0, 'offset', pi, 'qlim', [-2*pi 2*pi]);

            self.model = SerialLink(L, 'name', name, 'base', self.base);
           end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                if self.useGripper && linkIndex == self.model.n
                    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['UR3Link', num2str(linkIndex), 'Gripper.ply'], 'tri'); %#ok<AGROW>
                else
                    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['UR3Link', num2str(linkIndex), '.ply'], 'tri'); %#ok<AGROW>
                end
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1, self.model.n), 'noarrow', 'workspace', self.workspace);
            if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles, 'UserData');
                try 
                    h.link(linkIndex + 1).Children.FaceVertexCData = [plyData{linkIndex + 1}.vertex.red ...
                                                                  , plyData{linkIndex + 1}.vertex.green ...
                                                                  , plyData{linkIndex + 1}.vertex.blue]/255;
                    h.link(linkIndex + 1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
    end
end