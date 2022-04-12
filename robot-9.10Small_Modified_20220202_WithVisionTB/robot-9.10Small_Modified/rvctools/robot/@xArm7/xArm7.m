classdef xArm7 < handle
    %% Properties
    properties
        model;
        workspace = [-2 2 -2 2 -0.8 2]; 
        base;
        isHolding;
        heldProp;
        hitBoxes;
        %maximumReachAndVolume;
    end
    %% Methods
    methods
        function self = xArm7(base)
            if nargin < 1
               base = transl(0, 0, 0);
            end
            self.base = base;
            self.getXArm7();
            self.modelRobot();
            %figure(2)
            %initJointAngles = [0, 0, 0, 0, 0, 0, 0];
            %self.model.plot(initJointAngles);
            %self.maximumReachAndVolume = self.getMaximumReachAndVolume();
        end
        
        function getXArm7(self)
           L(1) = Link('offset', pi, 'd', 0.267, 'a', 0, 'alpha', -pi/2, 'qlim', [-2*pi, 2*pi]);
           L(2) = Link('offset', 0, 'd', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-2.0769, 2.0769]);
           L(3) = Link('offset', pi, 'd', 0.293, 'a', 0.0525, 'alpha', pi/2, 'qlim', [-2*pi, 2*pi]);
           L(4) = Link('offset', 0, 'd', 0, 'a', 0.0775, 'alpha', pi/2, 'qlim', [-0.1920, 3.9270]);
           L(5) = Link('offset', pi, 'd', 0.3425, 'a', 0, 'alpha', pi/2, 'qlim', [-2*pi, 2*pi]);
           L(6) = Link('offset', 0, 'd', 0, 'a', -0.076, 'alpha', -pi/2, 'qlim', [-1.6930, 1.8151]);
           L(7) = Link('offset', 0, 'd', 0.097 + 0.163 - 0.02, 'a', 0, 'alpha', 0, 'qlim', [-2*pi, 2*pi]);
           
           self.model = SerialLink(L, 'name', 'xArm7', 'base', self.base); 
        end
        
        function modelRobot(self)
            for linkIndex = 0:self.model.n
%                 if self.useGripper && linkIndex == self.model.n
%                     [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['xArm7Joint', num2str(linkIndex), 'Gripper.ply'], 'tri'); %#ok<AGROW>
%                 else
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['xArm7Joint', num2str(linkIndex), '.ply'], 'tri'); %#ok<AGROW>
%                 end
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
        