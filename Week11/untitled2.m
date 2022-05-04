close all 
clear all

pStar = [662 362; 362 362; 362 662; 662 662]';
P = [1.8, -0.25, 1.25; 1.8, 0.25, 1.25;1.8, 0.25, 0.75; 1.8, -0.25, 0.75]';
q0 = [pi/2 -pi/3 -pi/3 -pi/6 0 0];
robot = UR10;

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512], 'name', 'mycamera');
fps = 25;
Tc0 = robot.model.fkine(q0);
robot.model.animate(q0);
cam.T = Tc0;
depth = 1.8;
cam.plot_camera('Tcam',Tc0);
%%
cam.plot(pStar, '*');
cam.hold(true);

cam.plot(P, 'Tcam', Tc0, 'o'); %create the camera view
%%
steps = 200;
deltaT  = 1/fps;
qMatrix(1,:) = q0;
for i=1:200
    uv = cam.plot(P);

    e = pStar - uv;
    e = e(:);
    
    if mean(e) == 0
        break
    end
    

    J = cam.visjac_p(uv, depth);
    % J = J(1:3,:);


    lambda = 0.1;
    v = lambda*pinv(J)*e;

    Jr = robot.model.jacob0(q0);
    
    qdot = (pinv(Jr)*v)'; 
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot;
    Tc0 = robot.model.fkine(qMatrix(i+1,:));

    cam.T = Tc0;
    
    robot.model.animate(qMatrix(i+1,:));
    cam.plot_camera('Tcam',Tc0);
    pause(0.1);
     
    
end
% P = ???

