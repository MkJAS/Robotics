function Lab8Starter( )
close all
clear all
%%
% Define the camera
cam = CentralCamera('focal', 0.015, 'pixel', 10e-6, ...
'resolution', [640 480], 'centre', [320 240], 'name', 'mycamera');


%Create 3D point
P = mkgrid( 2, 0.5, 'T', transl(0,0,5) );

%Set the position of the camera
Tc0 = transl(0,0,1)*troty(0.1);
cam.T = Tc0;

%camera view and plotting
cam.clf()
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view

% this is the 'external' view of the points and the camera
plot_sphere(P, 0.05, 'b')
cam.plot_camera(P, 'label','scale',0.15);
grid on