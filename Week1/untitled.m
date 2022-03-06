clear all;
close all;

imshow('Lab1_CircularRaceTrack.jpg'); 
axis on;

hold on;
car1Tr = se2(300, 550);

car1Tr_h = trplot2(car1Tr, 'frame', '1', 'color', 'b','length',50);
hold on;
circ = pi*460;

moveincr = circ/360;
turnincr = -2*pi/360;

moveTr = se2(moveincr,0,0);
turnTr = se2(0,0,turnincr);

for i = 1:360
    car1Tr = car1Tr * moveTr * turnTr;
    delete(car1Tr_h);
    try delete(texts); end;
    car1Tr_h = trplot2(car1Tr, 'frame', '1', 'color', 'b','length',50);
    message = sprintf([num2str(round(car1Tr(1,:),2,'significant')),'\n' ...
                      ,num2str(round(car1Tr(2,:),2,'significant')),'\n' ...
                      ,num2str(round(car1Tr(3,:),2,'significant'))]);
    texts = text(10, 50, message, 'FontSize', 10, 'Color', [.6 .2 .6]);
    drawnow();
    pause(0.01);
    hold on;
end
%% 


car2Tr = se2(300, 125);
car2Tr_h = trplot2(car2Tr, 'frame', '2', 'color', 'r','length',50);

moveincr2 = circ/360;
turnincr2 = 2*pi/360;

moveTr2 = se2(moveincr2,0,0);
turnTr2 = se2(0,0,turnincr2);


for i = 1:360
    car2Tr = car2Tr * moveTr2 * turnTr2;
    delete(car2Tr_h);
    try delete(texts); end;
    car2Tr_h = trplot2(car2Tr, 'frame', '2', 'color', 'r','length',50);
    drawnow();
    pause(0.01);   
    hold on;
end
%% 
 delete(car2Tr_h);
 delete(car1Tr_h);
dist = zeros(1,360);

car1Tr = se2(300, 550);
car2Tr = se2(300, 125);
car1Tr_h = trplot2(car1Tr, 'frame', '1', 'color', 'b','length',50);
hold on;

car2Tr_h = trplot2(car2Tr, 'frame', '2', 'color', 'r','length',50);

rltvTr = car2Tr/car2Tr;

circ = pi*460;

moveincr = circ/360;
turnincr = -2*pi/360;
moveTr = se2(moveincr,0,0);
turnTr = se2(0,0,turnincr);

moveincr2 = circ/360;
turnincr2 = 2*pi/360;
moveTr2 = se2(moveincr2,0,0);
turnTr2 = se2(0,0,turnincr2);

for i = 1:360
    car1Tr = car1Tr * moveTr * turnTr;
    car2Tr = car2Tr * moveTr2 * turnTr2;
    delete(car2Tr_h);
    delete(car1Tr_h);
    car2Tr_h = trplot2(car2Tr, 'frame', '2', 'color', 'r','length',50);
    car1Tr_h = trplot2(car1Tr, 'frame', '1', 'color', 'b','length',50);
    try delete(texts); end;
    try delete(texts2); end;
    message = sprintf([num2str(round(car1Tr(1,:),2,'significant')),'\n' ...
                      ,num2str(round(car1Tr(2,:),2,'significant')),'\n' ...
                      ,num2str(round(car1Tr(3,:),2,'significant'))]);
    texts = text(10, 50, message, 'FontSize', 10, 'Color', [.6 .2 .6]);
    message2 = sprintf([num2str(round(car2Tr(1,:),2,'significant')),'\n' ...
                      ,num2str(round(car2Tr(2,:),2,'significant')),'\n' ...
                      ,num2str(round(car2Tr(3,:),2,'significant'))]);
    texts2 = text(10, 500, message2, 'FontSize', 10, 'Color', [.6 .2 .6]);
    drawnow();
    pause(0.01);   
    hold on;
    rltvTr = car2Tr/car2Tr;
        
%         car1_to_2Tr = inv(car1Tr) * car2Tr
%         car2_to_1Tr = inv(car2Tr) * car1Tr

    dist(i) = sqrt((car1Tr(1,3)- car2Tr(1,3))^2 + (car1Tr(2,3) - car2Tr(2,3))^2);

end
y = 1:360;
hold off;
figure;
plot(y,dist);
xlim([0 360]);
ylabel('Distance');
xlabel('Increments');    

















