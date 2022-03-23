function [wallgoals,m] = GetGoals(base1,base2,r,brick)

    wallgoals = zeros(9,3);

    A = base1(1:2); %UR3 base
    B = base2(1:2); %UR5 base

    V = B - A;
    m = (B(2)-A(2))/(B(1)-A(1)); %gradient between two bots
    normal = [ V(2), -V(1)];    %get line normal to vector connecting UR3 and UR5


    point = [A(1)+normal(1) A(2)+normal(2)];

    theta = atan((A(2)-point(2))/(A(1)-point(1)));   %get the angle between UR3 and normal line

    

    x1 = r*0.6*cos(theta)+A(1); %x,y coords at a distance of 0.6r from UR3 along normal line
    y1 = r*0.6*sin(theta)+A(2);


    %get a point (x,y) at a distance of d from another
    %point (x1,y1) given a known gradient m = atan(y/x)

    d = r*0.5 - brick.x/2;       
    theta = atan((A(2)-B(2))/(A(1)-B(1)));
    x = d*cos(theta)+x1;
    y = d*sin(theta)+y1;

    wallgoals(1,:) = [x,y,brick.z];
    d = brick.x;
    layer = 1;
    
    while layer <= 3
        switch(layer)
            case 1
                for i=2:3
                    x = d*cos(theta)+x;
                    y = d*sin(theta)+y;
                    wallgoals(i,:) = [x,y,brick.z];
                end
                layer = 2;
            case 2
                for i=4:6
                    x = wallgoals(i-3,1);
                    y = wallgoals(i-3,2);
                    wallgoals(i,:) = [x,y,brick.z*2];
                end
                layer = 3;
            case 3
                for i=7:9
                    x = wallgoals(i-6,1);
                    y = wallgoals(i-6,2);
                    wallgoals(i,:) = [x,y,brick.z*3];            
                end
                layer = 4;
        end        
    end


end

