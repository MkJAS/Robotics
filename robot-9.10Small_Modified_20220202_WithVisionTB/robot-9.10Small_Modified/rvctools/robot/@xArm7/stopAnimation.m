%% stopAnimation
%


function stopAnimation(robot, q, isHolding, prop)
   robot.model.animate(q);
   drawnow();
   if isHolding == 1
       prop.updatePosition(robot.model.fkine(q) * (transl(0, 0, 0) * trotx(pi) * trotz(pi/2)));
   end
end