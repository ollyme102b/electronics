%% Wheel Dynamics

Dynamics = [-sqrt(3)/2, 0.5, 1; 0 -1, 1; sqrt(3)/2, 0.5, 1];
Dynamics^-1



%% Plotting Sim

close all

vidObj = VideoWriter('olly.avi');
open(vidObj);

theta = 0;
d=0.5;
moveX = 0; 
moveY = 0;

theta2 = 0; 
moveX2 = 3; 
moveY2 = 0; 

for i = 1:200

    bodyX = [cos(theta+pi/2)+moveX, cos(theta + 7*pi/6)+moveX, cos(theta + 11*pi/6)+moveX, cos(theta+pi/2)+moveX]; 
    bodyY = [sin(theta+pi/2)+moveY, sin(theta + 7*pi/6)+moveY, sin(theta + 11*pi/6)+moveY, sin(theta+pi/2)+moveY];
    bodyX2 = [cos(theta2+pi/2)+moveX2, cos(theta2 + 7*pi/6)+moveX2, cos(theta2 + 11*pi/6)+moveX2, cos(theta2+pi/2)+moveX2]; 
    bodyY2 = [sin(theta2+pi/2)+moveY2, sin(theta2 + 7*pi/6)+moveY2, sin(theta2 + 11*pi/6)+moveY2, sin(theta2+pi/2)+moveY2];
    
    wheels = 0.6*[ cos(theta+5*pi/6), cos(theta + 3*pi/2), cos(theta + pi/6); 
               sin(theta+5*pi/6), sin(theta + 3*pi/2), sin(theta + pi/6)] + [ones(1,3)*moveX; ones(1,3)*moveY];
    wheels2 = 0.6*[ cos(theta2+5*pi/6), cos(theta2 + 3*pi/2), cos(theta2 + pi/6); 
               sin(theta2+5*pi/6), sin(theta2 + 3*pi/2), sin(theta2 + pi/6)] + [ones(1,3)*moveX2; ones(1,3)*moveY2];

    
    v = [sin(i/20), sin(i/5), cos(i/10)];       
    v2 = [1, 0, 0.5]; 
    spin = [v(1)*cos(theta+pi/3), v(2)*cos(theta + pi), v(3)*cos(theta-pi/3);
            v(1)*sin(theta+pi/3), v(2)*sin(theta + pi), v(3)*sin(theta-pi/3);];
    spin2 = [v2(1)*cos(theta2+pi/3), v2(2)*cos(theta2 + pi), v2(3)*cos(theta2-pi/3);
            v2(1)*sin(theta2+pi/3), v2(2)*sin(theta2 + pi), v2(3)*sin(theta2-pi/3);];
        
    dynamics = [sin(pi/3), cos(pi/3),   -d; 
                 0,       -1,            -d;
                -sin(pi/3), cos(pi/3),  -d]; % ** In the body frame **
        
    step = (dynamics)^-1*v'; % step = [vy, vx, omega];
    step = [step(1)*cos(theta) + step(2)*sin(theta), step(1)*sin(theta) + step(2)*cos(theta), step(3)];
    dt = 0.07; % Variable Time Step
    moveX = moveX + step(2)*dt; 
    moveY = moveY + step(1)*dt; 
    theta = theta + step(3)*dt;
    
    step2 = (dynamics)^-1*v2'; % step = [vy, vx, omega];
    step2 = [step2(1)*cos(theta2) + step2(2)*sin(theta2), step2(1)*sin(theta2) + step2(2)*cos(theta2), step2(3)];
    moveX2 = moveX2 + step2(2)*dt; 
    moveY2 = moveY2 + step2(1)*dt; 
    theta2 = theta2 + step2(3)*dt;
    
    clf
    plot(bodyX, bodyY)
    hold on
    quiver(wheels(1, :), wheels(2, :), spin(1,:), spin(2,:))
    plot(bodyX2, bodyY2)
    plot(moveX, moveY, '*')
    plot(moveX2, moveY2, 'o')
    quiver(wheels2(1, :), wheels2(2, :), spin2(1,:), spin2(2,:))
    xlim([-5, 5]); 
    ylim([-5, 5]);
    
    beamLength = sqrt((moveX2-moveX)^2 + (moveY2-moveY)^2)
    if beamLength<1.75
        plot([moveX, moveX2], [moveY, moveY2], 'r');
    elseif beamLength <2.25 
        plot([moveX, moveX2], [moveY, moveY2], 'm');
    elseif beamLength <2.75 
        plot([moveX, moveX2], [moveY, moveY2], 'y');
    elseif beamLength<3.25
        plot([moveX, moveX2], [moveY, moveY2], 'g');
    elseif beamLength<3.75 
        plot([moveX, moveX2], [moveY, moveY2], 'y');
    elseif beamLength<4.25 
        plot([moveX, moveX2], [moveY, moveY2], 'm');
    else 
        plot([moveX, moveX2], [moveY, moveY2], 'r');
    end
    title(sprintf('Beam Length = %0.5g', beamLength))
   currFrame = getframe(gcf);
   writeVideo(vidObj,currFrame);

end

% Close the file.
close(vidObj);








