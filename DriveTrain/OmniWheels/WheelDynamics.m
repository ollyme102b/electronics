%% Wheel Dynamics

Dynamics = [-sqrt(3)/2, 0.5, 1; 0 -1, 1; sqrt(3)/2, 0.5, 1];
Dynamics^-1



%% Plotting Sim

close all

vidObj = VideoWriter('olly.avi');
open(vidObj);

theta = 0
d=0.5;
moveX = 0; 
moveY = 0;

for i = 1:200

    bodyX = [cos(theta+pi/2)+moveX, cos(theta + 7*pi/6)+moveX, cos(theta + 11*pi/6)+moveX, cos(theta+pi/2)+moveX]; 
    bodyY = [sin(theta+pi/2)+moveY, sin(theta + 7*pi/6)+moveY, sin(theta + 11*pi/6)+moveY, sin(theta+pi/2)+moveY];
    wheels = 0.6*[ cos(theta+5*pi/6), cos(theta + 3*pi/2), cos(theta + pi/6); 
               sin(theta+5*pi/6), sin(theta + 3*pi/2), sin(theta + pi/6)] + [ones(1,3)*moveX; ones(1,3)*moveY];

    v = [sin(i/20), sin(i/5), cos(i/10)];        
    spin = [v(1)*cos(theta+pi/3), v(2)*cos(theta + pi), v(3)*cos(theta-pi/3);
            v(1)*sin(theta+pi/3), v(2)*sin(theta + pi), v(3)*sin(theta-pi/3);];
        
    dynamics = [sin(pi/3), cos(pi/3),   -d; 
                 0,       -1,            -d;
                -sin(pi/3), cos(pi/3),  -d]; % ** In the body frame **
        
    step = (dynamics)^-1*v'; % step = [vy, vx, omega];
    step = [step(1)*cos(theta) + step(2)*sin(theta), step(1)*sin(theta) + step(2)*cos(theta), step(3)];
    dt = 0.07; % Variable Time Step
    moveX = moveX + step(2)*dt; 
    moveY = moveY + step(1)*dt; 
    theta = theta + step(3)*dt;
    
        
    clf
    plot(bodyX, bodyY)
    hold on
    quiver(wheels(1, :), wheels(2, :), spin(1,:), spin(2,:))
    xlim([-5, 5]); 
    ylim([-5, 5]);
    
   currFrame = getframe(gcf);
   writeVideo(vidObj,currFrame);

end

% Close the file.
close(vidObj);







