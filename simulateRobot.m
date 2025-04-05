% Run the main.m file first

config = homeConfig;
speed = 100;
for i = 1:speed:numel(trajectoryTime) 
    clf
    for j = 1:1:7
        config(j).JointPosition = jointAngles(j,i);
    end
    tgtPose = T(:,:,i);
    show(robot,config,'Frames','off');
    hold on
    plot3(q(1,:),q(2,:),q(3,:),'r--','LineWidth',2)
    drawAxes(tgtPose)
    drawAxes(startTransformation)
    drawAxes(targetTransformation)
    title(['Position of robot at t = ' num2str(trajectoryTime(i)) ' sec'])
    axis equal
    axis([-0.3 0.5 -0.2 0.8 -0.1 1])
    view([-119 25])
    pause(0.001)
end

function drawAxes(T)
    p = tform2trvec(T);
    X = T(1:3,1);
    Y = T(1:3,2);
    Z = T(1:3,3);
    q1 = quiver3(p(1), p(2), p(3), X(1), X(2), X(3),'r','AutoScaleFactor',0.2,'LineWidth',2);
    q2 = quiver3(p(1), p(2), p(3), Y(1), Y(2), Y(3),'g','AutoScaleFactor',0.2,'LineWidth',2);
    q3 = quiver3(p(1), p(2), p(3), Z(1), Z(2), Z(3),'b','AutoScaleFactor',0.2,'LineWidth',2);
end