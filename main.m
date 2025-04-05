clc
clear
clf

% Load the robot - KUKA LBR iiwa 7 R800 7-axis robot
robot = loadrobot("kukaIiwa7");

% Add camera to robot
linkCamera = rigidBody('camera');
linkCamera.Mass = 0;
jointCamera = rigidBodyJoint('jnt_cam','fixed');
setFixedTransform(jointCamera,[0.0662 0 0.0431 -pi/2],'dh');
linkCamera.Joint = jointCamera;
addBody(robot,linkCamera,'iiwa_link_ee_kuka')

% Add block to robot
linkBlock = rigidBody('block');
linkBlock.Mass = 0;
jointBlock = rigidBodyJoint('jnt_bl','fixed');
setFixedTransform(jointBlock,[-0.0455 -pi 0.06 -pi/2],'dh');
linkBlock.Joint = jointBlock;
addBody(robot,linkBlock,'iiwa_link_ee_kuka')

% Define configuration
homeConfig = homeConfiguration(robot);
startConfig = homeConfig; % Start configuration
cameraConfig = homeConfig; % Configuration during capturing of image

% Joint positions at the start
startJoints = deg2rad([58.2686, 75.3224, 11.7968, 45.9029, -22.1081, -31.2831, -42.3712]);

% Joint positions at the image capture
cameraJoints = deg2rad([-77.26, -38.76, 26.22, 93.29, -56.69, -59.94, 118]);

% Add values to the startConfig and cameraConfig
for i = 1:1:length(startConfig)
    startConfig(i).JointPosition = startJoints(i);
    cameraConfig(i).JointPosition = cameraJoints(i);
end
figure(1)
show(robot,cameraConfig,"Frames","on");
axis equal
shg
view([202 14])
hold on

% Defining the names of links
body0 = robot.BodyNames{1};
bodyEnd = robot.BodyNames{10}; % End effector - 'iiwa_link_ee_kuka'
bodyCamera = robot.BodyNames{11};
bodyBlock = robot.BodyNames{12};

% Getting the transformation matrix
homeTransformation = getTransform(robot,homeConfig,bodyEnd,body0);
startTransformation = getTransform(robot,startConfig,bodyBlock,body0);
cameraTransformation = getTransform(robot,cameraConfig,bodyCamera,body0);

% Defining the transformation matrix betweenthe camera and aruco marker
cameraToAruco = trvec2tform([-0.14195360128424114, -0.06062556383004704,0.3528046636209403]) *...
                 eul2tform(deg2rad([68.70697922863141, -27.847557028831005, -172.95718336855933]),'ZYX');
arucoTransformation = cameraTransformation*cameraToAruco;

% locating the target w.r.t. the aruco marker
targetTransformation = arucoTransformation*trvec2tform([107.95/2 + 25, -107.95/2 - 25, 0]*1e-3);

% Obtaining the start and target positions
start = tform2trvec(startTransformation);
target = tform2trvec(targetTransformation);
scatter3(start(1),start(2),start(3),50,'o')
scatter3(target(1),target(2),target(3),'o')

drawAxes(startTransformation)
drawAxes(arucoTransformation)
drawAxes(targetTransformation)

%% Trajectory

% This is for the inverse kinematics (IK)
ikWeights = [1 1 1 1 1 1]; % Weights for IK - fisrt 3 for orientation, next 3 for position
ikInitGuess = startConfig; % Starting position as initial guess
ik = inverseKinematics('RigidBodyTree',robot); % Define IK - not solve

% Positions
Pstart = start;
Pend = target;
wayPoints = [Pstart;Pend]'; % Alternate set of wayPoints
wayPointVels = 0.1*[0 0 0;0 0 0]';

% Define time
wayPointTimes = [0 10];
ts = 0.005; % Sampling time
trajectoryTime = wayPointTimes(1):ts:wayPointTimes(2);

numWaypoints = size(wayPoints,2); % This si the number of points - 2001 (considering start as well)
% Remove start values while saving

% Define starting and end transformation matrix
T0 = startTransformation;
Tf = targetTransformation;

% Quntic trajectory - Main trajectory generation
[q,qd,qdd] = quinticpolytraj(wayPoints,wayPointTimes,trajectoryTime,"VelocityBoundaryCondition",wayPointVels);
% q - cartesian postion, qd - cart vel., ....

plot3(q(1,:),q(2,:),q(3,:),'r--')

% Interpolating all the transformation matrix inbetween start and end - considering orientation as well
[T,vel,acc] = transformtraj(T0,Tf,wayPointTimes,trajectoryTime);
% T - is where all the trans. matrix is saved - it is a 4x4xn array

jointAngles = zeros(7,numel(trajectoryTime)); % Define an array to store joint angles
for i = 1:numel(trajectoryTime) % loop to perform ik at each step
    % Solve IK
    tgtPose = T(:,:,i); % get the particular T-matrix
    if i == 1
        initialPosition = tform2trvec(tgtPose); % Saving actual inital position 
    end
    [config,info] = ik(bodyBlock,tgtPose,ikWeights,ikInitGuess); % main solving the ik
    % config is a structure that contains the angles at this point of time
    % info - ignore
    % bodyBlock - what point you want to solve the IK -- this is that
    % tgtPose - This is the T matrix - orientation and position
    % ikInitGuess - initial guess

    ikInitGuess = config; % next initial guess is the current configuration
    tableAng = struct2table(config); % config variable is a structure so convert to table first
    jointAngles(:,i) = table2array(tableAng(:,2)); % convert table variable to array and save it in a variable
end
hold off

finalPosition = tform2trvec(tgtPose); % actual Finial position

% Calculate errors at start, end, and also all over
errorInitial = norm(initialPosition-start)
errorFinal = norm(finalPosition-target)
errorAll = tgtPose - targetTransformation

% Write to .txt file
dlmwrite('Durbha_Ananth.txt',jointAngles(:,2:end)','Delimiter',' ', 'precision', '%.4f')

figure(2)

% plot of q - cartesian position
subplot(3,1,1)
plot(trajectoryTime,q(1,:),'r',trajectoryTime,q(2,:),'g',trajectoryTime,q(3,:),'b')
xlabel('Time (s)')
ylabel('Position (m)')
legend('X','Y','Z','Location','best','NumColumns',3)

% plot of qd - cartesian velocity
subplot(3,1,2)
plot(trajectoryTime,qd(1,:),'r',trajectoryTime,qd(2,:),'g',trajectoryTime,qd(3,:),'b')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('X','Y','Z','Location','best')

% plot of qdd - cartesian acc.
subplot(3,1,3)
plot(trajectoryTime,qdd(1,:),'r',trajectoryTime,qdd(2,:),'g',trajectoryTime,qdd(3,:),'b')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
legend('X','Y','Z','Location','best','NumColumns',3)

% Plotting the joint angles
figure(3)
jointAnglesDeg = rad2deg(jointAngles);
plot(trajectoryTime,jointAnglesDeg(1,:))
hold on
plot(trajectoryTime,jointAnglesDeg(2,:))
plot(trajectoryTime,jointAnglesDeg(3,:))
plot(trajectoryTime,jointAnglesDeg(4,:))
plot(trajectoryTime,jointAnglesDeg(5,:))
plot(trajectoryTime,jointAnglesDeg(6,:))
plot(trajectoryTime,jointAnglesDeg(7,:))
xlabel('Time (s)')
ylabel('Joint angles (degrees)')
legend('J1','J2','J3','J4','J5','J6','J7','NumColumns',7)

%% To see the simulation of the motion

% simulateRobot

%% Function

function drawAxes(T)
    p = tform2trvec(T);
    X = T(1:3,1);
    Y = T(1:3,2);
    Z = T(1:3,3);
    quiver3(p(1), p(2), p(3), X(1), X(2), X(3),'r','AutoScaleFactor',0.2,'LineWidth',2)
    quiver3(p(1), p(2), p(3), Y(1), Y(2), Y(3),'g','AutoScaleFactor',0.2,'LineWidth',2)
    quiver3(p(1), p(2), p(3), Z(1), Z(2), Z(3),'b','AutoScaleFactor',0.2,'LineWidth',2)
end