%% Initialize %%
clear all
clc
close all

bagMsgs = rosbag('double_camera_2021-04-21-16-59-11.bag');

%% Optitrack values %%

optiMsgs = select(bagMsgs, 'Topic', '/mavros/vision_pose/pose'); 

optiX = timeseries(optiMsgs, "Pose.Position.X");
optiY = timeseries(optiMsgs, "Pose.Position.Y");
optiZ = timeseries(optiMsgs, "Pose.Position.Z");

optiqW = timeseries(optiMsgs, "Pose.Orientation.W");
optiqX = timeseries(optiMsgs, "Pose.Orientation.X");
optiqY = timeseries(optiMsgs, "Pose.Orientation.Y");
optiqZ = timeseries(optiMsgs, "Pose.Orientation.Z");

optiPos = [optiX.Data, optiY.Data, optiZ.Data]';
optiQ = [optiqW.Data, optiqX.Data, optiqY.Data, optiqZ.Data]';

optiLen = optiMsgs.EndTime - optiMsgs.StartTime;
optiN = size(optiX.Data, 1);

optiTime = 0: optiLen/optiN : optiLen-(1/optiN);

%% Double camera values %%
doubleMsgs = select(bagMsgs, 'Topic', '/doppia_realsense/pose');

doubleX = timeseries(doubleMsigs, "Pose.Position.X");
doubleY = timeseries(doubleMsgs, "Pose.Position.Y");
doubleZ = timeseries(doubleMsgs, "Pose.Position.Z");

doublePos = [doubleX.Data, doubleY.Data, doubleZ.Data]';

%Non si può fare perchè l'optitrack ha più punti (freq. maggiore)
% Ropti= quat2rotm(optiQ');
% 
% doublePosOpti = zeros(3,size(doublePos,2));
% 
% for i= 1:1:size(Ropti, 3)
%     doublePosOpti(:,i) = Ropti(:,:,i) * doublePos(:,i);
% end


doublePosOpti = doublePosOpti + [optiX.Data(1); optiY.Data(1); optiZ.Data(1)]; 

doubleqW = timeseries(doubleMsgs, "Pose.Orientation.W");
doubleqX = timeseries(doubleMsgs, "Pose.Orientation.X");
doubleqY = timeseries(doubleMsgs, "Pose.Orientation.Y");
doubleqZ = timeseries(doubleMsgs, "Pose.Orientation.Z");

doubleQ = [doubleqW.Data, doubleqX.Data, doubleqY.Data, doubleqZ.Data]';

doubleLen = doubleMsgs.EndTime - doubleMsgs.StartTime;
doubleN = size(doublePosOpti, 2);

doubleTime = 0: doubleLen/doubleN : doubleLen-(1/doubleN);

%% Master camera values %%
masterMsgs = select(bagMsgs, 'Topic', '/rst265_master/odom/sample');

masterX = timeseries(masterMsgs, "Pose.Pose.Position.X");
masterY = timeseries(masterMsgs, "Pose.Pose.Position.Y");
masterZ = timeseries(masterMsgs, "Pose.Pose.Position.Z");

masterPos = [masterX.Data, masterY.Data, masterZ.Data]';

masterqW = timeseries(masterMsgs, "Pose.Pose.Orientation.W");
masterqX = timeseries(masterMsgs, "Pose.Pose.Orientation.X");
masterqY = timeseries(masterMsgs, "Pose.Pose.Orientation.Y");
masterqZ = timeseries(masterMsgs, "Pose.Pose.Orientation.Z");

masterQ = [masterqW.Data, masterqX.Data, masterqY.Data, masterqZ.Data]';

masterLen = masterMsgs.EndTime - masterMsgs.StartTime;
masterN = size(masterPos, 2);

masterTime = 0: masterLen/masterN : masterLen-(1/masterN);

%% Slave camera values %%
slaveMsgs = select(bagMsgs, 'Topic', '/rst265_slave/odom/sample');

slaveX = timeseries(slaveMsgs, "Pose.Pose.Position.X");
slaveY = timeseries(slaveMsgs, "Pose.Pose.Position.Y");
slaveZ = timeseries(slaveMsgs, "Pose.Pose.Position.Z");

slavePos = [slaveX.Data, slaveY.Data, slaveZ.Data]';

slaveqW = timeseries(slaveMsgs, "Pose.Pose.Orientation.W");
slaveqX = timeseries(slaveMsgs, "Pose.Pose.Orientation.X");
slaveqY = timeseries(slaveMsgs, "Pose.Pose.Orientation.Y");
slaveqZ = timeseries(slaveMsgs, "Pose.Pose.Orientation.Z");

slaveQ = [slaveqW.Data, slaveqX.Data, slaveqY.Data, slaveqZ.Data]';

slaveLen = slaveMsgs.EndTime - slaveMsgs.StartTime;
slaveN = size(slavePos, 2);

slaveTime = 0: slaveLen/slaveN : slaveLen-(1/slaveN);


%% Optitrack Plots %%

figure(1)
subplot(2,1,1)
plot(optiTime, optiPos);
grid on;
title ("Optitrack position");
legend('x', 'y', 'z');

subplot(2,1,2)
plot(optiTime, optiQ);
grid on;
title ("Optitrack quaternion");
legend('w', 'x', 'y', 'z');

%% Double camera Plots %%

figure(2)
subplot(2,1,1)
plot(doubleTime, doublePos);
grid on;
title ("Double camera final position ");
legend('x', 'y', 'z');

subplot(2,1,2)
plot(doubleTime, doubleQ);
grid on;
title ("Double camera final quaternion");
legend('w', 'x', 'y', 'z');

%% Master camera Plots %%

figure(3)
subplot(2,1,1)
plot(masterTime, masterPos);
grid on;
title ("Master camera position ");
legend('x', 'y', 'z');

subplot(2,1,2)
plot(masterTime, masterQ);
grid on;
title ("Master camera quaternion");
legend('w', 'x', 'y', 'z');

%% Slave camera Plots %%

figure(4)
subplot(2,1,1)
plot(slaveTime, slavePos);
grid on;
title ("Slave camera position ");
legend('x', 'y', 'z');

subplot(2,1,2)
plot(slaveTime, slaveQ);
grid on;
title ("Slave camera quaternion");
legend('w', 'x', 'y', 'z');
