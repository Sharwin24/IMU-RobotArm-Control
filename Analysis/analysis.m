close all;
%% Getting RosBags
bag = rosbag('../Data/IMUData.bag');
elbowIMUBag = select(bag,'Topic','/elbow_imu');
robotArmCommand = select(bag,'Topic','/robot_arm_command');
elbowIMUStructs = readMessages(elbowIMUBag, 'DataFormat', 'struct');
elbowIMUStructs{1};
robotArmCommandStructs = readMessages(robotArmCommand, 'DataFormat', 'struct');
robotArmCommandStructs{1};
fprintf("Successfully Read Bag files and created Structs\n");
%% Gathering Data
imuTimeSeconds =  cellfun(@(m) double(m.Vectornav_.Stamp.Sec), elbowIMUStructs);
imuTimeNSeconds =  cellfun(@(m) double(m.Vectornav_.Stamp.Nsec), elbowIMUStructs);
imuTime = imuTimeSeconds + imuTimeNSeconds*1e-9;
imuTime = imuTime - imuTime(1);
imuTime = imuTime - 5; % Subtract Calibration Period
cmdTimeSeconds = cellfun(@(m) double(m.RobotArmCommand_.Stamp.Sec), robotArmCommandStructs);
cmdTimeNSeconds = cellfun(@(m) double(m.RobotArmCommand_.Stamp.Nsec), robotArmCommandStructs);
cmdTime = cmdTimeSeconds + cmdTimeNSeconds*1e-9;
cmdTime = cmdTime - cmdTime(1);
imuYaw = cellfun(@(m) double(m.Yaw), elbowIMUStructs);
imuYaw = rad2deg(imuYaw);
link1Angle = cellfun(@(m) double(m.Joint1), robotArmCommandStructs);
link2Angle = cellfun(@(m) double(m.Joint2), robotArmCommandStructs);
%% Plotting Data
figure;
hold on;
plot(imuTime, imuYaw, 'DisplayName', "Elbow IMU Yaw");
plot(cmdTime, link2Angle, "DisplayName", "Link 2");
xlabel('Time (s)');
ylabel('Angular Positon (deg)')
title('IMU angle vs Link Angle ');
legend();
