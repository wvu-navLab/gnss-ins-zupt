clear
format long
% bag=rosbag('data/t9/2020-11-05-13-00-06.bag');
bag=rosbag('E:/ashpile11-05-2020/t5/2020-11-05-12-17-37.bag');
% bSel = select(bag,'Topic','/imu/data');
bSel = select(bag,'Topic','/imu_filtered');
bSel2=select(bag,'Topic','/dead_reckoning/odometry');
bSel3=select(bag,'Topic','/dead_reckoning/joint_states');
bSel4=select(bag,'Topic','/core_nav/core_nav/cn_odom');
bSel7=select(bag,'Topic','/core_nav/core_nav/slip');
bSel5=select(bag,'Topic','/core_nav/core_nav/gp_input');
bSel6=select(bag,'Topic','/core_nav/core_nav/gp_result');
bSel8=select(bag,'Topic','/core_nav/core_nav/stop_cmd');
msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructsOd= readMessages(bSel2, 'DataFormat','struct');
msgStructsWheel=readMessages(bSel3, 'DataFormat','struct');
msgStructsCN=readMessages(bSel4, 'DataFormat','struct');
msgStructsSlip=readMessages(bSel7,'DataFormat','struct');
msgStructsGP=readMessages(bSel5,'DataFormat','struct');
msgStructsGPres=readMessages(bSel6,'DataFormat','struct');
stopcmd=readMessages(bSel8,'DataFormat','struct');
% writematrix(msgStructsGP{4, 1}.TimeArray , 'gaussianProcess\slipForecastPython\slipVal.csv','Delimiter','comma');
for i=1:length(msgStructsSlip)
timeSecSlip(i,1)=msgStructsSlip{i,1}.Header.Stamp.Sec;
timeNsecSlip(i,1)=msgStructsSlip{i,1}.Header.Stamp.Nsec;
Slip(i,1)=msgStructsSlip{i,1}.Point.X;
insVel(i,1)=msgStructsSlip{i,1}.Point.Y;
WOvel(i,1)=msgStructsSlip{i,1}.Point.Z;
end
% for i=1:length(msgStructsGPS)
% timeSecGPS(i,1)=msgStructsGPS{i,1}.Header.Stamp.Sec;
% timeNsecGPS(i,1)=msgStructsGPS{i,1}.Header.Stamp.Nsec;
% X(i,1)=msgStructsGPS{i,1}.X;
% Y(i,1)=msgStructsGPS{i,1}.Y;
% Z(i,1)=msgStructsGPS{i,1}.Z;
% % Speed(i,1)=msgStructsGPS{i,1}.Speed;
% % Err(i,1)=msgStructsGPS{i,1}.Err;
% % ErrHorz(i,1)=msgStructsGPS{i,1}.ErrHorz;
% % ErrVert(i,1)=msgStructsGPS{i,1}.ErrVert;
% % PositionCovariance(:,i)=msgStructsGPS{i,1}.PositionCovariance(:,1);
% end

for i=1:length(msgStructs)
timeSecImu(i,1)=msgStructs{i,1}.Header.Stamp.Sec;
timeNsecImu(i,1)=msgStructs{i,1}.Header.Stamp.Nsec;
Ax(i,1)=msgStructs{i,1}.LinearAcceleration.X;
Ay(i,1)=msgStructs{i,1}.LinearAcceleration.Y;
Az(i,1)=msgStructs{i,1}.LinearAcceleration.Z;
Gx(i,1)=msgStructs{i,1}.AngularVelocity.X;
Gy(i,1)=msgStructs{i,1}.AngularVelocity.Y;
Gz(i,1)=msgStructs{i,1}.AngularVelocity.Z;
end
for i=1:length(msgStructsOd)
timeSecOdom(i,1)=msgStructsOd{i,1}.Header.Stamp.Sec;
timeNsecOdom(i,1)=msgStructsOd{i,1}.Header.Stamp.Nsec;
Pos_x(i,1)=msgStructsOd{i,1}.Pose.Pose.Position.X;
Pos_y(i,1)=msgStructsOd{i,1}.Pose.Pose.Position.Y;
Pos_z(i,1)=msgStructsOd{i,1}.Pose.Pose.Position.Z;
Orien_x(i,1)=msgStructsOd{i,1}.Pose.Pose.Orientation.X;
Orien_y(i,1)=msgStructsOd{i,1}.Pose.Pose.Orientation.Y;
Orien_z(i,1)=msgStructsOd{i,1}.Pose.Pose.Orientation.Z;
Orien_w(i,1)=msgStructsOd{i,1}.Pose.Pose.Orientation.W;
lin_x(i,1)=msgStructsOd{i,1}.Twist.Twist.Linear.X;
lin_y(i,1)=msgStructsOd{i,1}.Twist.Twist.Linear.Y;
% lin_z(i,1)=msgStructsOd{i,1}.Twist.Twist.Linear.Z;
% ang_x(i,1)=msgStructsOd{i,1}.Twist.Twist.Angular.X;
% ang_y(i,1)=msgStructsOd{i,1}.Twist.Twist.Angular.Y;
ang_z(i,1)=msgStructsOd{i,1}.Twist.Twist.Angular.Z;
end
for i=1:length(msgStructsCN)
timeSecCN(i,1)=msgStructsCN{i,1}.Header.Stamp.Sec;
timeNsecCN(i,1)=msgStructsCN{i,1}.Header.Stamp.Nsec;
Pos_xCN(i,1)=msgStructsCN{i,1}.Pose.Pose.Position.X;
Pos_yCN(i,1)=msgStructsCN{i,1}.Pose.Pose.Position.Y;
Pos_zCN(i,1)=msgStructsCN{i,1}.Pose.Pose.Position.Z;
Orien_xCN(i,1)=msgStructsCN{i,1}.Pose.Pose.Orientation.X;
Orien_yCN(i,1)=msgStructsCN{i,1}.Pose.Pose.Orientation.Y;
Orien_zCN(i,1)=msgStructsCN{i,1}.Pose.Pose.Orientation.Z;
Orien_wCN(i,1)=msgStructsCN{i,1}.Pose.Pose.Orientation.W;
lin_xCN(i,1)=msgStructsCN{i,1}.Twist.Twist.Linear.X;
lin_yCN(i,1)=msgStructsCN{i,1}.Twist.Twist.Linear.Y;
lin_zCN(i,1)=msgStructsCN{i,1}.Twist.Twist.Linear.Z;
ang_xCN(i,1)=msgStructsCN{i,1}.Twist.Twist.Angular.X;
ang_yCN(i,1)=msgStructsCN{i,1}.Twist.Twist.Angular.Y;
ang_zCN(i,1)=msgStructsCN{i,1}.Twist.Twist.Angular.Z;
end
for i=1:length(msgStructsWheel)
timeSecWheel(i,1)=msgStructsWheel{i,1}.Header.Stamp.Sec;
timeNsecWheel(i,1)=msgStructsWheel{i,1}.Header.Stamp.Nsec;   
frontLeftPos(i,1)=msgStructsWheel{i,1}.Position(1);
frontRightPos(i,1)=msgStructsWheel{i,1}.Position(2);
rearLeftPos(i,1)=msgStructsWheel{i,1}.Position(3);
rearRightPos(i,1)=msgStructsWheel{i,1}.Position(4);
frontLeftVel(i,1)=msgStructsWheel{i,1}.Velocity(1);
frontRightVel(i,1)=msgStructsWheel{i,1}.Velocity(2);
rearLeftVel(i,1)=msgStructsWheel{i,1}.Velocity(3);
rearRightVel(i,1)=msgStructsWheel{i,1}.Velocity(4);
end
tNimu=double(timeNsecImu);
tSimu=double(timeSecImu);
tTimu=tSimu+(tNimu*1e-9);

tNodom=double(timeNsecOdom);
tSodom=double(timeSecOdom);
tTodom=tSodom+(tNodom*1e-9);

% tNGPS=double(timeNsecGPS);
% tSGPS=double(timeSecGPS);
% tTGPS=tSGPS+(tNGPS*1e-9);
clear bag* bSel* msg*
%  save test1430.mat timeSecWheel timeNsecWheel frontLeftPos frontRightPos rearLeftPos rearRightPos frontLeftVel frontRightVel rearLeftVel rearRightVel timeSecImu timeNsecImu tTimu Ax Ay Az Gx Gy Gz timeSecOdom timeNsecOdom tTodom Pos_x Pos_y Pos_z Orien_x Orien_y Orien_z Orien_w lin_x lin_y lin_z ang_x ang_y ang_z
