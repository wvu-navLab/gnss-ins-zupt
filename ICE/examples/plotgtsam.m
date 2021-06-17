

ice_start = 550;
icezupt_start = 550;
corenav_start = 2273;
rtk_start = 454;

fid = fopen('ice.xyz','r');
ECEF = textscan(fid, '%f %f %f %f','delimiter',' ');
fclose(fid);

time = ECEF{1}(ice_start:end);
X = ECEF{2}(ice_start:end);
Y = ECEF{3}(ice_start:end);
Z = ECEF{4}(ice_start:end);


fid2 = fopen('ice_zupt2.xyz','r');
ECEFzupt = textscan(fid, '%f %f %f %f','delimiter',' ');
fclose(fid2);

time2 = ECEFzupt{1}(icezupt_start:end);
X2 = ECEFzupt{2}(icezupt_start:end);
Y2 = ECEFzupt{3}(icezupt_start:end);
Z2 = ECEFzupt{4}(icezupt_start:end);


ecef = [X,Y,Z];
ecefzupt = [X2,Y2,Z2];

origin = [859153.0167; -4836303.7245; 4055378.4991];

enu = zeros(length(X),3);
enuzupt = zeros(length(X2),3);

for i = 1:length(X)
    enu(i, :) = xyz2enu(ecef(i,:), origin);
end

for i = 1:length(X2)
    enuzupt(i, :) = xyz2enu(ecefzupt(i,:), origin);
end


load('t10lastCN.mat');

gpsTime = gpsECEF.time(rtk_start:end);
rtkX = ENUGPS(1,rtk_start:end)';
rtkY = ENUGPS(2,rtk_start:end)';
rtkZ = ENUGPS(3,rtk_start:end)';


corenav_time = double(timeSecCN(corenav_start:end)) + 1e-9*double(timeNsecCN(corenav_start:end));

timediff = gpsTime(1) - corenav_time(1);
corenavtime = corenav_time + timediff*(ones(length(corenav_time),1));

cnX = (-1)*Pos_yCN(corenav_start:end);
cnY = Pos_xCN(corenav_start:end);
cnZ = Pos_zCN(corenav_start:end-3);
corenavtime = corenavtime(1:end-3);

% interpolating all to rtk time to  calculate errors
solice = interp1(time,enu,gpsTime,'linear');
solicezupt = interp1(time2,enuzupt,gpsTime,'linear');
solcn = interp1(corenavtime,[cnX,cnY,cnZ],gpsTime,'linear');


solrtk = [rtkX, rtkY, rtkZ];

errice = solrtk - solice;
erricezupt = solrtk - solicezupt;
errcn = solrtk - solcn;

normice = sqrt(errice(:,1).^2 + errice(:,2).^2 + errice(:,3).^2); 
normicezupt = sqrt(erricezupt(:,1).^2 + erricezupt(:,2).^2 + erricezupt(:,3).^2);
normcn = sqrt(errcn(:,1).^2 + errcn(:,2).^2 + errcn(:,3).^2);

maxice = max(normice);
maxicezupt = max(normicezupt);
maxcn = max(normcn);

rmsice = sqrt(sum(normice.^2)/length(normice));
rmsicezupt = sqrt(sum(normicezupt.^2)/length(normicezupt));
rmscn = sqrt(sum(normcn.^2)/length(normcn));

figure();
err = [normice',normicezupt'];
grp = [zeros(1,length(normice)),ones(1,length(normicezupt))];
boxplot(err, grp,'Labels',{'ICE','ICE-ZUPT'});
set(gca,'TickLabelInterpreter','latex');
ax = gca;
ax.FontSize = 13;
% xlabel('East','Interpreter','Latex');
ylabel('Norm Error (m)','Interpreter','Latex');
%title('3D Norm Error (m) - t9 ','Interpreter','Latex')


figure();
err = [errice(:,3)',erricezupt(:,3)'];
grp = [zeros(1,size(errice,1)),ones(1,size(erricezupt,1))];
boxplot(err, grp,'Labels',{'ICE','ICE-ZUPT'});
set(gca,'TickLabelInterpreter','latex');
ax = gca;
ax.FontSize = 13;
% xlabel('East','Interpreter','Latex');
ylabel('Up Error (m)','Interpreter','Latex');
%title('3D Norm Error (m) - t9 ','Interpreter','Latex')



figure();
plot(rtkX,rtkY,'r',enu(:,1),enu(:,2),'y',enuzupt(:,1), enuzupt(:,2),'b--',(-1)*Pos_yCN, Pos_xCN,'c','LineWidth',1.5,'MarkerSize',3)
set(gca,'TickLabelInterpreter','latex');
ax = gca;
ax.FontSize = 13;
lgd2 = legend('RTKlib','ICE','ICE-ZUPT','CoreNav','Interpreter','Latex');
xlabel('East (m)','Interpreter','Latex');
ylabel('North (m)','Interpreter','Latex');
title('2D trajectory (m) - t9 ','Interpreter','Latex')
lgd2.FontSize = 13;

% figure();
% plot3(rtkX,rtkY,rtkZ,'r',enu(:,1),enu(:,2),enu(:,3),'b',enuzupt(:,1),enuzupt(:,2),enuzupt(:,3),'k',(-1)*Pos_yCN, Pos_xCN, (-1)*Pos_zCN(1:length(Pos_xCN)),'g','LineWidth',1.5,'MarkerSize',3)
% set(gca,'TickLabelInterpreter','latex');
% ax = gca;
% ax.FontSize = 13;
% lgd2 = legend('RTKlib','ICE','ICE ZUPT','CoreNav','Interpreter','Latex');
% xlabel('East','Interpreter','Latex');
% ylabel('North','Interpreter','Latex');
% zlabel('Up','Interpreter','Latex');
% title('3D trajectory - t9 ','Interpreter','Latex')
% lgd2.FontSize = 13;

% rearVelTime = [tTodom, rearLeftVel(1:end-1)];
% 
% rearVelZupt = [];
% 
% for i = 1:size(rearVelTime,1)
%     if rearVelTime(i,2) == 0
%         rearVelZupt = [rearVelZupt;rearVelTime(i,:)];
%     end
% end
% 
% odom_start_time = tTodom(455);
% 
% timediff = gtsam_time - tTodom(455);
% 
% zuptTimes = rearVelZupt(:,1);
% 
% zuptTimesgtsam = zuptTimes + timediff*(ones(length(zuptTimes),1));
% 
% % zuptTimesgtsamrnd = roundn(zuptTimesgtsam,5);
% 
% figure; scatter(time,0.5*ones(length(time),1),'bs');
% hold on;
% scatter(zuptTimesgtsam,0.5*ones(length(zuptTimesgtsam),1),'ro');

