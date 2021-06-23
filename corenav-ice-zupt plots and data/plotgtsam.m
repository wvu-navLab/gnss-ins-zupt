
% function [normicenoisy0, normicenoisyzupt0] =  plotgtsam(n)
n = 3;
ice_start = 550;
if n ==1
    icenoisy_start = 512;
    icenoisyzupt_start = 512;
elseif n ==2 
    icenoisy_start = 508;
    icenoisyzupt_start = 504;
elseif n ==3 
    icenoisy_start = 495;
    icenoisyzupt_start = 495;
elseif n ==4 
    icenoisy_start = 499;
    icenoisyzupt_start = 499;
elseif n ==5 
    icenoisy_start = 506;
    icenoisyzupt_start = 506;
end
rtk_start = 454;

fid = fopen('ice.xyz','r');
ECEF = textscan(fid, '%f %f %f %f','delimiter',' ');
fclose(fid);

time = ECEF{1}(ice_start:end);
X = ECEF{2}(ice_start:end);
Y = ECEF{3}(ice_start:end);
Z = ECEF{4}(ice_start:end);


fid2 = fopen('icenoisy3.xyz','r');
ECEFnoisy = textscan(fid2, '%f %f %f %f','delimiter',' ');
fclose(fid2);

time2 = ECEFnoisy{1}(icenoisy_start:end);
X2 = ECEFnoisy{2}(icenoisy_start:end);
Y2 = ECEFnoisy{3}(icenoisy_start:end);
Z2 = ECEFnoisy{4}(icenoisy_start:end);


fid3 = fopen('icenoisyzupt3.xyz','r');
ECEFnoisyzupt = textscan(fid3, '%f %f %f %f','delimiter',' ');
fclose(fid3);

time3 = ECEFnoisyzupt{1}(icenoisyzupt_start:end);
X3 = ECEFnoisyzupt{2}(icenoisyzupt_start:end);
Y3 = ECEFnoisyzupt{3}(icenoisyzupt_start:end);
Z3 = ECEFnoisyzupt{4}(icenoisyzupt_start:end);


ecef = [X,Y,Z];
ecefnoisy = [X2,Y2,Z2];
ecefnoisyzupt = [X3,Y3,Z3];


origin = [859153.0167; -4836303.7245; 4055378.4991];

enu = zeros(length(X),3);
enunoisy = zeros(length(X2),3);
enunoisyzupt = zeros(length(X3),3);

for i = 1:length(X)
    enu(i, :) = xyz2enu(ecef(i,:), origin);
end

for i = 1:length(X2)
    enunoisy(i, :) = xyz2enu(ecefnoisy(i,:), origin);
end

for i = 1:length(X3)
    enunoisyzupt(i, :) = xyz2enu(ecefnoisyzupt(i,:), origin);
end


load('t10lastCN.mat');

gpsTime = gpsECEF.time(rtk_start:end);
rtkX = ENUGPS(1,rtk_start:end)';
rtkY = ENUGPS(2,rtk_start:end)';
rtkZ = ENUGPS(3,rtk_start:end)';


% corenav_time = double(timeSecCN(corenav_start:end)) + 1e-9*double(timeNsecCN(corenav_start:end));
% 
% timediff = gpsTime(1) - corenav_time(1);
% corenavtime = corenav_time + timediff*(ones(length(corenav_time),1));

% cnX = (-1)*Pos_yCN(corenav_start:end);
% cnY = Pos_xCN(corenav_start:end);
% cnZ = Pos_zCN(corenav_start:end-3);
% corenavtime = corenavtime(1:end-3);

% interpolating all to rtk time to  calculate errors
solice = interp1(time,enu,gpsTime,'linear');
solicenoisy= interp1(time2,enunoisy,gpsTime,'linear');
solicenoisyzupt= interp1(time3,enunoisyzupt,gpsTime,'linear');


solrtk = [rtkX, rtkY, rtkZ];

errice = solrtk - solice;
erricenoisy = solrtk - solicenoisy;
erricenoisyzupt = solrtk - solicenoisyzupt;

normice = sqrt(errice(:,1).^2 + errice(:,2).^2 + errice(:,3).^2); 
normicenoisy = sqrt(erricenoisy(:,1).^2 + erricenoisy(:,2).^2 + erricenoisy(:,3).^2);
normicenoisyzupt = sqrt(erricenoisyzupt(:,1).^2 + erricenoisyzupt(:,2).^2 + erricenoisyzupt(:,3).^2);

normicenoisy0 = normicenoisy(~isnan(normicenoisy));
normicenoisyzupt0 = normicenoisyzupt(~isnan(normicenoisyzupt));

maxice = max(normice)
maxicenoisy = max(normicenoisy0)
maxicenoisyzupt = max(normicenoisyzupt0)

rmsice = sqrt(sum(normice.^2)/length(normice))
rmsicenoisy = sqrt(sum(normicenoisy0.^2)/length(normicenoisy0))
rmsicenoisyzupt = sqrt(sum(normicenoisyzupt0.^2)/length(normicenoisyzupt0))

% 
% figure();
% err = [normice',normicenoisy0',normicenoisyzupt0'];
% grp = [zeros(1,length(normice)),ones(1,length(normicenoisy0)),2*ones(1,length(normicenoisyzupt0))];
% boxplot(err, grp,'Labels',{'ICE','ICE-noisy','ICE-noisy-zupt'});
% set(gca,'TickLabelInterpreter','latex');
% ax = gca;
% ax.FontSize = 13;
% % xlabel('East','Interpreter','Latex');
% ylabel('Norm Error (m)','Interpreter','Latex');
% %title('3D Norm Error (m) - t9 ','Interpreter','Latex')

% 
% figure();
% err = [errice(:,3)',erricenoisy(:,3)',erricenoisyzupt(:,3)'];
% grp = [zeros(1,size(errice,1)),ones(1,size(erricenoisy,1)),2*ones(1,size(erricenoisyzupt,1))];
% boxplot(err, grp,'Labels',{'ICE','ICE-noisy','ICE-noisy-zupt'});
% set(gca,'TickLabelInterpreter','latex');
% ax = gca;
% ax.FontSize = 13;
% % xlabel('East','Interpreter','Latex');
% ylabel('Up Error (m)','Interpreter','Latex');
% %title('3D Norm Error (m) - t9 ','Interpreter','Latex')


% 
% figure();
% plot(rtkX,rtkY,'r',enunoisy(:,1), enunoisy(:,2),'b--',enunoisyzupt(:,1), enunoisyzupt(:,2),'g.-','LineWidth',1.5,'MarkerSize',3)
% set(gca,'TickLabelInterpreter','latex');
% ax = gca;
% ax.FontSize = 13;
% lgd2 = legend('RTKlib','ICE-noisy','ICE-noisyzupt','Interpreter','Latex');
% xlabel('East (m)','Interpreter','Latex');
% ylabel('North (m)','Interpreter','Latex');
% title('2D trajectory (m) - t9 ','Interpreter','Latex')
% lgd2.FontSize = 13;
% 
figure();
plot3(rtkX,rtkY,rtkZ,'r',enu(:,1),enu(:,2),enu(:,3),'k',enunoisy(:,1),enunoisy(:,2),enunoisy(:,3),'b--',enunoisyzupt(:,1),enunoisyzupt(:,2),enunoisyzupt(:,3),'g.-','LineWidth',1.5,'MarkerSize',3)
set(gca,'TickLabelInterpreter','latex');
ax = gca;
ax.FontSize = 13;
lgd2 = legend('RTKlib','ICE','ICE-noisy','ICE-noisyzupt','Interpreter','Latex');
xlabel('East','Interpreter','Latex');
ylabel('North','Interpreter','Latex');
zlabel('Up','Interpreter','Latex');
title('3D trajectory - t10 ','Interpreter','Latex')
lgd2.FontSize = 13;

% end

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

