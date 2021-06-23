
l2_start = 1093;
ice_start = 1093;
rtk_start = 454;

fid1 = fopen('l2t11.xyz','r');
ECEFl2 = textscan(fid1, '%f %f %f %f','delimiter',' ');
fclose(fid1);

time = ECEFl2{1}(l2_start:end);
X = ECEFl2{2}(l2_start:end);
Y = ECEFl2{3}(l2_start:end);
Z = ECEFl2{4}(l2_start:end);


fid2 = fopen('icet11.xyz','r');
ECEFice = textscan(fid2, '%f %f %f %f','delimiter',' ');
fclose(fid2);

time1 = ECEFice{1}(ice_start:end);
X1 = ECEFice{2}(ice_start:end);
Y1 = ECEFice{3}(ice_start:end);
Z1 = ECEFice{4}(ice_start:end);

% fid3 = fopen('l2zupt.xyz','r');
% ECEFl2zupt = textscan(fid3, '%f %f %f %f','delimiter',' ');
% fclose(fid3);
% 
% time2 = ECEFl2zupt{1}(l2_start:end);
% X2 = ECEFl2zupt{2}(l2_start:end);
% Y2 = ECEFl2zupt{3}(l2_start:end);
% Z2 = ECEFl2zupt{4}(l2_start:end);
% 
% fid4 = fopen('icezupt.xyz','r');
% ECEFicezupt = textscan(fid4, '%f %f %f %f','delimiter',' ');
% fclose(fid4);
% 
% time3 = ECEFicezupt{1}(ice_start:end);
% X3 = ECEFicezupt{2}(ice_start:end);
% Y3 = ECEFicezupt{3}(ice_start:end);
% Z3 = ECEFicezupt{4}(ice_start:end);

ecefl2 = [X,Y,Z];
ecefice = [X1,Y1,Z1];
% ecefl2zupt = [X2,Y2,Z2];
% eceficezupt = [X3,Y3,Z3];

origin = [859154.0695, -4836304.2164, 4055377.5475];

enul2 = zeros(length(X),3);
enuice = zeros(length(X1),3);
% enul2zupt = zeros(length(X2),3);
% enuicezupt = zeros(length(X3),3);

for i = 1:length(X)
    enul2(i, :) = xyz2enu(ecefl2(i,:), origin);
end

for i = 1:length(X1)
    enuice(i, :) = xyz2enu(ecefice(i,:), origin);
end

% for i = 1:length(X2)
%     enul2zupt(i, :) = xyz2enu(ecefl2zupt(i,:), origin);
% end
% 
% for i = 1:length(X3)
%     enuicezupt(i, :) = xyz2enu(eceficezupt(i,:), origin);
% end


load('t11.mat');

gpsTime = gpsECEF.time(rtk_start:end);
rtkX = ENUGPS(1,rtk_start:end)';
rtkY = ENUGPS(2,rtk_start:end)';
rtkZ = ENUGPS(3,rtk_start:end)';


% interpolating all to rtk time to  calculate errors

soll2= interp1(time,enul2,gpsTime,'linear');
solice= interp1(time1,enuice,gpsTime,'linear');
% soll2zupt= interp1(time2,enul2zupt,gpsTime,'linear');
% solicezupt= interp1(time3,enuicezupt,gpsTime,'linear');

solrtk = [rtkX, rtkY, rtkZ];

errl2 = solrtk - soll2;
errice = solrtk - solice;
% errl2zupt = solrtk - soll2zupt;
% erricezupt = solrtk - solicezupt;

norml2 = sqrt(errl2(:,1).^2 + errl2(:,2).^2 + errl2(:,3).^2);
normice = sqrt(errice(:,1).^2 + errice(:,2).^2 + errice(:,3).^2);
% norml2zupt = sqrt(errl2zupt(:,1).^2 + errl2zupt(:,2).^2 + errl2zupt(:,3).^2);
% normicezupt = sqrt(erricezupt(:,1).^2 + erricezupt(:,2).^2 + erricezupt(:,3).^2);

norml20 = norml2(~isnan(norml2));
normice0 = normice(~isnan(normice));
% norml2zupt0 = norml2zupt(~isnan(norml2zupt));
% normicezupt0 = normicezupt(~isnan(normicezupt));

maxl2 = max(norml20)
maxice = max(normice0)
% maxl2zupt = max(norml2zupt0)
% maxicezupt = max(normicezupt0)

rmsl2 = sqrt(sum(norml20.^2)/length(norml20))
rmsice = sqrt(sum(normice0.^2)/length(normice0))
% rmsl2zupt = sqrt(sum(norml2zupt0.^2)/length(norml2zupt0))
% rmsicezupt = sqrt(sum(normicezupt0.^2)/length(normicezupt0))

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



% figure();
% plot(rtkX,rtkY,'r',enu(:,1),enu(:,2),'y',enunoisy(:,1), enunoisy(:,2),'b--',enunoisyzupt(:,1), enunoisyzupt(:,2),'g.-','LineWidth',1.5,'MarkerSize',3)
% set(gca,'TickLabelInterpreter','latex');
% ax = gca;
% ax.FontSize = 13;
% lgd2 = legend('RTKlib','ICE','ICE-noisy','ICE-noisyzupt','Interpreter','Latex');
% xlabel('East (m)','Interpreter','Latex');
% ylabel('North (m)','Interpreter','Latex');
% title('2D trajectory (m) - t9 ','Interpreter','Latex')
% lgd2.FontSize = 13;
% 
% figure();
% plot3(rtkX,rtkY,rtkZ,'b',enul2(:,1),enul2(:,2),enul2(:,3),'r',enuice(:,1),enuice(:,2),enuice(:,3),'k',...
%     enuicezupt(:,1),enuicezupt(:,2),enuicezupt(:,3),'c',enul2zupt(:,1),enul2zupt(:,2),enul2zupt(:,3),'g.-','LineWidth',1.5,'MarkerSize',3)
% set(gca,'TickLabelInterpreter','latex');

figure();
plot3(rtkX,rtkY,rtkZ,'b',enuice(:,1),enuice(:,2),enuice(:,3),'k',...
    enul2(:,1),enul2(:,2),enul2(:,3),'r','LineWidth',1.5,'MarkerSize',3)
set(gca,'TickLabelInterpreter','latex');
ax = gca;
ax.FontSize = 13;
lgd2 = legend('RTKlib','ice','l2','Interpreter','Latex');
xlabel('East','Interpreter','Latex');
ylabel('North','Interpreter','Latex');
zlabel('Up','Interpreter','Latex');
title('3D trajectory - t10 ','Interpreter','Latex')
lgd2.FontSize = 13;
