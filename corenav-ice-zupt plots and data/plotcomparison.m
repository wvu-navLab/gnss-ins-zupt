function plotcomparison(n)

icenoisy_start = 505;
l2noisy_start = 552;
rtk_start = 454;

fid2 = fopen(strcat('icenoisy', int2str(n),'.xyz'),'r');
ECEFicenoisy = textscan(fid2, '%f %f %f %f','delimiter',' ');
fclose(fid2);

time2 = ECEFicenoisy{1}(icenoisy_start:end);
X2 = ECEFicenoisy{2}(icenoisy_start:end);
Y2 = ECEFicenoisy{3}(icenoisy_start:end);
Z2 = ECEFicenoisy{4}(icenoisy_start:end);

fid3 = fopen(strcat('l2noisy', int2str(n),'.xyz'),'r');
ECEFl2noisy = textscan(fid3, '%f %f %f %f','delimiter',' ');
fclose(fid3);

time3 = ECEFl2noisy{1}(l2noisy_start:end);
X3 = ECEFl2noisy{2}(l2noisy_start:end);
Y3 = ECEFl2noisy{3}(l2noisy_start:end);
Z3 = ECEFl2noisy{4}(l2noisy_start:end);

eceficenoisy = [X2,Y2,Z2];
ecefl2noisy = [X3,Y3,Z3];

origin = [859153.0167; -4836303.7245; 4055378.4991];

enuicenoisy = zeros(length(X2),3);
enul2noisy = zeros(length(X3),3);


for i = 1:length(X2)
    enuicenoisy(i, :) = xyz2enu(eceficenoisy(i,:), origin);
end

for i = 1:length(X3)
    enul2noisy(i, :) = xyz2enu(ecefl2noisy(i,:), origin);
end


load('t10lastCN.mat');

gpsTime = gpsECEF.time(rtk_start:end);
rtkX = ENUGPS(1,rtk_start:end)';
rtkY = ENUGPS(2,rtk_start:end)';
rtkZ = ENUGPS(3,rtk_start:end)';


% interpolating all to rtk time to  calculate errors

solicenoisy= interp1(time2,enuicenoisy,gpsTime,'linear');
soll2noisy= interp1(time3,enul2noisy,gpsTime,'linear');

solrtk = [rtkX, rtkY, rtkZ];

erricenoisy = solrtk - solicenoisy;
errl2noisyzupt = solrtk - soll2noisy;

normicenoisy = sqrt(erricenoisy(:,1).^2 + erricenoisy(:,2).^2 + erricenoisy(:,3).^2);
norml2noisy = sqrt(errl2noisyzupt(:,1).^2 + errl2noisyzupt(:,2).^2 + errl2noisyzupt(:,3).^2);

normicenoisy0 = normicenoisy(~isnan(normicenoisy));
norml2noisy0 = norml2noisy(~isnan(norml2noisy));

maxicenoisy = max(normicenoisy0);
maxl2noisy = max(norml2noisy0);

rmsicenoisy = sqrt(sum(normicenoisy0.^2)/length(normicenoisy0))
rmsl2noisy = sqrt(sum(norml2noisy0.^2)/length(norml2noisy0))

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
figure();
plot3(rtkX,rtkY,rtkZ,'r',enuicenoisy(:,1),enuicenoisy(:,2),enuicenoisy(:,3),'b--',enul2noisy(:,1),enul2noisy(:,2),enul2noisy(:,3),'g.-','LineWidth',1.5,'MarkerSize',3)
set(gca,'TickLabelInterpreter','latex');
ax = gca;
ax.FontSize = 13;
lgd2 = legend('RTKlib','ICE-noisy','l2-noisy','Interpreter','Latex');
xlabel('East','Interpreter','Latex');
ylabel('North','Interpreter','Latex');
zlabel('Up','Interpreter','Latex');
title('3D trajectory - t10 ','Interpreter','Latex')
lgd2.FontSize = 13;

end