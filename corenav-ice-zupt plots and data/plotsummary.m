icemean = zeros(9832,1);
icezuptmean = zeros(9832,1);

l2mean = zeros(9830,1);
l2zuptmean = zeros(9830,1);

for i = 1:10
    [ice, icezupt] = plotgtsam(i);
    
    [l2, l2zupt] = plotl2(i);
    
    icemean = icemean + ice;
    icezuptmean = icezuptmean + icezupt;
    
    l2mean = l2mean + l2;
    l2zuptmean = l2zuptmean + l2zupt;
    
end

icemean = icemean/10;
icezuptmean = icezuptmean/10;
l2mean = l2mean/10;
l2zuptmean = l2zuptmean/10;

figure();
err = [icemean', icezuptmean', l2mean', l2zuptmean'];
grp = [zeros(1,length(icemean)),ones(1,length(icezuptmean)),2*ones(1,length(l2mean)), 3*ones(1,length(l2zuptmean))];
boxplot(err, grp,'Labels',{'ICE','ICE-Zupt','L2','L2-Zupt'});
set(gca,'TickLabelInterpreter','latex');
ax = gca;
ax.FontSize = 13;
% xlabel('East','Interpreter','Latex');
ylabel('Norm Error (m)','Interpreter','Latex');
title('Mean 3D norm error statistics (m)','Interpreter','Latex')
