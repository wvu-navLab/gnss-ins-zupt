get_stats

figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;


boxplot(l2_d1lq_rh,repmat(1,length(bce_d1lq_rh),1),'position',1, 'symbol','')
boxplot(dcs_d1lq_rh,repmat(2,length(bce_d1lq_rh),1),'position',1.25, 'symbol','')
boxplot(mm_d1lq_rh,repmat(3,length(bce_d1lq_rh),1),'position',1.5, 'symbol','')
boxplot(bce_d1lq_rh,repmat(4,length(bce_d1lq_rh),1),'position',1.75, 'symbol','')

boxplot(l2_d2lq_rh,repmat(5,length(bce_d2lq_rh),1),'position',3, 'symbol','')
boxplot(dcs_d2lq_rh,repmat(6,length(bce_d2lq_rh),1),'position',3.25, 'symbol','')
boxplot(mm_d2lq_rh,repmat(7,length(bce_d2lq_rh),1),'position',3.5, 'symbol','')
boxplot(bce_d2lq_rh,repmat(8,length(bce_d2lq_rh),1),'position',3.75, 'symbol','')

boxplot(l2_d3lq_rh,repmat(9,length(bce_d3lq_rh),1),'position',5, 'symbol','')
boxplot(dcs_d3lq_rh,repmat(10 ,length(bce_d3lq_rh),1),'position',5.25, 'symbol','')
boxplot(mm_d3lq_rh,repmat(11,length(bce_d3lq_rh),1),'position',5.5, 'symbol','')
boxplot(bce_d3lq_rh,repmat(12,length(bce_d3lq_rh),1),'position',5.75, 'symbol','')

positions = [1 1.25 1.5 1.75 3 3.25 3.5 3.75 5 5.25 5.5 5.75];
color = ['k', 'r', 'b', 'g'];
color = repmat(color,1,3);

set(gca,'xtick',[mean(positions(1:4)) mean(positions(5:8)) mean(positions(9:12)) ])
set(gca,'xticklabel',{'Collect 1','Collect 2','Collect 3'})

set(findobj(gca,'type','line'),'linew',2)

h = findobj(gca,'Tag','Box');
for j=1:length(h)
   p(j) = patch(get(h(j),'XData'),get(h(j),'YData'),color(j),'FaceAlpha',.75);
   pp(j) = p(j);
   uistack(p(j),'bottom');
end

c = get(gca, 'Children');
set(findobj(gca,'type','line'),'linew',2)

lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'k');

l = length(c);
hleg1 = legend(fliplr(pp(1:4)), 'L2', 'DCS', 'MM', 'BCE','Location','northwest' );
set(gca,'linewidth', 2, 'fontsize', 16)
grid on;
ylabel('Horizontal RSOS Positioning Error ( m. )')
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontWeight', 'bold')
set(findall(gcf,'-property','FontSize'),'FontSize',24)
axis tight
grid on;

ylim([0,20])
xlim([positions(1)-0.5, positions(end)+0.5])


% Remove whitespace around plot
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

saveas(gcf, 'lq_boxplot','png');

close;
