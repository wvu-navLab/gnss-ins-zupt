figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;

res = load('../hq/bce/01/iter_0.residuals');


S = scatter(res(1:2:end,1),res(1:2:end,2),30,'MarkerEdgeColor','k',...
              'MarkerFaceColor','k',...
              'LineWidth',1.5);


weight_out = 0.164493;
weight_in = 0.835487;
mean_out = [-0.42729 0.000537972];
mean_in =  [0.0210119 -6.65043e-05];
cov_out = [22.8517 -0.00716079; -0.00716079 0.000768311];
cov_in = [1.61554 -0.00120016; -0.00120016 0.000168894];

L(1) = error_ellipse(cov_out,mean_out,0.95);
L(2) = error_ellipse(cov_in, mean_in, 0.95);

color = ['r','g'];
P(1) = patch(get(L(1),'XData'),get(L(1),'YData'),color(1),'FaceAlpha',.75);
P(2) = patch(get(L(2),'XData'),get(L(2),'YData'),[1,1,1],'FaceAlpha',1.0);
set(get(get(P(2), 'Annotation') , 'LegendInformation'), 'IconDisplayStyle', 'off');
P(3) = patch(get(L(2),'XData'),get(L(2),'YData'),color(2),'FaceAlpha',.75);

delete(L(1));
% uistack(S,'top')
uistack(P(1),'bottom');

delete(L(2));
uistack(P(2),'top');
uistack(P(3),'top');

hold off;
alpha(S,0.55)

legend([S,P(1),P(3)],'State Estimation Residuals', 'Measurement Covariance Component 1', 'Measurement Covariance Component 2');
ylabel('Carrier-Phase Residuals ( m. )')
xlabel('Pseudorange Residuals ( m. )')
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontWeight', 'bold')
set(findall(gcf,'-property','FontSize'),'FontSize',24)

% Remove whitespace around plot
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

grid;
set(findobj(gca,'type','line'),'linew',2)

saveas(gcf, 'hq_residuals','png')

close;
