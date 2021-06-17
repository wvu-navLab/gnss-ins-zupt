figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;



%% Iteration 4
res = load('../lq/bce/01/iter_4.residuals');


S = scatter(res(1:2:end,1),res(1:2:end,2),30,'MarkerEdgeColor','k',...
              'MarkerFaceColor','k',...
              'LineWidth',1.5);

weights = [ 0.00312277    0.850688   0.0190912   0.0413032   0.0840677 0.000606844   0.0010652 ];

mean_1 = [-4.42252 -2.36123];
mean_2 = [-0.0455489 -0.000100445];
mean_3 = [-60.7555 0.008600];
mean_4 = [92.1898 -0.013034];
mean_5 = [-2.47776 0.032813];
mean_6 = [1.12509 1.1624];
mean_7 = [2.20347 0.841802];

cov_1 = [5.26881 0.066343;
    0.066343  0.06647];

cov_2  = [28.7695 -0.00867713;
    -0.00867713 6.52666e-05];

cov_3 = [120.197  0.0946214;
    0.0946214 0.00278443];

cov_4 = [192.3 0.00334264;
    0.00334264 0.00103844];

cov_5 = [17.2005 0.00678057;
    0.00678057  0.0072604];

cov_6 = [32.1174 -1.86452;
    -1.86452 0.480076];

cov_7 = [3.79718 0.0662516;
    0.0662516 0.0535907];

min_x = -91;
min_y = -3;
max_x = 133;
max_y = 3;

xlim([min_x, max_x]);

color = ['y','m','c','r','g','b','w'];
for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));
  L(i) = error_ellipse( eval(c),eval(m),0.95 );
  P(i) = patch(get(L(i),'XData'),get(L(i),'YData'),color(i),'FaceAlpha',.35);
  delete(L(i));
  uistack(P(i),'top');
end

set(gca,'linewidth', 2, 'fontsize', 16)
grid on;
ylabel('Carrier-Phase Residuals ( m. )')
xlabel('Pseudorange Residuals ( m. )')
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontWeight', 'bold')
set(findall(gcf,'-property','FontSize'),'FontSize',24)


hold off;
alpha(S,0.35)

% Remove whitespace around plot
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];


saveas(gcf, 'lq_residuals_iter_4','png')
close;


%Iteration 2
figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;

res = load('../lq/bce/01/iter_2.residuals');


S = scatter(res(1:2:end,1),res(1:2:end,2),30,'MarkerEdgeColor','k',...
              'MarkerFaceColor','k',...
              'LineWidth',1.5);

weights = [0.00310981   0.838988  0.0193018   0.043301  0.0921068 0.00314391];

mean_1 = [-0.930415  -1.25514];
mean_2 = [0.239016 -0.000201131];
mean_3 = [-16.069 0.00825808];
mean_4 = [38.3419 -0.0122074];
mean_5 = [-1.84349 0.0173844];
mean_6 = [2.78408 0.230594];

cov_1 = [5.16178 0.00496672;
0.00496672   0.032732];

cov_2 = [25.9068 -0.00994029;
-0.00994029 7.45283e-05];

cov_3 = [111.871  0.0112931;
0.0112931 0.00289449];

cov_4 = [243.39  -0.0470692;
-0.0470692 0.000939873];

cov_5 = [16.592  0.0127745;
0.0127745 0.00595875];

cov_6 = [11.5008 -0.737177;
-0.737177  0.160918];


min_x = -91;
min_y = -3;
max_x = 133;
max_y = 3;

xlim([min_x, max_x]);

color = ['y','m','c','r','g','b','w'];
for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));
  L(i) = error_ellipse( eval(c),eval(m),0.95 );
  P(i) = patch(get(L(i),'XData'),get(L(i),'YData'),color(i),'FaceAlpha',.35);
  delete(L(i));
  uistack(P(i),'top');
end

set(gca,'linewidth', 2, 'fontsize', 16)
grid on;
ylabel('Carrier-Phase Residuals ( m. )')
xlabel('Pseudorange Residuals ( m. )')
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontWeight', 'bold')
set(findall(gcf,'-property','FontSize'),'FontSize',24)


hold off;
alpha(S,0.35)

% Remove whitespace around plot
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];


saveas(gcf, 'lq_residuals_iter_2','png')
close;


%Iteration 0
figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;

res = load('../lq/bce/01/iter_0.residuals');


S = scatter(res(1:2:end,1),res(1:2:end,2),30,'MarkerEdgeColor','k',...
              'MarkerFaceColor','k',...
              'LineWidth',1.5);


weights = [0.719327  0.169959 0.0909872 0.0196921];

mean_1 = [-0.20258 -0.000683858];
mean_2 = [-1.25164 0.00534831];
mean_3 = [4.00192 -0.00116426];
mean_4 = [1.64532 -0.0167958];

cov_1 = [24.0954  -0.0119301;
-0.0119301 0.000283897];

cov_2 = [27.9842 0.0110428;
0.0110428 0.0056649];

cov_3 = [549.988  -0.212972;
-0.212972 0.00110898];

cov_4 = [5.41175 -0.00207212;
-0.00207212   0.0425468];



min_x = -91;
min_y = -3;
max_x = 133;
max_y = 3;

xlim([min_x, max_x]);

color = ['y','m','c','r','g','b','w'];
for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));
  L(i) = error_ellipse( eval(c),eval(m),0.95 );
  P(i) = patch(get(L(i),'XData'),get(L(i),'YData'),color(i),'FaceAlpha',.35);
  delete(L(i));
  uistack(P(i),'top');
end

set(gca,'linewidth', 2, 'fontsize', 16)
grid on;
ylabel('Carrier-Phase Residuals ( m. )')
xlabel('Pseudorange Residuals ( m. )')
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontWeight', 'bold')
set(findall(gcf,'-property','FontSize'),'FontSize',24)


hold off;
alpha(S,0.35)

% Remove whitespace around plot
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];


saveas(gcf, 'lq_residuals_iter_0','png')
close;
