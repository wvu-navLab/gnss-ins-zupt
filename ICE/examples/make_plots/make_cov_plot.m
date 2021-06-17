figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;



%% Iteration 4
res = load('../data/conf/all_1.residuals');
res_out = load('../data/conf/outliers_1.residuals');

S = scatter(res(1:2:end,1),res(1:2:end,2),30,'MarkerEdgeColor','k',...
              'MarkerFaceColor','k',...
              'LineWidth',1.5);


hold on;

S2 = scatter(res_out(1:end,1),res_out(1:end,2),30,'MarkerEdgeColor','r',...
              'MarkerFaceColor','r',...
              'LineWidth',3.5);

weights = [0.0295138888889   0.187934027778   0.782335069444];

mean_1 = [-10.1467818115 -0.0223681831437];
mean_2 = [12.125826295 -0.00603775824005];
mean_3 = [0 0 ];

cov_1 = [5.17772254419 -0.0121490364849;
    -0.0121490364849 0.0232115521311];

cov_2  = [7.16826290635 -0.0281522624125;
    -0.0281522624125 0.0053161847786];

cov_3 = [6.25 0 0.;
     0 0.0625];

min_x = -30;
min_y = -3;
max_x = 30;
max_y = 3;

xlim([min_x, max_x]);
ylim([-0.8,0.8])


color = ['y','m','c','r','g','b','w', 'y', 'm' 'c', 'r', 'g', 'b' ,'w' ,'y'];
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
alpha(S2,0.95)

% Remove whitespace around plot
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

%% DS 2
figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;



res = load('../data/conf/all_2.residuals');
res_out = load('../data/conf/outliers_2.residuals');

S = scatter(res(1:2:end,1),res(1:2:end,2),30,'MarkerEdgeColor','k',...
              'MarkerFaceColor','k',...
              'LineWidth',1.5);


hold on;

S2 = scatter(res_out(1:end,1),res_out(1:end,2),30,'MarkerEdgeColor','r',...
              'MarkerFaceColor','r',...
              'LineWidth',3.5);

weights = [0.0971986817326  0.00642655367232 0.00145951035782 0.012170433145 0.747245762712 0.0182674199623 0.0500235404896 0.0504237288136 0.0153013182674];

mean_1 = [-11.0592711213 0.162244206836];
mean_2 = [ 10.7784911998 -0.108276241479 ];
mean_3 = [5.88830096712 0.410734767492 ];
mean_4 = [ 1.69620307548 -0.838926121126 ];
mean_5 = [ -0.273599966769  0.043312089614 ];
mean_6 = [ -5.32481600595 -0.929505904439 ];
mean_7 = [ -7.4838996915 -0.456989441248 ];
mean_8 = [ 8.6719469218 -0.0384035844356 ];
mean_9 = [ 12.8936708506 -0.108807710174 ]
mean_10 = [ -0.273599966769  0.943312089614 ];



cov_1 = [26.2997679913 0.3548915333559;
    0.354891533355 0.0436042021801];

cov_2 = [1.0105855026 -0.00592721764661;
     -0.00592721764661 0.0116926460417];


cov_3 = [0.938815643788 0.0655477814637;
    0.0655477814637 0.0847533894361];

cov_4  = [2.25175127633 -0.089763746409;
    -0.089763746409 0.0395958680376];

cov_5 = [5.06619759316 -0.0151631014498;
     -0.0151631014498 0.0269556263362];

cov_6 = [4.32719597471 0.0408713482786;
    0.0408713482786 0.0180361473215];

cov_7  = [3.18653703684 0.137272503204;
    0.137272503204 0.221649561574];

cov_8 = [1.33553247476 -0.00852757365469;
     -0.00852757365469 0.00887462031513];

cov_9 = [10.9653295368 -0.117369799205;
          -0.117369799205 0.100578730254];


cov_10 = [5.73450924 -0.0242452498;
     -0.0242452498 0.0569556263362];



min_x = -91;
min_y = -3;
max_x = 133;
max_y = 3;

xlim([min_x, max_x]);

color = ['y','m','c','r','b','g','w', 'y', 'm' 'c', 'r', 'g', 'b' ,'w' ,'y'];
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







%% DS 3
figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;



res = load('../data/conf/all_3.residuals');
res_out = load('../data/conf/outliers_3.residuals');

S = scatter(res(1:2:end,1),res(1:2:end,2),30,'MarkerEdgeColor','k',...
              'MarkerFaceColor','k',...
              'LineWidth',1.5);


hold on;

S2 = scatter(res_out(1:end,1),res_out(1:end,2),30,'MarkerEdgeColor','r',...
              'MarkerFaceColor','r',...
              'LineWidth',3.5);

weights = [0.0953649947654  0.904635005235];

mean_1 = [-10.9590462219 0.0698842419532];
mean_2 = [  0 0  ];


cov_1 = [582.195208195 -3.00168184121;
    -3.00168184121 0.019905566708];

cov_2 = [6.25 0;
     0 0.0625];



min_x = -91;
min_y = -3;
max_x = 133;
max_y = 3;

xlim([min_x, max_x]);

color = ['y','m','c','r','b','g','w', 'y', 'm' 'c', 'r', 'g', 'b' ,'w' ,'y'];
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






%% High Quality DS 1
%% Incremental
figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;

res = load('../data/conf/all_1.residuals');
res_out = load('../data/conf/outliers_1.residuals');

S = scatter(res(1:2:end,1),res(1:2:end,2),30,'MarkerEdgeColor','k',...
              'MarkerFaceColor','k',...
              'LineWidth',1.5);

hold on;

S2 = scatter(res_out(1:end,1),res_out(1:end,2),30,'MarkerEdgeColor','r',...
              'MarkerFaceColor','r',...
              'LineWidth',3.5);

weights = [0.0637241628306  0.026626951891 0.909558624425];

mean_1 = [-12.098663401 0.0846820410545];
mean_2 = [11.3098181655 -0.0412620954993 ];
mean_3 = [0 0];


cov_1 = [7.46923381611 -0.0500295355646;
    -0.0500295355646 0.0035764340939];

cov_2 = [6.82045312263 -0.0193807166331;
        -0.0193807166331 0.00692748958703]

cov_3 = [6.25 0;
     0 0.0625];

color = ['r','b','g'];
for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));
  L(i) = error_ellipse( eval(c),eval(m),0.95 );
  P(i) = patch(get(L(i),'XData'),get(L(i),'YData'),color(i),'FaceAlpha',.35);
  delete(L(i));
  uistack(P(i),'bottom');
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



%% High Quality DS 3
%% Incremental
figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;

res = load('../data/conf/all_2.residuals');
res_out = load('../data/conf/outliers_2.residuals');

S = scatter(res(1:2:end,1),res(1:2:end,2),30,'MarkerEdgeColor','k',...
              'MarkerFaceColor','k',...
              'LineWidth',1.5);

hold on;

S2 = scatter(res_out(1:end,1),res_out(1:end,2),30,'MarkerEdgeColor','r',...
              'MarkerFaceColor','r',...
              'LineWidth',3.5);

weights = [1.0];

mean_1 = [0 0];

cov_1 = [6.25 0;
     0 0.0625];

color = ['g'];
for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));
  L(i) = error_ellipse( eval(c),eval(m),0.95 );
  P(i) = patch(get(L(i),'XData'),get(L(i),'YData'),color(i),'FaceAlpha',.35);
  delete(L(i));
  uistack(P(i),'bottom');
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




%% High Quality DS 3
%% Incremental
figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;

res = load('../data/conf/all_3.residuals');
res_out = load('../data/conf/outliers_3.residuals');

S = scatter(res(1:2:end,1),res(1:2:end,2),30,'MarkerEdgeColor','k',...
              'MarkerFaceColor','k',...
              'LineWidth',1.5);

hold on;

S2 = scatter(res_out(1:end,1),res_out(1:end,2),30,'MarkerEdgeColor','r',...
              'MarkerFaceColor','r',...
              'LineWidth',3.5);

weights = [0.00759585729485  0.0058854426203 0.986505232253];

mean_1 = [-11.0967048759 0.0833628658465];
mean_2 = [10.4737206533 -0.081360491149 ];
mean_3 = [0 0];


cov_1 = [12.0540034832 -0.128709427397;
    -0.128709427397 0.00655887925362];

cov_2 = [6.60333098664 -0.119627141694;
        -0.119627141694 0.00808057720917]

cov_3 = [6.25 0;
     0 0.0625];

color = ['r','b','g'];
for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));
  L(i) = error_ellipse( eval(c),eval(m),0.95 );
  P(i) = patch(get(L(i),'XData'),get(L(i),'YData'),color(i),'FaceAlpha',.35);
  delete(L(i));
  uistack(P(i),'bottom');
end

xlim([-30,30])

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
