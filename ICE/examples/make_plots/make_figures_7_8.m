f1 = figure('Renderer', 'painters', 'Position', [10 10 900 600]);
f2 = figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;

off_set = linspace(0,0.35, 5);
off_set_phase = linspace(0,2,5);
min_x = -1000;
min_y = -3;
max_x = 1000;
max_y = 3;

x = linspace(min_x,max_x,5000);
y = linspace(min_y,max_y,5000);

color = ['y','m','c','r','g','b','w'];

%% Iteration 4
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

hold on;


for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));

  m = eval(m);
  c = eval(c);
  p(i,:) = weights(i) * normpdf(x,m(1) , c(1,1));
  q(i,:) = weights(i) * normpdf(y,m(2), c(2,2)) ;
end

figure(f1);
hold on;
pdf = sum(p,1)./norm(sum(p,1));
fill(x,off_set(5)+pdf,color(5))

max_range = max(off_set(5)+pdf);

figure(f2);
hold on;
pdf = sum(q,1)./norm( sum(q,1) );
fill(y,off_set_phase(5)+pdf,color(5))
max_phase = max(off_set_phase(5)+pdf);

clearvars -except f1 f2 off_set off_set_phase min_x min_y max_x max_y x y color max_range max_phase;

%Iteration 3
weights = [0.00312376   0.843399  0.0227363  0.0413071  0.0872587 0.00212723];

mean_1 = [-2.31998 -1.79582];
mean_2 = [0.3703 -0.000220234];
mean_3 = [-32.1622 0.0179009];
mean_4 = [66.4431 -0.0150286];
mean_5 = [-1.94493 0.0246008];
mean_6 = [2.99175 0.53048];

cov_1 = [5.19326 0.0298447;
0.0298447 0.0513659];

cov_2 = [26.2008 -0.00896162;
-0.00896162 6.86976e-05];

cov_3 = [316.603   0.167115;
0.167115 0.00309956];

cov_4 = [198.457  -0.0155256;
-0.0155256 0.000999533];

cov_5 = [15.8555 0.00710944;
0.00710944 0.00683197];

cov_6 = [12.9893 -0.95102;
-0.95102 0.231845];

for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));

  m = eval(m);
  c = eval(c);
  p(i,:) = weights(i) * normpdf(x,m(1) , c(1,1));
  q(i,:) = weights(i) * normpdf(y,m(2), c(2,2)) ;
end

figure(f1);
hold on;
pdf = sum(p,1)./norm(sum(p,1));
fill(x,off_set(4)+pdf,color(4))

figure(f2);
hold on;
pdf = sum(q,1)./norm( sum(q,1) );
fill(y,off_set_phase(4)+pdf,color(4))

clearvars -except f1 f2 off_set off_set_phase min_x min_y max_x max_y x y color max_range max_phase;

%Iteration 2
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

for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));

  m = eval(m);
  c = eval(c);
  p(i,:) = weights(i) * normpdf(x,m(1) , c(1,1));
  q(i,:) = weights(i) * normpdf(y,m(2), c(2,2)) ;
end
figure(f1);
hold on;
pdf = sum(p,1)/norm(sum(p,1));
fill(x,off_set(3)+pdf,color(3))

figure(f2);
hold on;
pdf = sum(q,1)/norm( sum(q,1) );
fill(y,off_set_phase(3)+pdf,color(3))

clearvars -except f1 f2 off_set off_set_phase min_x min_y max_x max_y x y color max_range max_phase;

%Iteration 1
weights = [0.804251   0.108833  0.0798134 0.00311941 0.00394116];

mean_1 = [-0.0801387 5.27653e-05];
mean_2 = [-1.69867 0.0109677];
mean_3 = [10.8531 -0.00344178];
mean_4 = [0.74278 -0.703496];
mean_5 = [2.79127 0.0657553];

cov_1 = [23.6621  -0.0103445;
-0.0103445 8.99869e-05];

cov_2 = [19.115  0.0291247;
0.0291247 0.00426041];

cov_3 = [673.475     -0.2369;
-0.2369 0.000884116];

cov_4 = [5.42928 -0.0157724;
-0.0157724  0.0164323];

cov_5 = [10.9898 -0.331296;
-0.331296  0.064349];

for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));

  m = eval(m);
  c = eval(c);
  p(i,:) = weights(i) * normpdf(x,m(1) , c(1,1));
  q(i,:) = weights(i) * normpdf(y,m(2), c(2,2)) ;
end
figure(f1);
hold on;
pdf = sum(p,1)/norm(sum(p,1));
fill(x,off_set(2)+pdf,color(2))

figure(f2);
hold on;
pdf = sum(q,1)/norm( sum(q,1) );
fill(y,off_set_phase(2)+pdf,color(2))


clearvars -except f1 f2 off_set off_set_phase min_x min_y max_x max_y x y color max_range max_phase;


%Iteration 0
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

for i = 1:length(weights)
  c = strcat('cov_', string(i));
  m = strcat('mean_', string(i));

  m = eval(m);
  c = eval(c);
  p(i,:) = weights(i) * normpdf(x,m(1) , c(1,1));
  q(i,:) = weights(i) * normpdf(y,m(2), c(2,2)) ;
end
figure(f1);
hold on;
pdf = sum(p,1)/norm(sum(p,1));
fill(x,off_set(1)+pdf,color(1))

figure(f2);
hold on;
pdf = sum(q,1)/norm( sum(q,1) );
fill(y,off_set_phase(1)+pdf,color(1))



figure(f1);
hold on;
yticks(off_set);
tk = ["Iteration 1", "Iteration 2", "Iteration 3", "Iteration 4", "Iteration 5"];
yticklabels(tk);
l(1) = line([0 0],[0 10],'Color','Black','LineWidth',2,'LineStyle','--');
uistack(l(1),'bottom');


figure(f2);
hold on;
yticks(off_set + (off_set(2) - off_set(1))/2);
yticklabels(tk);
l(2) = line([0 0],[0 10],'Color','Black','LineWidth',2,'LineStyle','--');
uistack(l(2),'bottom');

figure(f2);
hold on;
yticks(off_set)

figure(f1);
hold off;
alpha(0.75);
hold on;
set(gca,'linewidth', 2, 'fontsize', 16)
% ylabel('Carrier-Phase Residuals ( m. )')
xlabel('Pseudorange Uncertianty Magnitude ( m. )')
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontWeight', 'bold')
set(findall(gcf,'-property','FontSize'),'FontSize',24)
ylim([0,max_range])
xlim([-200,200])

figure(f2);
hold off;
alpha(0.75);
hold on;
set(gca,'linewidth', 2, 'fontsize', 16)
% ylabel('Carrier-Phase Residuals ( m. )')
xlabel('Carrier-Phase Uncertianty Magnitude ( m. )')
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontWeight', 'bold')
set(findall(gcf,'-property','FontSize'),'FontSize',24)
ylim([0,max_phase])
xlim([-0.04, 0.07]);

yticks(off_set_phase);


% Remove whitespace around plot
figure(f1);
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
saveas(gcf, 'range_cov_evolution','png')
close;

figure(f2);
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
saveas(gcf, 'phase_cov_evolution','png')
close;
