truth_d1 = load('/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/data/truth/drive_1.xyz');

truth_d2 = load('/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/data/truth/drive_2.xyz');

truth_d3 = load('/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/data/truth/drive_3.xyz');

nom = [856514.1467,-4843013.0689, 4047939.8237];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% GET L2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

txt = "/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/test/sens/l2_t2/d3";

cd('/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/test/sens/l2_t2/d3');
d = dir();

clear x;
clear l2_mean;
clear l2_med;
clear l2_max;
clear l2_var;

for i = 3:length(d)
  file = strcat(txt,'/',d(i).name,'/ecef.sol');
  x(i-2) = str2double(d(i).name);
  data = load(file);
  [~, tmp,~] = getError(truth_d3, data, truth_d3(1,2:end));
  l2_mean(i-2) = mean(tmp);
  l2_med(i-2) = median(tmp);
  l2_max(i-2) = max(tmp);
  l2_var(i-2) = var(tmp);
end


figure('Renderer', 'painters', 'Position', [10 10 900 600]);
hold on;

[a,b] = sort(x);
a(a>1) = a(a>1)/2.5;
a(a<1) = -1 * (1./(a(a<1))) * 2.5;
ind = find(x==2.5);
plot(a,l2_med(b) - l2_med(ind), 'o',...
    'LineWidth',6,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor','g',...
    'MarkerSize',10)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% GET DCS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
txt = "/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/test/sens/dcs_t2/d3";

cd('/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/test/sens/dcs_t2/d3');
d = dir();

clear x;
clear dcs_mean;
clear dcs_med;
clear dcs_max;
clear dcs_var;

for i = 3:length(d)
  file = strcat(txt,'/',d(i).name,'/ecef.sol');
  x(i-2) = str2double(d(i).name);
  data = load(file);
  [~, tmp,~] = getError(truth_d3, data, truth_d3(1,2:end));
  dcs_mean(i-2) = mean(tmp);
  dcs_med(i-2) = median(tmp);
  dcs_max(i-2) = max(tmp);
  dcs_var(i-2) = var(tmp);
end

[a,b] = sort(x);
a(a>1) = a(a>1)/2.5;
a(a<1) = -1 * (1./(a(a<1))) * 2.5;
ind = find(x==2.5);
plot(a, dcs_med(b) - dcs_med(ind), 'o',...
    'LineWidth',6,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor','b',...
    'MarkerSize',10)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% GET MM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
txt = "/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/test/sens/mm_t2/d3";

cd('/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/test/sens/mm_t2/d3');

d = dir();

clear x;
clear mm_mean;
clear mm_med;
clear mm_max;
clear mm_var;

for i = 3:length(d)
  file = strcat(txt,'/',d(i).name,'/ecef.sol');
  x(i-2) = str2double(d(i).name);
  data = load(file);
  [~, tmp,~] = getError(truth_d3, data, truth_d3(1,2:end));
  mm_mean(i-2) = mean(tmp);
  mm_med(i-2) = median(tmp);
  mm_max(i-2) = max(tmp);
  mm_var(i-2) = var(tmp);
end

[a,b] = sort(x);
a(a>1) = a(a>1)/2.5;
a(a<1) = -1 * (1./(a(a<1))) * 2.5;
ind = find(x==2.5);
plot(a, mm_med(b) - mm_med(ind), 'o',...
    'LineWidth',6,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor','r',...
    'MarkerSize',10)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% GET BCE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
txt = "/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/test/sens/bce_t2/d3";

cd('/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/test/sens/bce_t2/d3');

d = dir();

clear x;
clear bce_mean;
clear bce_med;
clear bce_max;
clear bce_var;

for i = 4:length(d)
  file = strcat(txt,'/',d(i).name,'/ecef.sol');
  x(i-2) = str2double(d(i).name);
  data = load(file);
  [~, tmp,~] = getError(truth_d3, data, truth_d3(1,2:end));
  bce_mean(i-2) = mean(tmp);
  bce_med(i-2) = median(tmp);
  bce_max(i-2) = max(tmp);
  bce_var(i-2) = var(tmp);
end

[a,b] = sort(x);
a(a>1) = a(a>1)/2.5;
a(a<1) = -1 * (1./(a(a<1))) * 2.5;
ind = find(x==2.5);
plot(a, bce_med(b) - bce_med(ind), 'o',...
    'LineWidth',6,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','k',...
    'MarkerSize',10)


% Remove whitespace around plot
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
