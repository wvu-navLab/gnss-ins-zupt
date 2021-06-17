x = linspace(-3,3,5000);

% get m-estimator cost function Values
k = 1; %% set kernel width

l2 = (x.^2)./2;

for i = 1:length(x)
  if (abs( x(i) ) > k )
    huber(i) = k*abs(x(i)) - (k/2);
  else
    huber(i) = 1/2 * x(i)^2;
  end
end

cauchy = ( (k^2)/2 ) * log(1 + (x.^2));



figure('Position', [10,10,900,600])
hold on


plot(x,l2,'k','LineWidth', 5)
plot(x,huber,'k','LineWidth', 5, 'LineStyle', '--')
plot(x,cauchy,'k','LineWidth', 5, 'LineStyle', ':')


legend('l^2 Cost Function', 'Huber Cost Function', 'Cauchy Cost Function')

set(gca,'linewidth', 2, 'fontsize', 16)
grid on;
ylabel('\rho(x)')
xlabel('x')
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontWeight', 'bold')
set(findall(gcf,'-property','FontSize'),'FontSize',24)
axis tight
grid on;

% Remove whitespace around plot
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

saveas(gcf, 'm_estimators','png');
close;
