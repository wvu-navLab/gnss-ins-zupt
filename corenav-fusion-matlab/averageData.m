function [avgData]=averageData(rawData)
% data = [(1:length(rawData))', rawData]; % Create sample data
% column2 = data(:, 2);
% out = reshape(column2(1:length(rawData)), [], 5);
% avgData = mean(out, 2)';
count =0;
k=1;
for i=1:length(rawData)
count = count +1;
if (mod(count,5)==0)
avgData(k)=mean(rawData(i-4:i));
k=k+1;
count=0;
end
end