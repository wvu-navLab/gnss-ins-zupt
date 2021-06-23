
fid = fopen('out10.gtsam','r');
ECEF = textscan(fid, '%d %.12f %d %s %d %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f','delimiter',' ');
fclose(fid);

origin = [859153.0167; -4836303.7245; 4055378.4991];

satindex = ECEF{5};
timeindex = ECEF{3};
satX = ECEF{16};
satY = ECEF{17};
satZ = ECEF{18};

enuSat = zeros(length(satX),3);

for i = 1:length(satX)
    enuSat(i, :) = xyz2enu([satX(i), satY(i), satZ(i)], origin);
end

elv = zeros(length(satX),1);

for i = 1: length(elv)
    elv(i) = asind(enuSat(i,3)/norm(enuSat(i,:)));
end

range_noise = zeros(length(satX),1);
phase_noise = zeros(length(satX),1);

c = 299792458; 
L_chip = 1.023e-6*c;

% for k = 1:10
for j = 1:length(satX)
    if rand >= 0.9
        s.N = 1;
        s.elDeg = elv(j);
        s.view = false;
        s.Tc = 1.023e-6;
        param = simulateMultipathParameters(s);
    %   [param.etadB, param.Delta_theta, param.Delta_tau];
        %gnss rums eq 25-28
        delta_tau = param.Delta_tau;
        delta_theta = param.Delta_theta;
        delta_s = delta_tau*L_chip;
        beta = (-1)*2*pi*delta_s/0.1905;
        d = 0.6;
        range_noise(j) = 1.023e-6*c*errormodel(0.5,beta,delta_tau,d);
        phase_noise(j) = delta_theta*0.1905/(2*pi);
    end
end
% end

% for k = 1:10
 
ECEFcopy = ECEF;

for j = 1:length(satX)
    ECEFcopy{6}(j) = ECEFcopy{6}(j) + range_noise(j);
    ECEFcopy{7}(j) = ECEFcopy{7}(j) + phase_noise(j);
end

data = {};
for i = 1:length(satX)
    for j  = 1:20
        if j == 4
            data{i,j} = ECEFcopy{j}{i};
        else
            data{i,j} = ECEFcopy{j}(i);
        end
    end
end

fileID = fopen('noisydata_sparse.gtsam','w');
formatSpec = '%d %.12f %d %s %d %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n';
[nrows,ncols] = size(data);
for row = 1:nrows
    fprintf(fileID,formatSpec,data{row,:});
end
fclose(fileID);
  
    
% end








