
fid = fopen('out.gtsam','r');
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

multipath_noise = zeros(length(satX),10);
c = 299792458; 
L_chip = 1.023e-6*c;

for k = 1:10
    for j = 1:length(satX)
        s.N = 1;
        s.elDeg = elv(j);
        s.view = false;
        s.Tc = 1.023e-6;
        param = simulateMultipathParameters(s);
    %   [param.etadB, param.Delta_theta, param.Delta_tau];
        %gnss rums eq 25-28
        delta_tau = param.Delta_tau;
        delta_s = delta_tau*L_chip;
        beta = (-1)*2*pi*delta_s/0.1905;
        d = 0.6;
        multipath_noise(j,k) = 1.023e-6*c*errormodel(0.5,beta,delta_tau,d);
    end
end

for k = 1:10
 
    ECEFcopy = ECEF;
    
    for j = 1:length(satX)
        ECEFcopy{6}(j) = ECEFcopy{6}(j) + multipath_noise(j,k);
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


    fileID = fopen(strcat('noisydata', int2str(k),'.gtsam'),'w');
    formatSpec = '%d %.12f %d %s %d %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n';
    [nrows,ncols] = size(data);
    for row = 1:nrows
        fprintf(fileID,formatSpec,data{row,:});
    end
    fclose(fileID);
  
    
end

% figure();
% subplot(2,2,1)
% histogram(multipath_noise(:,1))
% subplot(2,2,2)
% histogram(multipath_noise(:,2))
% subplot(2,2,3)
% histogram(multipath_noise(:,3))
% subplot(2,2,4)
% histogram(multipath_noise(:,4))
% 
% diference = multipath_noise(:,1) - multipath_noise(:,8);







