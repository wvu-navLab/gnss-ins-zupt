for i=1:length(slipBL)
if slipBL(i) == 0
    slipname{:,i}='noSlip';
elseif abs(slipBL(i)) <= 0.25
    slipname{:,i}='lowSlip';
elseif abs(slipBL(i)) <= 0.50
    slipname{:,i}='medSlip';
else
    slipname{:,i}='highSlip';
end
end

slip=table(rollIMU'*180/pi, pitchIMU'*180/pi, yawIMU'*180/pi,slipBL',sideslip',slipname');
slip.Properties.VariableNames{1} = 'roll';
slip.Properties.VariableNames{2} = 'pitch';
slip.Properties.VariableNames{3} = 'yaw';
slip.Properties.VariableNames{4} = 'LongSlip';
slip.Properties.VariableNames{5} = 'LatSlip';
slip.Properties.VariableNames{6} = 'slipname';
% Longitudinal ~ pitch
% Lateral ~ roll
