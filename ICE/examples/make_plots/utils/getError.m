function [dff, rsos_h, rsos_t] = getError(truth,ref, nom)

for i = 1:length(truth)
    truth_enu(i,:) = xyz2enu(truth(i,2:end),nom);
end


for i = 1:length(ref)
    ref_enu(i,:) = xyz2enu(ref(i,2:end),nom);
end


ref_enu_t = [ref(:,1),ref_enu];

count = 1;
for i = 1:length(truth)
    for j = 1:length(ref_enu_t)
        if ( abs( truth(i,1) - ref_enu_t(j,1)) < 0.01 )
        dff(count, :) = truth_enu(i,:) - ref_enu(j,:);
        count = count+1;
    end
end
end

for i = 1:length(dff)
    rsos_t(i) = sqrt( dff(i,:) * dff(i,:)' );
    rsos_h(i) = sqrt( dff(i,1:2) * dff(i,1:2)' );
end
