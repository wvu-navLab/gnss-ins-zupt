function get_stats(est_data, truth_data)

    truth_ = load(truth_data);
    nom = [856514.1467,-4843013.0689, 4047939.8237];

    // l2_ = load(strcat(est_data,'/l2.xyz'));
    // [~, l2_rh, l2_rt] = getError(truth_, l2_, truth_(1,2:end));
    //
    // dcs_ = load(strcat(est_data, '/dcs.xyz'));
    // [~, dcs_rh, dcs_rt] = getError(truth_, dcs_, truth_(1,2:end));
    //
    // mm_ = load(strcat(est_data, '/mm.xyz'));
    // [~, mm_rh, mm_rt] = getError(truth_, mm_, truth_(1,2:end));

    ice_ = load(strcat(est_data, '/ice.xyz'));
    [~, ice_rh, ice_rt] = getError(truth_, ice_, truth_(1,2:end));

    type = {'L2';'DCS';'MM'; 'ICE'};
    med_ = [median(l2_rh);median(dcs_rh); median(mm_rh); median(ice_rh)];
    mean_ = [mean(l2_rh);mean(dcs_rh); mean(mm_rh); mean(ice_rh)];
    var_ = [var(l2_rh);var(dcs_rh); var(mm_rh); var(ice_rh)];
    max_ = [max(l2_rh);max(dcs_rh); max(mm_rh); max(ice_rh)];
    Stats = table(type, med_, mean_, var_, max_);

    StatsTrim = strtrim(cellstr(num2str(Stats.med_,'%.2f')));
    Stats.med_ = StatsTrim;

    StatsTrim = strtrim(cellstr(num2str(Stats.mean_,'%.2f')));
    Stats.mean_ = StatsTrim;

    StatsTrim = strtrim(cellstr(num2str(Stats.var_,'%.2f')));
    Stats.var_ = StatsTrim;

    StatsTrim = strtrim(cellstr(num2str(Stats.max_,'%.2f')));
    Stats.max_ = StatsTrim;

    writetable(Stats, strcat(est_data, '/stats.txt'),'Delimiter','\t','WriteRowNames',true);
