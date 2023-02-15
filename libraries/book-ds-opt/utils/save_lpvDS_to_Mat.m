function save_lpvDS_to_Mat(name, pkg_dir, ds_gmm, A_k, b_k, att, x0_all, dt, P_est, constr_type, est_options)
lpvDS_model =[];
lpvDS_model.name         = name;
lpvDS_model.ds_gmm       = ds_gmm;
lpvDS_model.A_k          = A_k;
lpvDS_model.b_k          = b_k;
lpvDS_model.att          = att;
lpvDS_model.x0_all       = x0_all;
lpvDS_model.dt           = dt;
lpvDS_model.P_est        = P_est;
lpvDS_model.constr_type  = constr_type;
lpvDS_model.est_options  = est_options;
matfile = strcat(pkg_dir,'/models/', lpvDS_model.name,'.mat');
save(matfile,'-struct', 'lpvDS_model')
end