function save_seDS_to_Mat(name, pkg_dir, Priors0, Mu0, Sigma0, Priors, Mu, Sigma, att, x0_all, dt, options, est_options)
seDS_model =[];
seDS_model.name         = name;


% Initial GMM parameters
seDS_model.Priors0      = Priors0;
seDS_model.Mu0          = Mu0;
seDS_model.Sigma0       = Sigma0;

% GMM parameters after SEDS optimization
seDS_model.Priors       = Priors;
seDS_model.Mu           = Mu;
seDS_model.Sigma        = Sigma;

seDS_model.att          = att;
seDS_model.x0_all       = x0_all;
seDS_model.dt           = dt;
seDS_model.options      = options;
seDS_model.est_options  = est_options;
matfile = strcat(pkg_dir,'/models/', seDS_model.name,'.mat');
save(matfile,'-struct', 'seDS_model')
end