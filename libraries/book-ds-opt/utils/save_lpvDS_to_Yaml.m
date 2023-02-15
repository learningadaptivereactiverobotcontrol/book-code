function save_lpvDS_to_Yaml(DS_name, pkg_dir,  ds_gmm, A_k, att, x0_all, dt)

% GMM parameters
K          = length(ds_gmm.Priors);
dim        = size(ds_gmm.Mu,1);
Priors_vec = ds_gmm.Priors;
Mu_vec     = ds_gmm.Mu(1:end);
Sigma_vec  = ds_gmm.Sigma(1:end);

% DS parameters
A_vec      = A_k(1:end);

% Initial points (to simulate)
x0_all_vec = x0_all(1:end);

% Create structure to dump in yaml file
lpvDS_model =[];
lpvDS_model.name         = DS_name;
lpvDS_model.K            = K;
lpvDS_model.M            = dim;
lpvDS_model.Priors       = Priors_vec;
lpvDS_model.Mu           = Mu_vec;
lpvDS_model.Sigma        = Sigma_vec;
lpvDS_model.A            = A_vec;
lpvDS_model.attractor    = att(1:end);
lpvDS_model.x0_all       = x0_all_vec;
lpvDS_model.dt           = dt;

% Visualize what will be dumped on yaml file
lpvDS_dump = YAML.dump(lpvDS_model);
yamlfile = strcat(pkg_dir,'/models/', lpvDS_model.name,'.yml');

% Save yaml file
fprintf('The following parameters were saved in: %s \n', yamlfile);
disp(lpvDS_dump);
YAML.write(yamlfile,lpvDS_model)

end