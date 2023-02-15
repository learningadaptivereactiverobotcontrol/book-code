function save_seDS_to_Yaml(DS_name, pkg_dir, Priors, Mu, Sigma, att, x0_all, dt)

% Write Text file with desired precision
txtlfile = strcat(pkg_dir,'/models/', DS_name,'.txt');
export2SEDS_Cpp_lib(txtlfile,Priors,Mu,Sigma);
% Reconstruct
data = dlmread(txtlfile);
dim_rec = data(1,1);
k_rec   = data(1,2);
data(1,:) = [];
% reconstructing the priors
Priors_rec = data(1:k_rec,1);
data(1:k_rec,:) = [];
% reconstructing the Mu
Mu_rec = data(1,1:dim_rec*k_rec);
Mu_rec = reshape(Mu_rec,dim_rec,k_rec);
data(1,:) = [];
% reconstructing the Sigma
Sigma_rec = zeros(dim_rec,dim_rec,k_rec);
for n = 1: k_rec   
    Sigma_rec(:,:,n) = transpose(reshape(data(n,:),dim_rec,dim_rec));        
end
Priors_vec = Priors_rec(1:end);
Mu_vec = Mu_rec(1:end);
Sigma_vec = Sigma_rec(1:end);
attractor = zeros(1,6);
attractor(1:3) = att(1:end);
x0_all_vec = x0_all(1:end);

if max(Sigma_vec)>1e6
    Sigma_scale = round(max(Sigma_vec)/1e6);
else
    Sigma_scale = 1;
end

if max(Mu_vec)>1e6
    Mu_scale = round(max(Mu_vec)/1e6);
else
    Mu_scale = 1;
end

Mu_vec = Mu_vec./Mu_scale;
Sigma_vec = Sigma_vec./Sigma_scale;

% Create structure to dump in yaml file
seDS_model = struct('name',         DS_name,...
                    'K',            k_rec,...
                    'dim',          dim_rec,...
                    'Priors',       Priors_vec,...
                    'Mu',           Mu_vec,...
                    'Sigma',        Sigma_vec,...
                    'attractor',    attractor,...
                    'xO_all',       x0_all_vec,...
                    'dt',           dt,...
                    'Mu_scale',     Mu_scale,...
                    'Sigma_scale',  Sigma_scale);

% Visualize what will be dumped on yaml file
seDS_dump = YAML.dump(seDS_model);
yamlfile = strcat(pkg_dir,'/models/', seDS_model.name,'.yml');

% Save yaml file
fprintf('The following parameters were saved in: %s \n', yamlfile);
disp(seDS_dump);
YAML.write(yamlfile,seDS_model)

end