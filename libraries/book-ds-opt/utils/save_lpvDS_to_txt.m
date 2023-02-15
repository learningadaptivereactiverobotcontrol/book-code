function save_lpvDS_to_txt(DS_name, pkg_dir,  ds_gmm, A_k,  att)

model_dir = strcat(pkg_dir,'/models/',DS_name, '/');
mkdir(model_dir)

% GMM parameters
Priors = ds_gmm.Priors;
Mu     = ds_gmm.Mu;
Sigma  = ds_gmm.Sigma;

% Writing Dimensions
Dimensions = [length(Priors); size(Mu,1)];
dlmwrite(strcat(model_dir,'dimensions'), Dimensions,'Delimiter',' ','precision','%.6f');

% Writing attractor
dlmwrite(strcat(model_dir,'attractor'), att,'Delimiter',' ','precision','%.6f');

% Writing Priors
dlmwrite(strcat(model_dir,'Priors'), Priors,'Delimiter',' ','precision','%.6f');

% Writing Mu
dlmwrite(strcat(model_dir,'Mu'), Mu, 'newline','unix','Delimiter',' ','precision','%.6f');

% Writing Sigma
for i=1:length(Priors)
    dlmwrite(strcat(model_dir,'Sigma'), Sigma(:,:,i),'newline','unix','-append','Delimiter',' ','precision','%.6f');    
end

% Writing A's
for i=1:length(Priors)   
    dlmwrite(strcat(model_dir,'A_k'), A_k(:,:,i),'newline','unix','-append','Delimiter',' ','precision','%.6f');
end

