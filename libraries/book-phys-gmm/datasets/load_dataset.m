function [Data] = load_dataset(pkg_dir, dataset, sub_sample, nb_trajectories)

dataset_name = [];
switch dataset
    case 1
        dataset_name = '2D_concentric.mat';   
    case 2
        dataset_name = '2D_opposing.mat';           
    case 3
        dataset_name = '2D_multiple.mat';           
    case 4
        dataset_name = '2D_snake.mat';
    case 5
        dataset_name = '2D_messy-snake.mat';
    case 6 
        dataset_name = '2D_viapoint.mat';
    case 7
        dataset_name = '2D_Lshape.mat';
    case 8
        dataset_name = '2D_Ashape.mat';
    case 9
        dataset_name = '2D_Sshape.mat';
    case 10
        dataset_name = '2D_multi-behavior.mat';        
    case 11
        dataset_name = '3D_viapoint_2.mat';
        traj_ids = [1 2];
    case 12
        dataset_name = '3D_sink.mat';
    case 13
        dataset_name = '3D_bumpy-snake.mat';
end

if isempty(sub_sample)
   sub_sample = 2; 
end

if dataset <= 6
    Data_ = load(strcat(pkg_dir,'/datasets/',dataset_name));
    Data = Data_.Data(:,1:sub_sample:end);
elseif dataset <= 10
    data_ = load(strcat(pkg_dir,'/datasets/',dataset_name));
    data = data_.data;
    N = length(data);    
    Data = [];
    for n=1:N
        data_ = data{n};
        Data = [Data data_(:,1:sub_sample:end)];
    end
else
    data_ = load(strcat(pkg_dir,'/datasets/',dataset_name));
    data = data_.data;
    N = length(data);    
    Data = [];
    traj_ids = randsample(N,nb_trajectories);
    for l=1:length(traj_ids)
        % Gather Data
        data_ = data{traj_ids(l)};
        Data = [Data data_(:,1:sub_sample:end)];
    end
end