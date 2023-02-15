function [Data, Data_sh, att, x0_all, dt, data] = processDataStructure3(data, sub_sample)

% Computing the attractor and shifting all the trajectories
N = length(data);att_ = [];
M = size(data{1},1)/2;
for n=1:N
    att_ = [att_ data{n}(1:M,end)];
end
att = mean(att_,2);
shifts = att_ - repmat(att,[1 size(att_,2)]);
Data = []; Data_sh = []; x0_all = [];
for l=1:N
    % Gather Data
    data_ = data{l};
    shifts_ = repmat(shifts(:,l),[1 length(data_)]);
    data_(1:M,:)       = data_(1:M,:) - shifts_;
    data_(M+1:end,end) = zeros(M,1);
    data_(M+1:end,end-1) = (data_(M+1:end,end) + zeros(M,1))/2;
    data_(M+1:end,end-2) = (data_(M+1:end,end-2) + data_(M+1:end,end-1))/2;
    Data = [Data data_];
    
    % All starting position for reproduction accuracy comparison
    x0_all = [x0_all data_(1:M,1)];
    
    % Shift data to origin for Sina's approach + SEDS
    data_(1:M,:) = data_(1:M,:) - repmat(att, [1 length(data_)]);
    data_(M+1:end,end) = zeros(M,1);
    
    Data_sh = [Data_sh data_];
    
    % Generate new data structure for SEDS + Diff-DS
    data{l} = data_;
end
data_12 = data{1}(:,1:M);
dt = abs((data_12(1,1) - data_12(1,2))/data_12(M+1,1));

Data    = Data(:,1:sub_sample:end-1);
Data_sh = Data_sh(:,1:sub_sample:end-1);

end