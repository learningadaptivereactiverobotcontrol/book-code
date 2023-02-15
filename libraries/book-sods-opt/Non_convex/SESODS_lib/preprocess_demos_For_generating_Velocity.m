function [demos_P,demos_V,demos_O] = preprocess_demos_For_generating_Velocity(demos,demos_P,demos_V,time,tol_cutting,options)

%%
%checking if a fixed time step is provided or not.
if length(time)==1
    dt = time;
end

 %dimensionality of demosntrations
Data=[];
index = 1;
norm=0;
if (strcmp(options.Velocity,'True')~=1)
for i=1:length(demos)
    d = size(demos{1},1);
    clear tmp tmp_d
    % de-noising data (not necessary)
    for j=1:d
        tmp(j,:)=demos{i}(j,:);
%         tmp(j,:) = smooth(demos{i}(j,:),25); 
    end
%     for ii=1:size(tmp,2)
%         if rem(ii,2)==0
%             tmp(:,j)=[];
%         end
%     end
        % saving the initial point of each demo
    x0(:,i) = tmp(:,1);
    
    %saving the final point (target) of each demo
    xT(:,i) = demos{i}(:,end); 
    
    % shifting demos to the origin
    tmp = tmp - repmat(xT(:,i),1,size(tmp,2));
    % computing the first time derivative
    if length(time)==1
        tmp_d = diff(tmp,1,2)/dt;
    else
        tmp_d = diff(tmp,1,2)./repmat(diff(time{i}),d,1);
    end
    
    % trimming demonstrations
    ind = find(sqrt(sum(tmp_d.*tmp_d,1))>tol_cutting);
    tmp = tmp(:,min(ind):max(ind)+1);
    tmp_d = tmp_d(:,min(ind):max(ind));
%     tmp_d(:,1)=tmp_d(:,2);
%     A=-1/(size(tmp_d,2)-);
%     for ii=1:size(tmp_d,2)
%      tmp_d(:,ii)=tmp_d(:,ii).*(A*(ii-1)+1);
%     end
%     tmp_d(:,end)=zeros(d,1);
%     tmp_d(:,end-10:end)=zeros(d,10);
    % saving demos next to each other
    Data = [Data [tmp;tmp_d zeros(d,1)]];
    index = [index size(Data,2)+1];
    if norm==0
        norm=max(max(abs((tmp_d))));
%                  norm=1;
    end
    demos_V{i}=[tmp_d zeros(d,1) ]/norm;
    demos_P{i}=tmp;
    demos_O{i}=[tmp;[tmp_d/norm zeros(d,1) ]];
end

xT = mean(xT,2); %we consider the mean value of all demonstraions' final point as the target
x0 = mean(x0,2); %the mean value of all demonstrations' initial point
else
    
    d = size(demos_P{1},1);
    for i=1:length(demos_P)
        
    clear tmp tmp_d
    % de-noising data (not necessary)
    for j=1:d
        tmp(j,:) = smooth(demos_P{i}(j,:),25);
        tmp_d(j,:) = smooth(demos_V{i}(j,:),25);
    end
    
    
    %saving the final point (target) of each demo
    xT(:,i) = demos_P{i}(1:d,end); 
    
    % shifting demos to the origin
    tmp = tmp - repmat(xT(:,i),1,size(tmp,2));
    tmp_d(:,end)=zeros(d,1);
    tmp_d(:,1)=zeros(d,1);
    % saving demos next to each other
    Data = [Data [tmp;tmp_d]];
    index = [index size(Data,2)+1];
    if norm==0
        norm=max(max(abs(tmp_d)));
%         norm=1;
    end
    demos_V{i}=[tmp_d]/norm;
    demos_P{i}=tmp;
    demos_O{i}=[tmp;[tmp_d/norm]];
end
end