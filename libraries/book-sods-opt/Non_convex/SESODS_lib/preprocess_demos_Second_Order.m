function [X0 , XT,V0 , VT, Data_P, Data_V,Data_A, index,demos_P,demos_V,demos_O] = preprocess_demos_Second_Order(demos_P,demos_V,options)
time=options.dt;
tol_cutting=options.tol_cutting;
%checking if a fixed time step is provided or not.
if length(time)==1
    dt = time;
end
if (strcmp(options.Velocity,'True')~=1)
d = size(demos_P{1},1); %dimensionality of demosntrations
else
    d = size(demos_P{1},1)/2;
end
Data_P=[];
Data_V=[];
for i=1:length(demos_V)
    i
    clear tmp tmp_d ind tmpv tmp_V_Out_put index
Data_V_A_handle=[];
index = 1;    
    % de-noising data (not necessary)
    demos_V{i}(1:d,end)=demos_V{i}(1:d,end-1)/2;
    demos_V{i}(1:d,end+1)=zeros(d,1);
    for j=1:d
        tmp(j,:) = demos_V{i}(j,:); 
    end
%     for i_handle=ceil(15*size(tmp,2)/20):size(tmp,2)
%         if norm(tmp(:,i_handle))>norm(tmp(:,i_handle-1))
%             tmp(:,i_handle)=(norm(tmp(:,i_handle-1))/norm(tmp(:,i_handle))).*tmp(:,i_handle);
%         end
%     end
    % computing the first time derivative
    if (strcmp(options.Velocity,'True')~=1)
        if length(time)==1
            tmp_d = diff(tmp,1,2)/dt;
        else
            tmp_d = diff(tmp,1,2)./repmat(diff(time{i}),d,1);
        end
%         for i_handle=ceil(19*size(tmp_d,2)/20):size(tmp_d,2)
%             if norm(tmp_d(:,i_handle))>norm(tmp_d(:,i_handle-1))
%                 tmp_d(:,i_handle)=(norm(tmp_d(:,i_handle-1))/norm(tmp_d(:,i_handle))).*tmp_d(:,i_handle);
%             end
%         end
    else
        for j=d+1:2*d
            tmp_d(j-d,:) = demos_V{i}(j,:); 
        end 
    end
    % trimming demonstrations
    ind = find(sqrt(sum(tmp_d.*tmp_d,1))>tol_cutting);
    tmp = tmp(:,min(ind):max(ind)+1); 
    tmp_d = tmp_d(:,min(ind):max(ind));
    
    % saving the initial point of each demo
    V0(:,i) = tmp(:,1);
    tmpv=tmp;
    tmp(:,end)=[];
    tmp(:,end)=zeros(d,1);
    tmp_d(:,end)=zeros(d,1);
    %saving the final point (target) of each demo
    VT(:,i) = tmp(:,end); 
    
    % shifting demos to the origin 
%     tmp = tmp - repmat(VT(:,i),1,size(tmp,2));
    tmp_V_Out_put(:,:,i)=tmp;
    
    % saving demos next to each other 
    Data_V_A_handle = [Data_V_A_handle [tmp;tmp_d]];
    index = [index size(Data_V_A_handle,2)+1];
    % end 
    Data_P_A_handle=[];
    Data_A_handle=[];
    index = 1; 
    % for i=1:length(demos_V)
    clear tmp
     
    % de-noising data (not necessary)
    for j=1:d
        tmp(j,:) = demos_P{i}(j,:); 
    end
    
    % trimming demonstrations
    tmp = tmp(:,min(ind):max(ind));
    
    % saving the initial point of each demo
    X0(:,i) = tmp(:,1);
    
    %saving the final point (target) of each demo
    XT(:,i) = tmp(:,end); 
    
    % shifting demos to the origin
    tmp = tmp - repmat(XT(:,i),1,size(tmp,2));

    % saving demos next to each other
    Data_P_A_handle = [Data_P_A_handle [tmp;tmp_d]];
    Data_A_handle = [Data_A_handle [tmp; tmp_V_Out_put(:,:,i);tmp_d]];
    index = [index size(Data_P_A_handle,2)+1];
if (strcmp(options.Normilizing,'True')==1)
    Data_V_handle=[];
    Data_P_handle=[];
    Step_Size=max(dt.*sqrt(sum(Data_V_A_handle(1:d,:).^2,1)));
    Data_P_handle(1:d,1)=Data_P_A_handle(1:d,1);
    Data_P_handle(d+1:2*d,1)=Data_P_A_handle(d+1:2*d,1);
    Data_V_handle(1:d,1)=Data_V_A_handle(1:d,1);
    Data_V_handle(d+1:2*d,1)=Data_V_A_handle(d+1:2*d,1);

    i_Sub=1;
    handle_P=Data_P_handle(1:d,1);
    Pointer=1;
    while (norm(Data_P_handle(1:d,i_Sub))>0.005)&&(norm(handle_P)>0.01)
        Handle_Norm=100;
        Pointer=10;
        for o=1:length(Data_P_A_handle)
            if norm(Data_P_A_handle(1:d,o)-handle_P)<Handle_Norm
                Handle_Norm=norm(Data_P_A_handle(1:d,o)-handle_P);
                Pointer=o;
            end
        end
        Data_V_handle(1:d,i_Sub+1)=Data_V_A_handle(1:d,Pointer);
        Data_V_handle(d+1:2*d,i_Sub+1)=Data_V_A_handle(d+1:2*d,Pointer);
        Data_P_handle(d+1:2*d,i_Sub+1)=Data_P_A_handle(d+1:2*d,Pointer);
        Data_P_handle(1:d,i_Sub+1)=Data_P_A_handle(1:d,Pointer);
        Vq=Data_V_handle(1:d,i_Sub+1);
        if (Pointer>length(Data_P_A_handle)-2)
            break
        end
        if norm(Vq)~=0
            Vq=Vq/norm(Vq);
        else
            JJJ=1;
            while norm(Vq)==0
                Vq=Data_V_A_handle(1:d,Pointer+JJJ);
                JJJ=JJJ+1;
            end
            Vq=Vq/norm(Vq);
        end
        handle_P=Data_P_handle(1:d,i_Sub+1)+Step_Size*Vq;
        i_Sub=i_Sub+1;
    end
else
    Data_P_handle=Data_P_A_handle;
    Data_V_handle=Data_V_A_handle;
end    
%     if size(Data_P_handle,2)>50
% %         Jump=ceil(length(demos_V)*size(Data_P_handle,2)/50);
% Jump=2;
%         i_Sub=1;
%         counter=1;
%         while i_Sub<size(Data_P_handle,2)
%             P_handle(:,counter)=Data_P_handle(:,i_Sub);
%             V_handle(:,counter)=Data_V_handle(:,i_Sub);
%             i_Sub=i_Sub+Jump;
%             counter=counter+1;
%         end
%     else
        P_handle=Data_P_handle;
        V_handle=Data_V_handle;
%     end
    
    demos_V{i}=V_handle;
    demos_P{i}=P_handle;
    demos_O{i}=[P_handle(1:d,:);V_handle(1:d,:)];
    Data_P=[Data_P P_handle];
    Data_V=[Data_V V_handle];
    
    
end
Data_A=[Data_P(1:d,:);Data_V(1:d,:);Data_P(d+1:2*d,:)];

VT = mean(VT,2); %we consider the mean value of all demonstraions' final point as the target
V0 = mean(V0,2); %the mean value of all demonstrations' initial point
XT = mean(XT,2); %we consider the mean value of all demonstraions' final point as the target
X0 = mean(X0,2); %the mean value of all demonstrations' initial point