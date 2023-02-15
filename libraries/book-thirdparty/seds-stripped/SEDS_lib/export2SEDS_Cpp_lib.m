function out = export2SEDS_Cpp_lib(name,Priors,Mu,Sigma)
dlmwrite(name, size(Mu),'Delimiter',' ','precision','%i');
dlmwrite(name, Priors,'newline','pc','-append','Delimiter',' ','precision','%.6f');
%dlmwrite(name, reshape(Mu/1000,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, reshape(Mu,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
for i=1:size(Mu,2)
    %dlmwrite(name, reshape(Sigma(:,:,i)/1000^2,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
    dlmwrite(name, reshape(Sigma(:,:,i)',1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
end
out = true;