% print all options from the object via obj.print() and add a flag if it is
% a default setting or changed by user from obj.opts
% If minlev/maxlev specified, print only on these levels, if not all (0-Inf)
function [errmsg] = print_opts(obj, minlev, maxlev)
  
  errmsg = [];
  if (nargin<=1)
    minlev=0;
    maxlev=Inf;
  elseif (nargin<=2)
    maxlev=Inf;
  end

  errmsg=obj.print(minlev,maxlev,'All option settings (* = set by user):');

  % create an array of flags if the option is coming from user or as default
  optnames = fieldnames(obj.allopts);
  usrset = isfield(obj.opts, optnames);
  no_opts=length(optnames);

  for i=1:no_opts
    fld=obj.allopts.(optnames{i});
    if (usrset(i))
      flag='*';
    else
      flag=' ';
    end
    if (isempty(fld))
      str='[not used]';
    elseif (isnumeric(fld))   
      str=sprintf('%g ',fld);  % this will work even with arrays
    elseif (ischar(fld))
      str=fld;
    else
      str='[other type, in use]'
    end
    errmsg=obj.print(minlev,maxlev,'  %-20s %s: %s',optnames{i},flag,str);
  end

