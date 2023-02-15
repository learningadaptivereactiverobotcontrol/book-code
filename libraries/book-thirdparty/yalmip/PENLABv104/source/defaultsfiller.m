% fill in default values to the missing fields in penm structure
% so that I don't need to check if they exist
% all except callbacks and xinit, Yinit
function penm=defaultsfiller(penm)

  if (~isfield(penm,'probname') || isempty(penm.probname))
    penm.probname=penlab.default_probname();
  end

  if (~isfield(penm,'comment'))
    penm.comment='';
  end

  if (~isfield(penm,'userdata'))
    penm.userdata=[];
  end

  if (~isfield(penm,'opts'))
    penm.opts=struct();
  end

  if (~isfield(penm,'Nx'))
    penm.Nx=0;
  end

  if (~isfield(penm,'NY'))
    penm.NY=0;
  end

  if (~isfield(penm,'Y'))
    penm.Y=cell(penm.NY,1);
  end

  if (~isfield(penm,'lbx'))
    penm.lbx=-Inf(penm.Nx,1);
  end

  if (~isfield(penm,'ubx'))
    penm.ubx=Inf(penm.Nx,1);
  end

  if (~isfield(penm,'lbxbar'))
    penm.lbxbar=[];
  end

  if (~isfield(penm,'ubxbar'))
    penm.ubxbar=[];
  end

  if (~isfield(penm,'lbY'))
    penm.lbY=-Inf(penm.NY,1);
  end

  if (~isfield(penm,'ubY'))
    penm.ubY=Inf(penm.NY,1);
  end

  if (~isfield(penm,'lbYbar'))
    penm.lbYbar=[];
  end

  if (~isfield(penm,'ubYbar'))
    penm.ubYbar=[];
  end

  if (~isfield(penm,'lbYx'))
    % cell array of empty matrices --> will use default values
    penm.lbYx=cell(penm.NY,1);
  end

  if (~isfield(penm,'ubYx'))
    penm.ubYx=cell(penm.NY,1);
  end

  if (~isfield(penm,'NgNLN'))
    penm.NgNLN=0;
  end

  if (~isfield(penm,'NgLIN'))
    penm.NgLIN=0;
  end

  if (~isfield(penm,'NANLN'))
    penm.NANLN=0;
  end

  if (~isfield(penm,'NALIN'))
    penm.NALIN=0;
  end

  if (~isfield(penm,'lbg'))
    penm.lbg=-Inf(penm.NgNLN + penm.NgLIN,1);
  end

  if (~isfield(penm,'ubg'))
    penm.ubg=Inf(penm.NgNLN + penm.NgLIN,1);
  end

  if (~isfield(penm,'lbA'))
    penm.lbA=-Inf(penm.NANLN + penm.NALIN,1);
  end

  if (~isfield(penm,'ubA'))
    penm.ubA=Inf(penm.NANLN + penm.NALIN,1);
  end

  %if (~isfield(penm,'Y'))
  %  penm.Y=;
  %end

  


