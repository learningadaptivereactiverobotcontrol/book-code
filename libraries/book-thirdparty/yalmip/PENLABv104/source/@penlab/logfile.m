% open/close log file based on option settings (in obj.allopts)
% (task==0 -> close, otherwise->open)
function [] = logfile(obj,task)

  % first of all, close it if it is open
  if (obj.fid~=-1)
    %disp('PenLAB DBG: closing file');
    fclose(obj.fid);
    obj.fid=-1;
  end

  if (task && obj.allopts.outlev_file>0 && ~isempty(obj.allopts.out_filename))
    obj.fid = fopen(obj.allopts.out_filename, 'wt');
    if (obj.fid==-1)
      disp('PenLAB DBG: warning: cannot open the log file');
      warning('PenLAB: Cannot open the log file for writing!')
    end
  end

