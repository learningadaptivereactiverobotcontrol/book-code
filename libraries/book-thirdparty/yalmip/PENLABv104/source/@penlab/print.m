% Publisher (printing manager) for PenLab
% Print 'msg' with optional data in varargin to the screen, to a log file or 
% using the user's printing routine; depending on printing level (outlev) set 
% in the option settings.
% Input:
%   minlev, maxlev ... minimum (inclusive) and maximum (exclusive) output level
%      when the message gets printed, levels are typically 1~5, use Inf if there
%      is no maximal restriction
%   msg, varargin ... message in the sprintf/fprintf format, \n is added
% Output:
%   errmsg ... error message from sprintf (if any)
%
function [errmsg] = print(obj, minlev, maxlev, msg, varargin)

  errmsg=[];
  [text, errmsg] = sprintf(msg,varargin{:});

  level=obj.allopts.outlev;
  if (level>=minlev && level<maxlev)
    disp(text);
  end

  level=obj.allopts.outlev_file;
  if (obj.fid~=-1 && level>=minlev && level<maxlev)
    fprintf(obj.fid,[text '\n']);
  end
  
  if (~isempty(obj.allopts.user_prn))
    user_prn(minlev, maxlev, text);
  end

