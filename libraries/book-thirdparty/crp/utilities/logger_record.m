function logger_record(logger,level,message)
% logger_record(logger,level,message)
%   creates a logging structure to be passed to log_message
%
%   Args:
%       logger   - the logger previously initialized by logger_init
%       level    - log message priority level
%                - level should be one of 'DEBUG', 'INFO', 'PROGRESS', 'ERROR', and 'SEVERE'
%       message  - log message
%   Returns:
%       none  

% Copyright October, 2006, Brown University, Providence, RI. 
% All Rights Reserved 

% Permission to use, copy, modify, and distribute this software and its
% documentation for any purpose other than its incorporation into a commercial
% product is hereby granted without fee, provided that the above copyright
% notice appear in all copies and that both that copyright notice and this
% permission notice appear in supporting documentation, and that the name of
% Brown University not be used in advertising or publicity pertaining to
% distribution of the software without specific, written prior permission. 

% BROWN UNIVERSITY DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
% INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR ANY
% PARTICULAR PURPOSE. IN NO EVENT SHALL BROWN UNIVERSITY BE LIABLE FOR ANY
% SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
% RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF
% CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
% CONNECTION WITH THE USE.

% Author: Frank Wood fwood@cs.brown.edu

global LOGGER_LEVEL

if(nargin < 3)
    error('Too few arguments to ''logger_record'', type ''help logger_record''')
end

if(record_log_message(logger,level))
    
    try
        if(logger.fh < 1)
            error('Bad logger file handle')
        end
        
        
        [st,i] = dbstack;
        if(length(st)<2)
            script_name = 'Command line';
            script_line = -1;
        else
            script_name = st(2).name;
            script_line = st(2).line;
        end
        
        % script, line, level, message
        fprintf(logger.fh,sprintf('%s\t::\t%s:%d\t::\t%s\n', level, strrep(script_name,'\','/'), script_line, message));
        
    catch
        
        error(sprintf('%s \nProbable: Logger not properly initialized',lasterr))
    end
    
else
    error('Too low priority to log')
end



% checks to see whether or not the message should be logged according 
% to the logger class variable
function do_log = record_log_message(logger,level)
do_log =0;
global LOGGER_LEVEL
found_index = -1;
for i=1:length(LOGGER_LEVEL)
    if(length(LOGGER_LEVEL{i})==length(level) & (upper(LOGGER_LEVEL{i})==upper(level)))
        found_index=i; 
    end
end
if(found_index > 0 & found_index > logger.level)
    do_log = 1;
end

