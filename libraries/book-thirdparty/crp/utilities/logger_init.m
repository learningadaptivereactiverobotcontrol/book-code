function logger = logger_init(filename,level,restart)
% logger = init_logger(filename,restart)
%   creates a logging structure to be passed to log_message
%
%   Args:
%       filename - the log filename, defaults to STDERR
%       level    - only log messages with priority above at or above 
%                - level will be recorded, further
%                - level should be one of DEBUG, INFO, PROGRESS, ERROR, and SEVERE
%                - defaults to 
%       restart  - whether or not to "rewind" the log to the beginning, 
%                  defaults to 0
%   Returns:
%       logger   - struct to pass to subsequent logger_record calls

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

SEVERE=5;
PROBLEM=4;
PROGRESS=3;
INFO=2;
DEBUG=1;

LOGGER_LEVEL = {'DEBUG' 'INFO' 'PROGRESS' 'PROBLEM' 'SEVERE'};

if(nargin < 3)
    restart = 0;
elseif(~(restart==0 | restart==1))
    error('Bad parameter value for restart, must be 1 or 0, type ''help init_logger''')
end

if(nargin < 2)
    level = 'DEBUG';
end

logger.level = find_logger_level(level);

if(nargin < 1)
    logger.fh = 2;
    logger.initialized = 1;
    return
end
logger.initialized =0;
logger.fh=2;

if(restart)
    [logger.fh, mesg] = fopen(filename,'w+');
else
    [logger.fh, mesg] = fopen(filename,'a');
end

if(logger.fh == -1)
    error(sprintf('Couldn''t open log file %s',file))
end

logger.initialized = 1;

function i = find_logger_level(level)
    global LOGGER_LEVEL
    for i=1:length(LOGGER_LEVEL)
        if(upper(LOGGER_LEVEL{i})==upper(level))
            return 
        end
    end
    i=-1;
