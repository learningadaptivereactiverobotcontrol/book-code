% GENERATE_DOC - to create internal documentation in HTML format.
%   Using m2html from ./doc_generator, produce internal (programmer's)
%   documentation from comments in m-files to ./doc_internal.
%   Start with ./doc_internal/index.html.
%
%   See also m2html.

m2html('mfiles',{'source','utilities','modules'},...
       'htmldir','doc/internal',...
       'recursive','on')

