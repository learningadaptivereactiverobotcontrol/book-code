%Install_lightspeed
% Compiles mex files for the lightspeed library.

% Written by Tom Minka
% (c) Microsoft Corporation. All rights reserved.

% thanks to Kevin Murphy for suggesting this routine.
% thanks to Ruben Martinez-Cantin for UNDERSCORE_LAPACK_CALL

% Some modifications by Nadia Figueroa and Dominic Reber to run on 
% MAC OS MOJAVE and higher and Windows


fprintf('Compiling lightspeed 2.6 mex files...\n');
fprintf('Change directory to lightspeed for this to work.\n');

% Matlab version
v = sscanf(version,'%d.%d.%*s (R%d) %*s');
% v(3) is the R number
% could also use v(3)>=13
atleast65 = (v(1)>6 || (v(1)==6 && v(2)>=5));
atleast73 = (v(1)>7 || (v(1)==7 && v(2)>=3));
atleast75 = (v(1)>7 || (v(1)==7 && v(2)>=5));
atleast76 = (v(1)>7 || (v(1)==7 && v(2)>=6));
atleast78 = (v(1)>7 || (v(1)==7 && v(2)>=8)); % R2009a
if atleast73
	% largeArrayDims and mwSize were added in version 7.3 (R2006b)
	% http://www.mathworks.com/help/techdoc/rn/bqt6wtq.html
	flags = ' -largeArrayDims ';
else 
	flags = ' -DmwSize=int -DmwIndex=int ';
end

% copy matlab's original repmat.m as xrepmat.m
if exist('xrepmat.m') ~= 2
  w = fullfile(matlabroot,'toolbox','matlab','elmat','repmat.m');
  cmd = ['"' w '" xrepmat.m'];
  if ispc
    system(['copy ' cmd]);
  else
    system(['cp -rp ' cmd]);
  end
end

% these are done first to initialize mex
eval(['mex' flags '-c flops.c']);
eval(['mex' flags 'sameobject.c']);
eval(['mex' flags 'int_hist.c']);
eval(['mex' flags '-c mexutil.c']);
eval(['mex' flags '-c util.c']);

libdir = '';
if ispc
    if (v(1)==9)
        flags = ' -R2017b ';
    end
    
	[compiler,options] = mexcompiler;
    libdir = fullfile(matlabroot, 'extern\lib\win64\mingw64');
	engmatopts = [compiler 'engmatopts.bat'];
    
elseif ismac
	options = struct;
    % Check OS here and make an if-statement
    % OLDER MacOSX versions
	% this installer is set up for 64-bit MacOSX 10.6 with gcc-4.0
	% if you are using something else, run 'mex -v -c flops.c'
	% and use the output to change these strings
% 	options.COMPILER = 'gcc-4.0';
% 	options.COMPFLAGS = '-fno-common -no-cpp-precomp -arch x86_64 -isysroot /Developer/SDKs/MacOSX10.14.sdk -mmacosx-version-min=10.6  -fexceptions';
% 	options.OPTIMFLAGS = '-O -DNDEBUG';
   
    % For MAC OS X with Xcode version > 10.0
    % Find your version of xcode developer tools for the build flags
    
    [~, xcode_path] = system("find /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs -name 'MacOSX1*' ");
    xcode_path = convertCharsToStrings(xcode_path(1:end-1));
    if xcode_path ~= ""
        xcode_version = extractBetween(xcode_path,"SDKs/MacOSX",".sdk");
        
        options.COMPILER = convertStringsToChars(strcat("/usr/bin/xcrun -sdk macosx",xcode_version, " clang"));
        options.COMPFLAGS = convertStringsToChars(strcat("-fno-common -arch x86_64 -mmacosx-version-min=10.9 -fexceptions -isysroot " ,xcode_path, " -DMATLAB_DEFAULT_RELEASE=R2017b  -DUSE_MEX_CMD   -DMATLAB_MEX_FILE"));
        options.OPTIMFLAGS = '-O2 -fwrapv -DNDEBUG';
        
    else
        error("ERROR: Xcode Command Line tools not installed");
        return;
    end

    
else
	options = struct;
	options.COMPILER = 'cc';
	options.COMPFLAGS = '-fPIC';
	options.OPTIMFLAGS = '-O';
end

% Routines that use LAPACK
lapacklib = '';
blaslib = '';
lapackflags = flags;
if atleast78
	lapackflags = [lapackflags ' -DBLAS64'];
end
if ispc
  if strncmp(compiler,'MSVC',4)
    if atleast65
      % version >= 6.5
      lapacklib = fullfile(libdir,'libmwlapack.lib');
    end
  else
    lapacklib = fullfile(libdir,'libmwlapack.lib');
  end
  if atleast75
    blaslib = fullfile(libdir,'libmwblas.lib');
  end
  %%% Paste the location of libmwlapack.lib %%%
  %lapacklib = '';
  if ~exist(lapacklib,'file')
    lapacklib = 'dtrsm.c';
    fprintf('libmwlapack.lib was not found.  To get additional optimizations, paste its location into install_lightspeed.m\n');
  else
    fprintf('Using the lapack library at %s\n',lapacklib);
  end
else
  % in version 7.5, non-PC systems do not need to specify lapacklib, 
  % but they must use an underscore when calling lapack routines
	% http://www.mathworks.com/help/techdoc/matlab_external/br_2m24-1.html
	lapackflags = [lapackflags ' -DUNDERSCORE_LAPACK_CALL'];
  if atleast76
    lapacklib = '-lmwlapack';
    blaslib = '-lmwblas';
  end
end
disp(['mex' lapackflags ' solve_triu.c "' lapacklib '" "' blaslib '"'])
eval(['mex' lapackflags ' solve_triu.c "' lapacklib '" "' blaslib '"']);
eval(['mex' lapackflags ' solve_tril.c "' lapacklib '" "' blaslib '"']);

if ispc
  % Windows
  %if exist('util.obj','file')
  eval(['mex' flags 'addflops.c flops.obj'])
	if atleast78
		eval(['mex' flags 'gammaln.c util.obj -outdir @double'])
	else
		eval(['mex' flags 'gammaln.c util.obj'])
	end
  eval(['mex' flags 'digamma.c util.obj'])
  eval(['mex' flags 'trigamma.c util.obj'])
  eval(['mex' flags 'tetragamma.c util.obj'])
	eval(['mex' flags 'setnonzeros.c'])
  if strncmp(compiler,'MSVC',4)
		clear random.dll randomseed randbinom randgamma sample_hist
    disp(['install_random.bat "' options.VSINSTALLDIR '" ' options.vcvarsopts]);
    system(['install_random.bat "' options.VSINSTALLDIR '" ' options.vcvarsopts]);
    eval(['mex' flags 'randomseed.c util.obj random.lib'])
    eval(['mex' flags 'randbinom.c mexutil.obj util.obj random.lib'])
    eval(['mex' flags 'randgamma.c mexutil.obj util.obj random.lib'])
    eval(['mex' flags 'sample_hist.c util.obj random.lib'])
  else
    fprintf('mexcompiler is not MSVC. The randomseed() function will have no effect.');
    eval(['mex' flags 'randomseed.c util.obj random.c'])
    eval(['mex' flags 'randbinom.c mexutil.obj util.obj random.c'])
    eval(['mex' flags 'randgamma.c mexutil.obj util.obj random.c'])
    eval(['mex' flags 'sample_hist.c util.obj random.c'])
  end
else
  % UNIX
  eval(['mex' flags 'addflops.c flops.o'])
  eval(['mex' flags 'gammaln.c util.o -lm -outdir @double'])
  eval(['mex' flags 'digamma.c util.o -lm'])
  eval(['mex' flags 'trigamma.c util.o -lm'])
  eval(['mex' flags 'tetragamma.c util.o -lm'])
	eval(['mex' flags 'setnonzeros.c'])
  if ismac
    disp('Doing the mac stuff.........~+!!!!');
    % thanks to Nicholas Butko for these mac-specific lines
		clear librandom.dylib randomseed randbinom randgamma sample_hist
		cmd = [options.COMPILER{1,1} ' -c random.c; ' options.COMPILER{1,1} ' -dynamiclib -Wl,-install_name,`pwd`/librandom.dylib -o librandom.dylib random.o'];
		disp(cmd);
		system(cmd)
    eval(['mex' flags 'randomseed.c util.o librandom.dylib -lm'])
    eval(['mex' flags 'randbinom.c mexutil.o util.o librandom.dylib -lm'])
    eval(['mex' flags 'randgamma.c mexutil.o util.o librandom.dylib -lm'])
    eval(['mex' flags 'sample_hist.c util.o librandom.dylib -lm'])
  else
    % this command only works on linux
    clear librandom.so randomseed randbinom randgamma sample_hist
    cmd = [options.COMPILER ' ' options.COMPFLAGS ' ' options.OPTIMFLAGS ' -c random.c; ' options.COMPILER ' ' options.COMPFLAGS ' -shared -Wl,-E -Wl,-soname,"`pwd`/librandom.so" -o librandom.so random.o'];
		disp(cmd);
		system(cmd)
    eval(['mex' flags 'randomseed.c util.o librandom.so -lm'])
    eval(['mex' flags 'randbinom.c mexutil.o util.o librandom.so -lm'])
    eval(['mex' flags 'randgamma.c mexutil.o util.o librandom.so -lm'])
    eval(['mex' flags 'sample_hist.c util.o librandom.so -lm'])
  end
  eval(['mex' flags 'repmat.c mexutil.o'])
end

addpath(genpath(pwd))
fprintf('Done.\n');
fprintf('Type "test_lightspeed" to verify the installation.\n');
