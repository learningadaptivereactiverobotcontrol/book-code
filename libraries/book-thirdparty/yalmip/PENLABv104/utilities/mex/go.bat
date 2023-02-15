call mex.bat -g -DMEMORY_MATLAB %1 %2 %3 -Iinc -DASL_NO_FPINITMT my_amplfunc4.c lib/amplsolver.a

echo. 
echo Copy the compiled result to pennonM/algorithm as 'amplf' with the same suffix.
echo.

