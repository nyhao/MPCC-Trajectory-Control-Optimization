% FORCESNLPsolver : A fast customized optimization solver.
% 
% Copyright (C) 2013-2016 EMBOTECH GMBH [info@embotech.com]. All rights reserved.
% 
% 
% This software is intended for simulation and testing purposes only. 
% Use of this software for any commercial purpose is prohibited.
% 
% This program is distributed in the hope that it will be useful.
% EMBOTECH makes NO WARRANTIES with respect to the use of the software 
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
% PARTICULAR PURPOSE. 
% 
% EMBOTECH shall not have any liability for any damage arising from the use
% of the software.
% 
% This Agreement shall exclusively be governed by and interpreted in 
% accordance with the laws of Switzerland, excluding its principles
% of conflict of laws. The Courts of Zurich-City shall have exclusive 
% jurisdiction in case of any dispute.
% 

if exist( '../src/FORCESNLPsolver.c', 'file' )
    mex -c -O -DUSEMEXPRINTS ../src/FORCESNLPsolver.c 
mex -c -O -DMEXARGMUENTCHECKS FORCESNLPsolver_mex.c 
if( ispc )
    mex FORCESNLPsolver.obj FORCESNLPsolver_mex.obj -output "FORCESNLPsolver" 
    delete('*.obj');
elseif( ismac )
    mex FORCESNLPsolver.o FORCESNLPsolver_mex.o -output "FORCESNLPsolver"
    delete('*.o');
else % we're on a linux system
    mex FORCESNLPsolver.o FORCESNLPsolver_mex.o -output "FORCESNLPsolver" -lrt
    delete('*.o');
end
copyfile(['FORCESNLPsolver.',mexext], ['../../FORCESNLPsolver.',mexext], 'f');
copyfile( 'FORCESNLPsolver.m', '../../FORCESNLPsolver.m','f');
else
    fprintf('Could not find source file. This file is meant to be used for building from source code.');
end
