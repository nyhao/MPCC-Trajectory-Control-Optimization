#FORCESNLPsolver : A fast customized optimization solver.
#
#Copyright (C) 2013-2016 EMBOTECH GMBH [info@embotech.com]. All rights reserved.
#
#
#This software is intended for simulation and testing purposes only. 
#Use of this software for any commercial purpose is prohibited.
#
#This program is distributed in the hope that it will be useful.
#EMBOTECH makes NO WARRANTIES with respect to the use of the software 
#without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
#PARTICULAR PURPOSE. 
#
#EMBOTECH shall not have any liability for any damage arising from the use
#of the software.
#
#This Agreement shall exclusively be governed by and interpreted in 
#accordance with the laws of Switzerland, excluding its principles
#of conflict of laws. The Courts of Zurich-City shall have exclusive 
#jurisdiction in case of any dispute.
#
#def __init__():
'''
a Python wrapper for a fast solver generated by FORCES Pro v1.6.104

   OUTPUT = FORCESNLPsolver_py.FORCESNLPsolver_solve(PARAMS) solves a multistage problem
   subject to the parameters supplied in the following dictionary:
       PARAMS['x0'] - column vector of length 5313
       PARAMS['xinit'] - column vector of length 26
       PARAMS['xfinal'] - column vector of length 6
       PARAMS['all_parameters'] - column vector of length 6440

   OUTPUT returns the values of the last iteration of the solver where
       OUTPUT['x001'] - column vector of size 33
       OUTPUT['x002'] - column vector of size 33
       OUTPUT['x003'] - column vector of size 33
       OUTPUT['x004'] - column vector of size 33
       OUTPUT['x005'] - column vector of size 33
       OUTPUT['x006'] - column vector of size 33
       OUTPUT['x007'] - column vector of size 33
       OUTPUT['x008'] - column vector of size 33
       OUTPUT['x009'] - column vector of size 33
       OUTPUT['x010'] - column vector of size 33
       OUTPUT['x011'] - column vector of size 33
       OUTPUT['x012'] - column vector of size 33
       OUTPUT['x013'] - column vector of size 33
       OUTPUT['x014'] - column vector of size 33
       OUTPUT['x015'] - column vector of size 33
       OUTPUT['x016'] - column vector of size 33
       OUTPUT['x017'] - column vector of size 33
       OUTPUT['x018'] - column vector of size 33
       OUTPUT['x019'] - column vector of size 33
       OUTPUT['x020'] - column vector of size 33
       OUTPUT['x021'] - column vector of size 33
       OUTPUT['x022'] - column vector of size 33
       OUTPUT['x023'] - column vector of size 33
       OUTPUT['x024'] - column vector of size 33
       OUTPUT['x025'] - column vector of size 33
       OUTPUT['x026'] - column vector of size 33
       OUTPUT['x027'] - column vector of size 33
       OUTPUT['x028'] - column vector of size 33
       OUTPUT['x029'] - column vector of size 33
       OUTPUT['x030'] - column vector of size 33
       OUTPUT['x031'] - column vector of size 33
       OUTPUT['x032'] - column vector of size 33
       OUTPUT['x033'] - column vector of size 33
       OUTPUT['x034'] - column vector of size 33
       OUTPUT['x035'] - column vector of size 33
       OUTPUT['x036'] - column vector of size 33
       OUTPUT['x037'] - column vector of size 33
       OUTPUT['x038'] - column vector of size 33
       OUTPUT['x039'] - column vector of size 33
       OUTPUT['x040'] - column vector of size 33
       OUTPUT['x041'] - column vector of size 33
       OUTPUT['x042'] - column vector of size 33
       OUTPUT['x043'] - column vector of size 33
       OUTPUT['x044'] - column vector of size 33
       OUTPUT['x045'] - column vector of size 33
       OUTPUT['x046'] - column vector of size 33
       OUTPUT['x047'] - column vector of size 33
       OUTPUT['x048'] - column vector of size 33
       OUTPUT['x049'] - column vector of size 33
       OUTPUT['x050'] - column vector of size 33
       OUTPUT['x051'] - column vector of size 33
       OUTPUT['x052'] - column vector of size 33
       OUTPUT['x053'] - column vector of size 33
       OUTPUT['x054'] - column vector of size 33
       OUTPUT['x055'] - column vector of size 33
       OUTPUT['x056'] - column vector of size 33
       OUTPUT['x057'] - column vector of size 33
       OUTPUT['x058'] - column vector of size 33
       OUTPUT['x059'] - column vector of size 33
       OUTPUT['x060'] - column vector of size 33
       OUTPUT['x061'] - column vector of size 33
       OUTPUT['x062'] - column vector of size 33
       OUTPUT['x063'] - column vector of size 33
       OUTPUT['x064'] - column vector of size 33
       OUTPUT['x065'] - column vector of size 33
       OUTPUT['x066'] - column vector of size 33
       OUTPUT['x067'] - column vector of size 33
       OUTPUT['x068'] - column vector of size 33
       OUTPUT['x069'] - column vector of size 33
       OUTPUT['x070'] - column vector of size 33
       OUTPUT['x071'] - column vector of size 33
       OUTPUT['x072'] - column vector of size 33
       OUTPUT['x073'] - column vector of size 33
       OUTPUT['x074'] - column vector of size 33
       OUTPUT['x075'] - column vector of size 33
       OUTPUT['x076'] - column vector of size 33
       OUTPUT['x077'] - column vector of size 33
       OUTPUT['x078'] - column vector of size 33
       OUTPUT['x079'] - column vector of size 33
       OUTPUT['x080'] - column vector of size 33
       OUTPUT['x081'] - column vector of size 33
       OUTPUT['x082'] - column vector of size 33
       OUTPUT['x083'] - column vector of size 33
       OUTPUT['x084'] - column vector of size 33
       OUTPUT['x085'] - column vector of size 33
       OUTPUT['x086'] - column vector of size 33
       OUTPUT['x087'] - column vector of size 33
       OUTPUT['x088'] - column vector of size 33
       OUTPUT['x089'] - column vector of size 33
       OUTPUT['x090'] - column vector of size 33
       OUTPUT['x091'] - column vector of size 33
       OUTPUT['x092'] - column vector of size 33
       OUTPUT['x093'] - column vector of size 33
       OUTPUT['x094'] - column vector of size 33
       OUTPUT['x095'] - column vector of size 33
       OUTPUT['x096'] - column vector of size 33
       OUTPUT['x097'] - column vector of size 33
       OUTPUT['x098'] - column vector of size 33
       OUTPUT['x099'] - column vector of size 33
       OUTPUT['x100'] - column vector of size 33
       OUTPUT['x101'] - column vector of size 33
       OUTPUT['x102'] - column vector of size 33
       OUTPUT['x103'] - column vector of size 33
       OUTPUT['x104'] - column vector of size 33
       OUTPUT['x105'] - column vector of size 33
       OUTPUT['x106'] - column vector of size 33
       OUTPUT['x107'] - column vector of size 33
       OUTPUT['x108'] - column vector of size 33
       OUTPUT['x109'] - column vector of size 33
       OUTPUT['x110'] - column vector of size 33
       OUTPUT['x111'] - column vector of size 33
       OUTPUT['x112'] - column vector of size 33
       OUTPUT['x113'] - column vector of size 33
       OUTPUT['x114'] - column vector of size 33
       OUTPUT['x115'] - column vector of size 33
       OUTPUT['x116'] - column vector of size 33
       OUTPUT['x117'] - column vector of size 33
       OUTPUT['x118'] - column vector of size 33
       OUTPUT['x119'] - column vector of size 33
       OUTPUT['x120'] - column vector of size 33
       OUTPUT['x121'] - column vector of size 33
       OUTPUT['x122'] - column vector of size 33
       OUTPUT['x123'] - column vector of size 33
       OUTPUT['x124'] - column vector of size 33
       OUTPUT['x125'] - column vector of size 33
       OUTPUT['x126'] - column vector of size 33
       OUTPUT['x127'] - column vector of size 33
       OUTPUT['x128'] - column vector of size 33
       OUTPUT['x129'] - column vector of size 33
       OUTPUT['x130'] - column vector of size 33
       OUTPUT['x131'] - column vector of size 33
       OUTPUT['x132'] - column vector of size 33
       OUTPUT['x133'] - column vector of size 33
       OUTPUT['x134'] - column vector of size 33
       OUTPUT['x135'] - column vector of size 33
       OUTPUT['x136'] - column vector of size 33
       OUTPUT['x137'] - column vector of size 33
       OUTPUT['x138'] - column vector of size 33
       OUTPUT['x139'] - column vector of size 33
       OUTPUT['x140'] - column vector of size 33
       OUTPUT['x141'] - column vector of size 33
       OUTPUT['x142'] - column vector of size 33
       OUTPUT['x143'] - column vector of size 33
       OUTPUT['x144'] - column vector of size 33
       OUTPUT['x145'] - column vector of size 33
       OUTPUT['x146'] - column vector of size 33
       OUTPUT['x147'] - column vector of size 33
       OUTPUT['x148'] - column vector of size 33
       OUTPUT['x149'] - column vector of size 33
       OUTPUT['x150'] - column vector of size 33
       OUTPUT['x151'] - column vector of size 33
       OUTPUT['x152'] - column vector of size 33
       OUTPUT['x153'] - column vector of size 33
       OUTPUT['x154'] - column vector of size 33
       OUTPUT['x155'] - column vector of size 33
       OUTPUT['x156'] - column vector of size 33
       OUTPUT['x157'] - column vector of size 33
       OUTPUT['x158'] - column vector of size 33
       OUTPUT['x159'] - column vector of size 33
       OUTPUT['x160'] - column vector of size 33
       OUTPUT['x161'] - column vector of size 33

   [OUTPUT, EXITFLAG] = FORCESNLPsolver_py.FORCESNLPsolver_solve(PARAMS) returns additionally
   the integer EXITFLAG indicating the state of the solution with 
       1 - Optimal solution has been found (subject to desired accuracy)
       2 - (only branch-and-bound) A feasible point has been identified for which the objective value is no more than codeoptions.mip.mipgap*100 per cent worse than the global optimum 
       0 - Timeout - maximum number of iterations reached
      -1 - (only branch-and-bound) Infeasible problem (problems solving the root relaxation to the desired accuracy)
      -2 - (only branch-and-bound) Out of memory - cannot fit branch and bound nodes into pre-allocated memory.
      -6 - NaN or INF occured during evaluation of functions and derivatives. Please check your initial guess.
      -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
     -10 - The convex solver could not proceed due to an internal error
    -100 - License error

   [OUTPUT, EXITFLAG, INFO] = FORCESNLPsolver_py.FORCESNLPsolver_solve(PARAMS) returns 
   additional information about the last iterate:
       INFO.it        - number of iterations that lead to this result
       INFO.it2opt    - number of convex solves
       INFO.res_eq    - max. equality constraint residual
       INFO.res_ineq  - max. inequality constraint residual
       INFO.pobj      - primal objective
       INFO.mu        - duality measure
       INFO.solvetime - Time needed for solve (wall clock time)
       INFO.fevalstime - Time needed for function evaluations (wall clock time)

 See also COPYING

'''

import ctypes
import os
import numpy as np
import numpy.ctypeslib as npct
import sys

#_lib = ctypes.CDLL(os.path.join(os.getcwd(),'FORCESNLPsolver/lib/FORCESNLPsolver.so')) 
try:
	_lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)),'FORCESNLPsolver/lib/FORCESNLPsolver_withModel.so'))
	csolver = getattr(_lib,'FORCESNLPsolver_solve')
except:
	_lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)),'FORCESNLPsolver/lib/libFORCESNLPsolver_withModel.so'))
	csolver = getattr(_lib,'FORCESNLPsolver_solve')

class FORCESNLPsolver_params_ctypes(ctypes.Structure):
#	@classmethod
#	def from_param(self):
#		return self
	_fields_ = [('x0', ctypes.c_double * 5313),
('xinit', ctypes.c_double * 26),
('xfinal', ctypes.c_double * 6),
('all_parameters', ctypes.c_double * 6440),
]

FORCESNLPsolver_params = {'x0' : np.array([]),
'xinit' : np.array([]),
'xfinal' : np.array([]),
'all_parameters' : np.array([]),
}
params = {'x0' : np.array([]),
'xinit' : np.array([]),
'xfinal' : np.array([]),
'all_parameters' : np.array([]),
}

class FORCESNLPsolver_outputs_ctypes(ctypes.Structure):
#	@classmethod
#	def from_param(self):
#		return self
	_fields_ = [('x001', ctypes.c_double * 33),
('x002', ctypes.c_double * 33),
('x003', ctypes.c_double * 33),
('x004', ctypes.c_double * 33),
('x005', ctypes.c_double * 33),
('x006', ctypes.c_double * 33),
('x007', ctypes.c_double * 33),
('x008', ctypes.c_double * 33),
('x009', ctypes.c_double * 33),
('x010', ctypes.c_double * 33),
('x011', ctypes.c_double * 33),
('x012', ctypes.c_double * 33),
('x013', ctypes.c_double * 33),
('x014', ctypes.c_double * 33),
('x015', ctypes.c_double * 33),
('x016', ctypes.c_double * 33),
('x017', ctypes.c_double * 33),
('x018', ctypes.c_double * 33),
('x019', ctypes.c_double * 33),
('x020', ctypes.c_double * 33),
('x021', ctypes.c_double * 33),
('x022', ctypes.c_double * 33),
('x023', ctypes.c_double * 33),
('x024', ctypes.c_double * 33),
('x025', ctypes.c_double * 33),
('x026', ctypes.c_double * 33),
('x027', ctypes.c_double * 33),
('x028', ctypes.c_double * 33),
('x029', ctypes.c_double * 33),
('x030', ctypes.c_double * 33),
('x031', ctypes.c_double * 33),
('x032', ctypes.c_double * 33),
('x033', ctypes.c_double * 33),
('x034', ctypes.c_double * 33),
('x035', ctypes.c_double * 33),
('x036', ctypes.c_double * 33),
('x037', ctypes.c_double * 33),
('x038', ctypes.c_double * 33),
('x039', ctypes.c_double * 33),
('x040', ctypes.c_double * 33),
('x041', ctypes.c_double * 33),
('x042', ctypes.c_double * 33),
('x043', ctypes.c_double * 33),
('x044', ctypes.c_double * 33),
('x045', ctypes.c_double * 33),
('x046', ctypes.c_double * 33),
('x047', ctypes.c_double * 33),
('x048', ctypes.c_double * 33),
('x049', ctypes.c_double * 33),
('x050', ctypes.c_double * 33),
('x051', ctypes.c_double * 33),
('x052', ctypes.c_double * 33),
('x053', ctypes.c_double * 33),
('x054', ctypes.c_double * 33),
('x055', ctypes.c_double * 33),
('x056', ctypes.c_double * 33),
('x057', ctypes.c_double * 33),
('x058', ctypes.c_double * 33),
('x059', ctypes.c_double * 33),
('x060', ctypes.c_double * 33),
('x061', ctypes.c_double * 33),
('x062', ctypes.c_double * 33),
('x063', ctypes.c_double * 33),
('x064', ctypes.c_double * 33),
('x065', ctypes.c_double * 33),
('x066', ctypes.c_double * 33),
('x067', ctypes.c_double * 33),
('x068', ctypes.c_double * 33),
('x069', ctypes.c_double * 33),
('x070', ctypes.c_double * 33),
('x071', ctypes.c_double * 33),
('x072', ctypes.c_double * 33),
('x073', ctypes.c_double * 33),
('x074', ctypes.c_double * 33),
('x075', ctypes.c_double * 33),
('x076', ctypes.c_double * 33),
('x077', ctypes.c_double * 33),
('x078', ctypes.c_double * 33),
('x079', ctypes.c_double * 33),
('x080', ctypes.c_double * 33),
('x081', ctypes.c_double * 33),
('x082', ctypes.c_double * 33),
('x083', ctypes.c_double * 33),
('x084', ctypes.c_double * 33),
('x085', ctypes.c_double * 33),
('x086', ctypes.c_double * 33),
('x087', ctypes.c_double * 33),
('x088', ctypes.c_double * 33),
('x089', ctypes.c_double * 33),
('x090', ctypes.c_double * 33),
('x091', ctypes.c_double * 33),
('x092', ctypes.c_double * 33),
('x093', ctypes.c_double * 33),
('x094', ctypes.c_double * 33),
('x095', ctypes.c_double * 33),
('x096', ctypes.c_double * 33),
('x097', ctypes.c_double * 33),
('x098', ctypes.c_double * 33),
('x099', ctypes.c_double * 33),
('x100', ctypes.c_double * 33),
('x101', ctypes.c_double * 33),
('x102', ctypes.c_double * 33),
('x103', ctypes.c_double * 33),
('x104', ctypes.c_double * 33),
('x105', ctypes.c_double * 33),
('x106', ctypes.c_double * 33),
('x107', ctypes.c_double * 33),
('x108', ctypes.c_double * 33),
('x109', ctypes.c_double * 33),
('x110', ctypes.c_double * 33),
('x111', ctypes.c_double * 33),
('x112', ctypes.c_double * 33),
('x113', ctypes.c_double * 33),
('x114', ctypes.c_double * 33),
('x115', ctypes.c_double * 33),
('x116', ctypes.c_double * 33),
('x117', ctypes.c_double * 33),
('x118', ctypes.c_double * 33),
('x119', ctypes.c_double * 33),
('x120', ctypes.c_double * 33),
('x121', ctypes.c_double * 33),
('x122', ctypes.c_double * 33),
('x123', ctypes.c_double * 33),
('x124', ctypes.c_double * 33),
('x125', ctypes.c_double * 33),
('x126', ctypes.c_double * 33),
('x127', ctypes.c_double * 33),
('x128', ctypes.c_double * 33),
('x129', ctypes.c_double * 33),
('x130', ctypes.c_double * 33),
('x131', ctypes.c_double * 33),
('x132', ctypes.c_double * 33),
('x133', ctypes.c_double * 33),
('x134', ctypes.c_double * 33),
('x135', ctypes.c_double * 33),
('x136', ctypes.c_double * 33),
('x137', ctypes.c_double * 33),
('x138', ctypes.c_double * 33),
('x139', ctypes.c_double * 33),
('x140', ctypes.c_double * 33),
('x141', ctypes.c_double * 33),
('x142', ctypes.c_double * 33),
('x143', ctypes.c_double * 33),
('x144', ctypes.c_double * 33),
('x145', ctypes.c_double * 33),
('x146', ctypes.c_double * 33),
('x147', ctypes.c_double * 33),
('x148', ctypes.c_double * 33),
('x149', ctypes.c_double * 33),
('x150', ctypes.c_double * 33),
('x151', ctypes.c_double * 33),
('x152', ctypes.c_double * 33),
('x153', ctypes.c_double * 33),
('x154', ctypes.c_double * 33),
('x155', ctypes.c_double * 33),
('x156', ctypes.c_double * 33),
('x157', ctypes.c_double * 33),
('x158', ctypes.c_double * 33),
('x159', ctypes.c_double * 33),
('x160', ctypes.c_double * 33),
('x161', ctypes.c_double * 33),
]

FORCESNLPsolver_outputs = {'x001' : np.array([]),
'x002' : np.array([]),
'x003' : np.array([]),
'x004' : np.array([]),
'x005' : np.array([]),
'x006' : np.array([]),
'x007' : np.array([]),
'x008' : np.array([]),
'x009' : np.array([]),
'x010' : np.array([]),
'x011' : np.array([]),
'x012' : np.array([]),
'x013' : np.array([]),
'x014' : np.array([]),
'x015' : np.array([]),
'x016' : np.array([]),
'x017' : np.array([]),
'x018' : np.array([]),
'x019' : np.array([]),
'x020' : np.array([]),
'x021' : np.array([]),
'x022' : np.array([]),
'x023' : np.array([]),
'x024' : np.array([]),
'x025' : np.array([]),
'x026' : np.array([]),
'x027' : np.array([]),
'x028' : np.array([]),
'x029' : np.array([]),
'x030' : np.array([]),
'x031' : np.array([]),
'x032' : np.array([]),
'x033' : np.array([]),
'x034' : np.array([]),
'x035' : np.array([]),
'x036' : np.array([]),
'x037' : np.array([]),
'x038' : np.array([]),
'x039' : np.array([]),
'x040' : np.array([]),
'x041' : np.array([]),
'x042' : np.array([]),
'x043' : np.array([]),
'x044' : np.array([]),
'x045' : np.array([]),
'x046' : np.array([]),
'x047' : np.array([]),
'x048' : np.array([]),
'x049' : np.array([]),
'x050' : np.array([]),
'x051' : np.array([]),
'x052' : np.array([]),
'x053' : np.array([]),
'x054' : np.array([]),
'x055' : np.array([]),
'x056' : np.array([]),
'x057' : np.array([]),
'x058' : np.array([]),
'x059' : np.array([]),
'x060' : np.array([]),
'x061' : np.array([]),
'x062' : np.array([]),
'x063' : np.array([]),
'x064' : np.array([]),
'x065' : np.array([]),
'x066' : np.array([]),
'x067' : np.array([]),
'x068' : np.array([]),
'x069' : np.array([]),
'x070' : np.array([]),
'x071' : np.array([]),
'x072' : np.array([]),
'x073' : np.array([]),
'x074' : np.array([]),
'x075' : np.array([]),
'x076' : np.array([]),
'x077' : np.array([]),
'x078' : np.array([]),
'x079' : np.array([]),
'x080' : np.array([]),
'x081' : np.array([]),
'x082' : np.array([]),
'x083' : np.array([]),
'x084' : np.array([]),
'x085' : np.array([]),
'x086' : np.array([]),
'x087' : np.array([]),
'x088' : np.array([]),
'x089' : np.array([]),
'x090' : np.array([]),
'x091' : np.array([]),
'x092' : np.array([]),
'x093' : np.array([]),
'x094' : np.array([]),
'x095' : np.array([]),
'x096' : np.array([]),
'x097' : np.array([]),
'x098' : np.array([]),
'x099' : np.array([]),
'x100' : np.array([]),
'x101' : np.array([]),
'x102' : np.array([]),
'x103' : np.array([]),
'x104' : np.array([]),
'x105' : np.array([]),
'x106' : np.array([]),
'x107' : np.array([]),
'x108' : np.array([]),
'x109' : np.array([]),
'x110' : np.array([]),
'x111' : np.array([]),
'x112' : np.array([]),
'x113' : np.array([]),
'x114' : np.array([]),
'x115' : np.array([]),
'x116' : np.array([]),
'x117' : np.array([]),
'x118' : np.array([]),
'x119' : np.array([]),
'x120' : np.array([]),
'x121' : np.array([]),
'x122' : np.array([]),
'x123' : np.array([]),
'x124' : np.array([]),
'x125' : np.array([]),
'x126' : np.array([]),
'x127' : np.array([]),
'x128' : np.array([]),
'x129' : np.array([]),
'x130' : np.array([]),
'x131' : np.array([]),
'x132' : np.array([]),
'x133' : np.array([]),
'x134' : np.array([]),
'x135' : np.array([]),
'x136' : np.array([]),
'x137' : np.array([]),
'x138' : np.array([]),
'x139' : np.array([]),
'x140' : np.array([]),
'x141' : np.array([]),
'x142' : np.array([]),
'x143' : np.array([]),
'x144' : np.array([]),
'x145' : np.array([]),
'x146' : np.array([]),
'x147' : np.array([]),
'x148' : np.array([]),
'x149' : np.array([]),
'x150' : np.array([]),
'x151' : np.array([]),
'x152' : np.array([]),
'x153' : np.array([]),
'x154' : np.array([]),
'x155' : np.array([]),
'x156' : np.array([]),
'x157' : np.array([]),
'x158' : np.array([]),
'x159' : np.array([]),
'x160' : np.array([]),
'x161' : np.array([]),
}


class FORCESNLPsolver_info(ctypes.Structure):
#	@classmethod
#	def from_param(self):
#		return self
	_fields_ = [('it', ctypes.c_int),
('it2opt', ctypes.c_int),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('pobj',ctypes.c_double),
('dobj',ctypes.c_double),
('dgap',ctypes.c_double),
('rdgap',ctypes.c_double),
('mu',ctypes.c_double),
('mu_aff',ctypes.c_double),
('sigma',ctypes.c_double),
('lsit_aff', ctypes.c_int),
('lsit_cc', ctypes.c_int),
('step_aff',ctypes.c_double),
('step_cc',ctypes.c_double),
('solvetime',ctypes.c_double),
('fevalstime',ctypes.c_double)
]

class FILE(ctypes.Structure):
        pass
if sys.version_info.major == 2:
	PyFile_AsFile = ctypes.pythonapi.PyFile_AsFile # problem here with python 3 http://stackoverflow.com/questions/16130268/python-3-replacement-for-pyfile-asfile
	PyFile_AsFile.argtypes = [ctypes.py_object]
	PyFile_AsFile.restype = ctypes.POINTER(FILE)

# determine data types for solver function prototype 
csolver.argtypes = ( ctypes.POINTER(FORCESNLPsolver_params_ctypes), ctypes.POINTER(FORCESNLPsolver_outputs_ctypes), ctypes.POINTER(FORCESNLPsolver_info), ctypes.POINTER(FILE))
csolver.restype = ctypes.c_int

def FORCESNLPsolver_solve(params_arg):
	'''
a Python wrapper for a fast solver generated by FORCES Pro v1.6.104

   OUTPUT = FORCESNLPsolver_py.FORCESNLPsolver_solve(PARAMS) solves a multistage problem
   subject to the parameters supplied in the following dictionary:
       PARAMS['x0'] - column vector of length 5313
       PARAMS['xinit'] - column vector of length 26
       PARAMS['xfinal'] - column vector of length 6
       PARAMS['all_parameters'] - column vector of length 6440

   OUTPUT returns the values of the last iteration of the solver where
       OUTPUT['x001'] - column vector of size 33
       OUTPUT['x002'] - column vector of size 33
       OUTPUT['x003'] - column vector of size 33
       OUTPUT['x004'] - column vector of size 33
       OUTPUT['x005'] - column vector of size 33
       OUTPUT['x006'] - column vector of size 33
       OUTPUT['x007'] - column vector of size 33
       OUTPUT['x008'] - column vector of size 33
       OUTPUT['x009'] - column vector of size 33
       OUTPUT['x010'] - column vector of size 33
       OUTPUT['x011'] - column vector of size 33
       OUTPUT['x012'] - column vector of size 33
       OUTPUT['x013'] - column vector of size 33
       OUTPUT['x014'] - column vector of size 33
       OUTPUT['x015'] - column vector of size 33
       OUTPUT['x016'] - column vector of size 33
       OUTPUT['x017'] - column vector of size 33
       OUTPUT['x018'] - column vector of size 33
       OUTPUT['x019'] - column vector of size 33
       OUTPUT['x020'] - column vector of size 33
       OUTPUT['x021'] - column vector of size 33
       OUTPUT['x022'] - column vector of size 33
       OUTPUT['x023'] - column vector of size 33
       OUTPUT['x024'] - column vector of size 33
       OUTPUT['x025'] - column vector of size 33
       OUTPUT['x026'] - column vector of size 33
       OUTPUT['x027'] - column vector of size 33
       OUTPUT['x028'] - column vector of size 33
       OUTPUT['x029'] - column vector of size 33
       OUTPUT['x030'] - column vector of size 33
       OUTPUT['x031'] - column vector of size 33
       OUTPUT['x032'] - column vector of size 33
       OUTPUT['x033'] - column vector of size 33
       OUTPUT['x034'] - column vector of size 33
       OUTPUT['x035'] - column vector of size 33
       OUTPUT['x036'] - column vector of size 33
       OUTPUT['x037'] - column vector of size 33
       OUTPUT['x038'] - column vector of size 33
       OUTPUT['x039'] - column vector of size 33
       OUTPUT['x040'] - column vector of size 33
       OUTPUT['x041'] - column vector of size 33
       OUTPUT['x042'] - column vector of size 33
       OUTPUT['x043'] - column vector of size 33
       OUTPUT['x044'] - column vector of size 33
       OUTPUT['x045'] - column vector of size 33
       OUTPUT['x046'] - column vector of size 33
       OUTPUT['x047'] - column vector of size 33
       OUTPUT['x048'] - column vector of size 33
       OUTPUT['x049'] - column vector of size 33
       OUTPUT['x050'] - column vector of size 33
       OUTPUT['x051'] - column vector of size 33
       OUTPUT['x052'] - column vector of size 33
       OUTPUT['x053'] - column vector of size 33
       OUTPUT['x054'] - column vector of size 33
       OUTPUT['x055'] - column vector of size 33
       OUTPUT['x056'] - column vector of size 33
       OUTPUT['x057'] - column vector of size 33
       OUTPUT['x058'] - column vector of size 33
       OUTPUT['x059'] - column vector of size 33
       OUTPUT['x060'] - column vector of size 33
       OUTPUT['x061'] - column vector of size 33
       OUTPUT['x062'] - column vector of size 33
       OUTPUT['x063'] - column vector of size 33
       OUTPUT['x064'] - column vector of size 33
       OUTPUT['x065'] - column vector of size 33
       OUTPUT['x066'] - column vector of size 33
       OUTPUT['x067'] - column vector of size 33
       OUTPUT['x068'] - column vector of size 33
       OUTPUT['x069'] - column vector of size 33
       OUTPUT['x070'] - column vector of size 33
       OUTPUT['x071'] - column vector of size 33
       OUTPUT['x072'] - column vector of size 33
       OUTPUT['x073'] - column vector of size 33
       OUTPUT['x074'] - column vector of size 33
       OUTPUT['x075'] - column vector of size 33
       OUTPUT['x076'] - column vector of size 33
       OUTPUT['x077'] - column vector of size 33
       OUTPUT['x078'] - column vector of size 33
       OUTPUT['x079'] - column vector of size 33
       OUTPUT['x080'] - column vector of size 33
       OUTPUT['x081'] - column vector of size 33
       OUTPUT['x082'] - column vector of size 33
       OUTPUT['x083'] - column vector of size 33
       OUTPUT['x084'] - column vector of size 33
       OUTPUT['x085'] - column vector of size 33
       OUTPUT['x086'] - column vector of size 33
       OUTPUT['x087'] - column vector of size 33
       OUTPUT['x088'] - column vector of size 33
       OUTPUT['x089'] - column vector of size 33
       OUTPUT['x090'] - column vector of size 33
       OUTPUT['x091'] - column vector of size 33
       OUTPUT['x092'] - column vector of size 33
       OUTPUT['x093'] - column vector of size 33
       OUTPUT['x094'] - column vector of size 33
       OUTPUT['x095'] - column vector of size 33
       OUTPUT['x096'] - column vector of size 33
       OUTPUT['x097'] - column vector of size 33
       OUTPUT['x098'] - column vector of size 33
       OUTPUT['x099'] - column vector of size 33
       OUTPUT['x100'] - column vector of size 33
       OUTPUT['x101'] - column vector of size 33
       OUTPUT['x102'] - column vector of size 33
       OUTPUT['x103'] - column vector of size 33
       OUTPUT['x104'] - column vector of size 33
       OUTPUT['x105'] - column vector of size 33
       OUTPUT['x106'] - column vector of size 33
       OUTPUT['x107'] - column vector of size 33
       OUTPUT['x108'] - column vector of size 33
       OUTPUT['x109'] - column vector of size 33
       OUTPUT['x110'] - column vector of size 33
       OUTPUT['x111'] - column vector of size 33
       OUTPUT['x112'] - column vector of size 33
       OUTPUT['x113'] - column vector of size 33
       OUTPUT['x114'] - column vector of size 33
       OUTPUT['x115'] - column vector of size 33
       OUTPUT['x116'] - column vector of size 33
       OUTPUT['x117'] - column vector of size 33
       OUTPUT['x118'] - column vector of size 33
       OUTPUT['x119'] - column vector of size 33
       OUTPUT['x120'] - column vector of size 33
       OUTPUT['x121'] - column vector of size 33
       OUTPUT['x122'] - column vector of size 33
       OUTPUT['x123'] - column vector of size 33
       OUTPUT['x124'] - column vector of size 33
       OUTPUT['x125'] - column vector of size 33
       OUTPUT['x126'] - column vector of size 33
       OUTPUT['x127'] - column vector of size 33
       OUTPUT['x128'] - column vector of size 33
       OUTPUT['x129'] - column vector of size 33
       OUTPUT['x130'] - column vector of size 33
       OUTPUT['x131'] - column vector of size 33
       OUTPUT['x132'] - column vector of size 33
       OUTPUT['x133'] - column vector of size 33
       OUTPUT['x134'] - column vector of size 33
       OUTPUT['x135'] - column vector of size 33
       OUTPUT['x136'] - column vector of size 33
       OUTPUT['x137'] - column vector of size 33
       OUTPUT['x138'] - column vector of size 33
       OUTPUT['x139'] - column vector of size 33
       OUTPUT['x140'] - column vector of size 33
       OUTPUT['x141'] - column vector of size 33
       OUTPUT['x142'] - column vector of size 33
       OUTPUT['x143'] - column vector of size 33
       OUTPUT['x144'] - column vector of size 33
       OUTPUT['x145'] - column vector of size 33
       OUTPUT['x146'] - column vector of size 33
       OUTPUT['x147'] - column vector of size 33
       OUTPUT['x148'] - column vector of size 33
       OUTPUT['x149'] - column vector of size 33
       OUTPUT['x150'] - column vector of size 33
       OUTPUT['x151'] - column vector of size 33
       OUTPUT['x152'] - column vector of size 33
       OUTPUT['x153'] - column vector of size 33
       OUTPUT['x154'] - column vector of size 33
       OUTPUT['x155'] - column vector of size 33
       OUTPUT['x156'] - column vector of size 33
       OUTPUT['x157'] - column vector of size 33
       OUTPUT['x158'] - column vector of size 33
       OUTPUT['x159'] - column vector of size 33
       OUTPUT['x160'] - column vector of size 33
       OUTPUT['x161'] - column vector of size 33

   [OUTPUT, EXITFLAG] = FORCESNLPsolver_py.FORCESNLPsolver_solve(PARAMS) returns additionally
   the integer EXITFLAG indicating the state of the solution with 
       1 - Optimal solution has been found (subject to desired accuracy)
       2 - (only branch-and-bound) A feasible point has been identified for which the objective value is no more than codeoptions.mip.mipgap*100 per cent worse than the global optimum 
       0 - Timeout - maximum number of iterations reached
      -1 - (only branch-and-bound) Infeasible problem (problems solving the root relaxation to the desired accuracy)
      -2 - (only branch-and-bound) Out of memory - cannot fit branch and bound nodes into pre-allocated memory.
      -6 - NaN or INF occured during evaluation of functions and derivatives. Please check your initial guess.
      -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
     -10 - The convex solver could not proceed due to an internal error
    -100 - License error

   [OUTPUT, EXITFLAG, INFO] = FORCESNLPsolver_py.FORCESNLPsolver_solve(PARAMS) returns 
   additional information about the last iterate:
       INFO.it        - number of iterations that lead to this result
       INFO.it2opt    - number of convex solves
       INFO.res_eq    - max. equality constraint residual
       INFO.res_ineq  - max. inequality constraint residual
       INFO.pobj      - primal objective
       INFO.mu        - duality measure
       INFO.solvetime - Time needed for solve (wall clock time)
       INFO.fevalstime - Time needed for function evaluations (wall clock time)

 See also COPYING

	'''
	global _lib

	# convert parameters
	params_py = FORCESNLPsolver_params_ctypes()
	for par in params_arg:
		try:
			#setattr(params_py, par, npct.as_ctypes(np.reshape(params_arg[par],np.size(params_arg[par]),order='A'))) 
			params_arg[par] = np.require(params_arg[par], dtype=np.float64, requirements='F')
			setattr(params_py, par, npct.as_ctypes(np.reshape(params_arg[par],np.size(params_arg[par]),order='F')))  
		except:
			raise ValueError('Parameter ' + par + ' does not have the appropriate dimensions or data type. Please use numpy arrays for parameters.')
    
	outputs_py = FORCESNLPsolver_outputs_ctypes()
	info_py = FORCESNLPsolver_info()
	if sys.version_info.major == 2:
		if sys.platform.startswith('win'):
			fp = None # if set to none, the solver prints to stdout by default - necessary because we have an access violation otherwise under windows
		else:
			#fp = open('stdout_temp.txt','w')
			fp = sys.stdout
		try:
			PyFile_AsFile.restype = ctypes.POINTER(FILE)
			exitflag = _lib.FORCESNLPsolver_solve( params_py, ctypes.byref(outputs_py), ctypes.byref(info_py), PyFile_AsFile(fp) , _lib.FORCESNLPsolver_casadi2forces )
			#fp = open('stdout_temp.txt','r')
			#print (fp.read())
			#fp.close()
		except:
			#print 'Problem with solver'
			raise
	elif sys.version_info.major == 3:
		if sys.platform.startswith('win'):
			libc = ctypes.cdll.msvcrt
		elif sys.platform.startswith('darwin'):
			libc = ctypes.CDLL('libc.dylib')
		else:
			libc = ctypes.CDLL('libc.so.6')       # Open libc
		cfopen = getattr(libc,'fopen')        # Get its fopen
		cfopen.restype = ctypes.POINTER(FILE) # Yes, fopen gives a file pointer
		cfopen.argtypes = [ctypes.c_char_p, ctypes.c_char_p] # Yes, fopen gives a file pointer 
		fp = cfopen('stdout_temp.txt'.encode('utf-8'),'w'.encode('utf-8'))    # Use that fopen 

		try:
			if sys.platform.startswith('win'):
				exitflag = _lib.FORCESNLPsolver_solve( params_py, ctypes.byref(outputs_py), ctypes.byref(info_py), None , _lib.FORCESNLPsolver_casadi2forces)
			else:
				exitflag = _lib.FORCESNLPsolver_solve( params_py, ctypes.byref(outputs_py), ctypes.byref(info_py), fp , _lib.FORCESNLPsolver_casadi2forces)
			libc.fclose(fp)
			fptemp = open('stdout_temp.txt','r')
			print (fptemp.read())
			fptemp.close()			
		except:
			#print 'Problem with solver'
			raise

	# convert outputs
	for out in FORCESNLPsolver_outputs:
		FORCESNLPsolver_outputs[out] = npct.as_array(getattr(outputs_py,out))

	return FORCESNLPsolver_outputs,int(exitflag),info_py

solve = FORCESNLPsolver_solve


