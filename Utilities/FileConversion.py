# Based on NURBS15-1

#import math
#import matplotlib.pyplot as plt
#import numpy as np
#import scipy as sp
import os
#import json

def AddNumbersNames(inpc, inpt, outc):
    #os.chdir(os.path.dirname(os.path.realpath(__file__)))
    with open(inpc) as cf, open(inpt) as tf, open(outc, 'w') as of:
        vars = dict()
        tfls = tf.readlines()
        for tfl in tfls:
            tfl = tfl.strip()
            if tfl.startswith('float'):
                tfl = tfl[5:].strip()
            elif tfl.startswith('int'):
                tfl = tfl[3:].strip()
            else:
                continue
            if tfl.startswith('attr'):
                tfl = tfl[tfl.find(')')+1:]
            tfl = tfl.rstrip(';').rstrip()
            pairs = tfl.split(',')
            for pair in pairs:
                t = pair.split('@')
                if len(t) == 2: 
                    t[0] = t[0].strip()
                    if t[0][-1] == ']':
                        t[0] = t[0][:t[0].index('[')]
                    vars[int(t[1])] = t[0]
        cfls = cf.readlines()
        ofls = []
        for i,cfl in enumerate(cfls):
            ofls.append(f"/*{i:4} {vars.get(i, ''):15}*/ {cfl.strip()}\n")
        of.writelines(ofls)

def Sysvartable2Cdf(sysvartable, prefixfrom, cdftext, cdfcpp):
    #os.chdir(os.path.dirname(os.path.realpath(__file__)))
    types = {'TYPE_UINT32' : 'UINT32', 'TYPE_INT16' : 'INT16', 'TYPE_UINT16' : 'UINT16', 'TYPE_INT8' : 'INT8', 'TYPE_UINT8' : 'UINT8'}
    cdfls = []
    with open(prefixfrom) as pf:
        pfls = [p.strip() for p in pf.readlines()]
        cformat = pfls[0] == '"'
        for pfl in pfls:
            pfl = pfl.strip()
            if cformat: pfl = pfl.strip('"').rstrip('\n')
            if pfl.startswith('//'): 
                cdfls.append(pfl+'\n')
            elif pfl.startswith('const'):
                cdfls.append(pfl+'\n')
            else:
                break
    with open(sysvartable) as svf:
        svls = svf.readlines()
        for i,svl in enumerate(svls):
            ce = svl.find('//')
            if ce >= 0: svl = svl[:ce]
            svl = svl.strip().rstrip(',')
            if svl.startswith('/*'):
                var = ''
                ce = svl.find('*/')
                tt = svl[2:ce].split()
                if (len(tt) == 2):
                    ind = int(tt[0].strip())
                    if (ind != i):
                        break;
                    var = tt[1].strip()
                svl = svl[ce+2:].strip()
            if (not svl.startswith('{')) or (not svl.endswith('}')):
                break;
            svl = svl[1:-2]
            tt = svl.split(',')
            if len(tt) < 3: 
                continue
            size = tt[0]
            if size == '0': 
                continue
            iatts = [a.strip() for a in tt[1].split('|')]
            oatts = []
            t = next((a for a in iatts if a.startswith('TYPE_')), None)
            os = 'float' if not t or t == 'TYPE_FLOAT' else 'int'
            if types.get(t, '') != '': oatts.append(types[t])
            if not ('VF_DIRECTWRITE' in iatts) and not ('VF_PROPWRITE' in iatts): oatts.append('READONLY')
            if 'VF_PROPREAD' in iatts: oatts.append('PROPREAD')
            if 'VF_PROPWRITE' in iatts: oatts.append('PROPWRITE')
            if 'VF_FLASH' in iatts: oatts.append('FLASH')
            if len(oatts) > 0:
                os += f" attr({','.join(oatts)})"
            os += f" {var}"
            if size != '1':
                os += f'[{size}]'
            os += f' @{i};\n'
            cdfls.append(os)
    with open(cdftext, 'w') as cdf, open(cdfcpp, 'w') as cdc:
        cdf.writelines(cdfls)
        cdcls = ['"'+s.rstrip("\n")+'\\n"\n' for s in cdfls]
        cdc.writelines(cdcls)


#cf = "C:\\Projects\\XactRobotics\\MartefH7\\MH7-fw\\Martef\\sysvar.inc"
#tf = "C:\\Projects\\XactRobotics\\MartefH7\\MH7-sw\\Martefon\\ControllerDefinitions\\xact.cdf"                    
#of = "C:\\Projects\\XactRobotics\\MartefH7\\MH7-fw\\Martef\\sysvartable1.inc"
#AddNumbersNames(cf, tf, of)

svf = "C:\\Projects\\XactRobotics\\MartefH7\\MH7-fw\\Martef\\sysvar.inc"
pf = "C:\\Projects\\XactRobotics\\MartefH7\\MH7-sw\\Martefon\\ControllerDefinitions\\xact.cdf"                    
cdf = "C:\\Projects\\XactRobotics\\MartefH7\\MH7-fw\\Martef\\xact.cdf"
cdc = "C:\\Projects\\XactRobotics\\MartefH7\\MH7-fw\\Martef\\xact.cdc"
Sysvartable2Cdf(svf, pf, cdf, cdc)