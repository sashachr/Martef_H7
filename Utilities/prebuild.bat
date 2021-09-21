set cur_dir=%CD%
set proj_dir=%1
set "proj_dir=%proj_dir:/=\%"
cd %proj_dir%
del Martef\xact.cdf 
del Martef\xact.cdc
python Utilities\CreateCdf.py Martef\sysvar.inc Martef\xactprefix.cdf Martef\xact.cdf Martef\xact.cdc
del Martef\gitversion.h 
python Utilities\GitVersion.py
cd %cur_dir%
