"/// <Controller Product=AB07 Firmware=Default Revision=1.0 />;\n"
"const int NAX=6;\n"
"const int SCOPEDEPTH=1000;\n"
"const float PERIOD=0.05;\n"
"int attr(UINT32,READONLY,PROPREAD) __status[4] @1;\n"
"int attr(UINT32,READONLY,PROPREAD) __astatus[NAX] @2;\n"
"int attr(UINT32,PROPREAD,PROPWRITE) _scope[32064] @40;\n"
"float attr(PROPWRITE) RPOS[NAX] @110;\n"
"float attr(PROPWRITE) RVEL[NAX] @111;\n"
"float attr(READONLY) RACC[NAX] @112;\n"
"float attr(READONLY) RJERK[NAX] @113;\n"
"float attr(READONLY) RCUR[NAX] @114;\n"
"float attr(READONLY) TPOS[NAX] @115;\n"
"float attr(READONLY) TVEL[NAX] @116;\n"
"float attr(PROPWRITE) FPOS[NAX] @120;\n"
"float attr(READONLY) FVEL[NAX] @121;\n"
"float attr(READONLY) FACC[NAX] @122;\n"
"float attr(READONLY) FCUR[NAX] @123;\n"
"float attr(READONLY) FCUR1[NAX] @124;\n"
"float attr(READONLY) PE[NAX] @125;\n"
"int attr(UINT8) SGMOD[2] @340;\n"
"float attr(PROPWRITE) SGPRD[2] @341;\n"
"float SGDUT[2] @342;\n"
"float SGMIN[2] @343;\n"
"float SGMAX[2] @344;\n"
"float SGN[2] @345;\n"