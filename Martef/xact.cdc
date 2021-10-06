"/// <Controller Product=AB07 Firmware=Default Revision=1.0 />;\n"
"const int NAX=6;\n"
"const int SCOPEDEPTH=1000;\n"
"const float PERIOD=0.05;\n"
"int attr(UINT32,READONLY,PROPREAD) STATE[4] @1;\n"
"int attr(UINT32) ASTATE[NAX] @2;\n"
"int attr(UINT32,READONLY) FSTATE[NAX] @3;\n"
"int attr(UINT32,READONLY) AFAULT[NAX] @5;\n"
"int attr(UINT32) TICK @10;\n"
"float attr(PROPREAD,PROPWRITE) TIME @11;\n"
"int attr(UINT32,PROPREAD,PROPWRITE) __scope[32064] @40;\n"
"float attr(FLASH) VEL[NAX] @101;\n"
"float attr(FLASH) ACC[NAX] @102;\n"
"float attr(FLASH) DEC[NAX] @103;\n"
"float attr(FLASH) KDEC[NAX] @104;\n"
"float attr(FLASH) JERK[NAX] @105;\n"
"float attr(PROPWRITE) RPOS[NAX] @110;\n"
"float attr(PROPWRITE) RVEL[NAX] @111;\n"
"float attr(READONLY) RACC[NAX] @112;\n"
"float attr(READONLY) RJERK[NAX] @113;\n"
"float TPOS[NAX] @115;\n"
"float TVEL[NAX] @116;\n"
"float attr(PROPWRITE) FPOS[NAX] @120;\n"
"float attr(READONLY) FVEL[NAX] @121;\n"
"float attr(READONLY) FACC[NAX] @122;\n"
"float attr(READONLY) FCA[NAX] @123;\n"
"float attr(READONLY) FCB[NAX] @124;\n"
"float attr(READONLY) FCD[NAX] @125;\n"
"float attr(READONLY) FCQ[NAX] @126;\n"
"float attr(READONLY) PE[NAX] @127;\n"
"float attr(PROPWRITE) VIN[NAX] @131;\n"
"float attr(PROPWRITE) CIN[NAX] @132;\n"
"float attr(READONLY) CDOUT[NAX] @133;\n"
"float attr(PROPWRITE) CQOUT[NAX] @134;\n"
"float attr(PROPWRITE) COMA[NAX] @138;\n"
"float attr(FLASH) ENDIL[NAX] @180;\n"
"float attr(FLASH) PEL[NAX] @181;\n"
"float attr(FLASH) CURL[NAX] @182;\n"
"float attr(FLASH) PWML[NAX] @183;\n"
"float attr(FLASH) NSL[NAX] @184;\n"
"float attr(FLASH) PSL[NAX] @185;\n"
"float attr(FLASH) MTL[NAX] @186;\n"
"float attr(FLASH) OTL[NAX] @187;\n"
"float attr(FLASH) ENR[NAX] @200;\n"
"float EOFF[NAX] @201;\n"
"float attr(FLASH) ENR1[NAX] @202;\n"
"float EOFF1[NAX] @203;\n"
"float attr(FLASH) PKP[NAX] @206;\n"
"float attr(FLASH) PKI[NAX] @207;\n"
"float attr(FLASH) PLI[NAX] @208;\n"
"float attr(FLASH) VKP[NAX] @209;\n"
"float attr(FLASH) VKI[NAX] @210;\n"
"float attr(FLASH) VLI[NAX] @211;\n"
"float attr(FLASH) CKP[NAX] @212;\n"
"float attr(FLASH) CKI[NAX] @213;\n"
"float attr(FLASH) CLI[NAX] @214;\n"
"int attr(FLASH) BQENABLE[NAX] @220;\n"
"int attr(FLASH) BQMODE[NAX] @221;\n"
"float attr(FLASH) BQA1[NAX] @222;\n"
"float attr(FLASH) BQA2[NAX] @223;\n"
"float attr(FLASH) BQB0[NAX] @224;\n"
"float attr(FLASH) BQB1[NAX] @225;\n"
"float attr(FLASH) BQB2[NAX] @226;\n"
"int attr(FLASH) BQ1AENABLE[NAX] @227;\n"
"int attr(FLASH) BQ1AMODE[NAX] @228;\n"
"float attr(FLASH) BQ1AA1[NAX] @229;\n"
"float attr(FLASH) BQ1AA2[NAX] @230;\n"
"float attr(FLASH) BQ1AB0[NAX] @231;\n"
"float attr(FLASH) BQ1AB1[NAX] @232;\n"
"float attr(FLASH) BQ1AB2[NAX] @233;\n"
"int attr(FLASH) BQ2AENABLE[NAX] @234;\n"
"int attr(FLASH) BQ2AMODE[NAX] @235;\n"
"float attr(FLASH) BQ2AA1[NAX] @236;\n"
"float attr(FLASH) BQ2AA2[NAX] @237;\n"
"float attr(FLASH) BQ2AB0[NAX] @238;\n"
"float attr(FLASH) BQ2AB1[NAX] @239;\n"
"float attr(FLASH) BQ2AB2[NAX] @240;\n"
"int attr(FLASH) BQ3AENABLE[NAX] @241;\n"
"int attr(FLASH) BQ3AMODE[NAX] @242;\n"
"float attr(FLASH) BQ3AA1[NAX] @243;\n"
"float attr(FLASH) BQ3AA2[NAX] @244;\n"
"float attr(FLASH) BQ3AB0[NAX] @245;\n"
"float attr(FLASH) BQ3AB1[NAX] @246;\n"
"float attr(FLASH) BQ3AB2[NAX] @247;\n"
"int attr(UINT32) AFAULTMASK[NAX] @251;\n"
"int attr(UINT32) AFAULTKILL[NAX] @253;\n"
"int attr(UINT32) AFAULTDISABLE[NAX] @255;\n"
"int attr(UINT8,PROPWRITE) ROCIN[NAX] @268;\n"
"int attr(UINT8,PROPWRITE) ROTPOS[NAX] @270;\n"
"int attr(UINT8,PROPWRITE) ROTVEL[NAX] @271;\n"
"int attr(UINT8,PROPWRITE) RORPOS[NAX] @272;\n"
"int attr(UINT8,PROPWRITE) RORVEL[NAX] @273;\n"
"float attr(READONLY) AIN[24] @280;\n"
"int attr(UINT8) SGMOD[2] @340;\n"
"float attr(PROPWRITE) SGPRD[2] @341;\n"
"float SGDUT[2] @342;\n"
"float SGMIN[2] @343;\n"
"float SGMAX[2] @344;\n"
"float SGN[2] @345;\n"
