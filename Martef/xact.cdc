"/// <Controller Product=AB07 Firmware=Default Revision=1.0 />;\n"
"const int NAX=6;\n"
"const int SCOPEDEPTH=1000;\n"
"const float PERIOD=0.05;\n"
"int attr(UINT32,READONLY,PROPREAD) STATE[4] @1;\n"
"int attr(UINT32) ASTATE[NAX] @2;\n"
"int attr(UINT32,READONLY) FSTATE[NAX] @3;\n"
"int attr(UINT32) AFAULT[NAX] @5;\n"
"int attr(UINT32,READONLY) AHEALTH[NAX] @7;\n"
"int attr(UINT32) TICK @10;\n"
"float attr(PROPREAD,PROPWRITE) TIME @11;\n"
"int attr(UINT32,PROPREAD,PROPWRITE) __scope[32064] @40;\n"
"int attr(UINT32,PROPREAD,PROPWRITE) __flkey @70;\n"
"int attr(UINT32,PROPREAD,PROPWRITE) __flsave @71;\n"
"int attr(UINT32,PROPREAD,PROPWRITE) ENABLE[NAX] @99;\n"
"float attr(FLASH,VEL) VEL[NAX] @101;\n"
"float attr(FLASH,ACC) ACC[NAX] @102;\n"
"float attr(FLASH,ACC) DEC[NAX] @103;\n"
"float attr(FLASH,ACC) KDEC[NAX] @104;\n"
"float attr(FLASH,JERK) JERK[NAX] @105;\n"
"float attr(PROPWRITE,POS) RPOS[NAX] @110;\n"
"float attr(PROPWRITE,VEL) RVEL[NAX] @111;\n"
"float attr(READONLY,ACC) RACC[NAX] @112;\n"
"float attr(READONLY,JERK) RJERK[NAX] @113;\n"
"float attr(PROPWRITE,POS) TPOS[NAX] @115;\n"
"float attr(VEL) TVEL[NAX] @116;\n"
"float attr(PROPWRITE,POS1) FPOS1[NAX] @119;\n"
"float attr(PROPWRITE,POS) FPOS[NAX] @120;\n"
"float attr(READONLY,VEL) FVEL[NAX] @121;\n"
"float attr(READONLY,ACC) FACC[NAX] @122;\n"
"float attr(READONLY,PERCENT) FCA[NAX] @123;\n"
"float attr(READONLY,PERCENT) FCB[NAX] @124;\n"
"float attr(READONLY,PERCENT) FCD[NAX] @125;\n"
"float attr(READONLY,PERCENT) FCQ[NAX] @126;\n"
"float attr(READONLY,POS) PE[NAX] @127;\n"
"float attr(READONLY,VEL) VE[NAX] @128;\n"
"float attr(READONLY,VEL) NFVEL[NAX] @129;\n"
"float attr(PROPWRITE,VEL) VIN[NAX] @131;\n"
"float attr(PROPWRITE,PERCENT) CIN[NAX] @132;\n"
"float attr(READONLY,PERCENT) CDOUT[NAX] @133;\n"
"float attr(PROPWRITE,PERCENT) CQOUT[NAX] @134;\n"
"float attr(READONLY,PERCENT) OUTA[NAX] @135;\n"
"float attr(READONLY,PERCENT) OUTB[NAX] @136;\n"
"float attr(READONLY,PERCENT) OUTC[NAX] @137;\n"
"float attr(PROPWRITE) COMA[NAX] @138;\n"
"float attr(VEL) GVEL[NAX] @140;\n"
"float attr(VEL) GACC[NAX] @141;\n"
"float attr(VEL) GJERK[NAX] @142;\n"
"float attr(VEL) GTV[NAX] @143;\n"
"float attr(VEL) GTA[NAX] @144;\n"
"float attr(VEL) GTJ[NAX] @145;\n"
"int attr(PROPWRITE) GAXES[NAX] @150;\n"
"int attr(PROPWRITE) MTYPE[NAX] @151;\n"
"int CTYPE[NAX] @152;\n"
"float attr(PROPWRITE) GFIFO[NAX] @153;\n"
"float attr(FLASH) VELF[NAX] @179;\n"
"float attr(FLASH,POS) ENDIL[NAX] @180;\n"
"float attr(FLASH,POS) PEL[NAX] @181;\n"
"float attr(FLASH,PERCENT) CURL[NAX] @182;\n"
"float attr(FLASH,PERCENT) PWML[NAX] @183;\n"
"float attr(FLASH,POS) NSL[NAX] @184;\n"
"float attr(FLASH,POS) PSL[NAX] @185;\n"
"float attr(FLASH) MTL[NAX] @186;\n"
"float attr(FLASH) OTL[NAX] @187;\n"
"float attr(FLASH) ENR[NAX] @200;\n"
"float attr(FLASH) ENR1[NAX] @202;\n"
"int COMP[NAX] @204;\n"
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
"int attr(PROPWRITE,FLASH) BQMODE[NAX] @221;\n"
"float attr(FLASH) BQP0[NAX] @222;\n"
"float attr(FLASH) BQP1[NAX] @223;\n"
"float attr(FLASH) BQP2[NAX] @224;\n"
"float attr(FLASH) BQP3[NAX] @225;\n"
"float attr(FLASH) BQP4[NAX] @226;\n"
"float attr(PROPWRITE,FLASH) BQA1[NAX] @227;\n"
"float attr(PROPWRITE,FLASH) BQA2[NAX] @228;\n"
"float attr(PROPWRITE,FLASH) BQB0[NAX] @229;\n"
"float attr(PROPWRITE,FLASH) BQB1[NAX] @230;\n"
"float attr(PROPWRITE,FLASH) BQB2[NAX] @231;\n"
"int attr(FLASH) BQ1ENABLE[NAX] @232;\n"
"int attr(PROPWRITE,FLASH) BQ1MODE[NAX] @233;\n"
"float attr(FLASH) BQ1P0[NAX] @234;\n"
"float attr(FLASH) BQ1P1[NAX] @235;\n"
"float attr(FLASH) BQ1P2[NAX] @236;\n"
"float attr(FLASH) BQ1P3[NAX] @237;\n"
"float attr(FLASH) BQ1P4[NAX] @238;\n"
"float attr(PROPWRITE,FLASH) BQ1A1[NAX] @239;\n"
"float attr(PROPWRITE,FLASH) BQ1A2[NAX] @240;\n"
"float attr(PROPWRITE,FLASH) BQ1B0[NAX] @241;\n"
"float attr(PROPWRITE,FLASH) BQ1B1[NAX] @242;\n"
"float attr(PROPWRITE,FLASH) BQ1B2[NAX] @243;\n"
"int attr(FLASH) BQ2ENABLE[NAX] @244;\n"
"int attr(PROPWRITE,FLASH) BQ2MODE[NAX] @245;\n"
"float attr(FLASH) BQ2P0[NAX] @246;\n"
"float attr(FLASH) BQ2P1[NAX] @247;\n"
"float attr(FLASH) BQ2P2[NAX] @248;\n"
"float attr(FLASH) BQ2P3[NAX] @249;\n"
"float attr(FLASH) BQ2P4[NAX] @250;\n"
"float attr(PROPWRITE,FLASH) BQ2A1[NAX] @251;\n"
"float attr(PROPWRITE,FLASH) BQ2A2[NAX] @252;\n"
"float attr(PROPWRITE,FLASH) BQ2B0[NAX] @253;\n"
"float attr(PROPWRITE,FLASH) BQ2B1[NAX] @254;\n"
"float attr(PROPWRITE,FLASH) BQ2B2[NAX] @255;\n"
"int attr(FLASH) BQ3ENABLE[NAX] @256;\n"
"int attr(PROPWRITE,FLASH) BQ3MODE[NAX] @257;\n"
"float attr(FLASH) BQ3P0[NAX] @258;\n"
"float attr(FLASH) BQ3P1[NAX] @259;\n"
"float attr(FLASH) BQ3P2[NAX] @260;\n"
"float attr(FLASH) BQ3P3[NAX] @261;\n"
"float attr(FLASH) BQ3P4[NAX] @262;\n"
"float attr(PROPWRITE,FLASH) BQ3A1[NAX] @263;\n"
"float attr(PROPWRITE,FLASH) BQ3A2[NAX] @264;\n"
"float attr(PROPWRITE,FLASH) BQ3B0[NAX] @265;\n"
"float attr(PROPWRITE,FLASH) BQ3B1[NAX] @266;\n"
"float attr(PROPWRITE,FLASH) BQ3B2[NAX] @267;\n"
"int attr(UINT8,PROPWRITE) ROCIN[NAX] @268;\n"
"int attr(UINT8,PROPWRITE) ROVIN[NAX] @269;\n"
"int attr(UINT8,PROPWRITE) ROTPOS[NAX] @270;\n"
"int attr(UINT8,PROPWRITE) ROTVEL[NAX] @271;\n"
"int attr(UINT8,PROPWRITE) RORPOS[NAX] @272;\n"
"float attr(READONLY,PERCENT) AIN[24] @280;\n"
"int attr(UINT32) AFAULTMASK[NAX] @303;\n"
"int attr(UINT32) AFAULTKILL[NAX] @305;\n"
"int attr(UINT32) AFAULTDISABLE[NAX] @307;\n"
"int attr(UINT8) SGMOD[2] @340;\n"
"float attr(PROPWRITE) SGPRD[2] @341;\n"
"float SGDUT[2] @342;\n"
"float SGMIN[2] @343;\n"
"float SGMAX[2] @344;\n"
"float SGFILT[2] @345;\n"
"float SGN[2] @346;\n"
