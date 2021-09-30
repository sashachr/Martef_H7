/// <Controller Product=AB07 Firmware=Default Revision=1.0 />;
const int NAX=6;
const int SCOPEDEPTH=1000;
const float PERIOD=0.05;
int attr(UINT32,READONLY,PROPREAD) STATE[4] @1;
int attr(UINT32,READONLY) ASTATE[NAX] @2;
int attr(UINT32) RSTATE[NAX] @3;
int attr(UINT32) TICK @10;
float attr(PROPREAD,PROPWRITE) TIME @11;
int attr(UINT32,PROPREAD,PROPWRITE) __scope[32064] @40;
float attr(FLASH) VEL[NAX] @101;
float attr(FLASH) ACC[NAX] @102;
float attr(FLASH) DEC[NAX] @103;
float attr(FLASH) KDEC[NAX] @104;
float attr(FLASH) JERK[NAX] @105;
float attr(PROPWRITE) RPOS[NAX] @110;
float attr(PROPWRITE) RVEL[NAX] @111;
float attr(READONLY) RACC[NAX] @112;
float attr(READONLY) RJERK[NAX] @113;
float TPOS[NAX] @115;
float TVEL[NAX] @116;
float attr(PROPWRITE) FPOS[NAX] @120;
float attr(READONLY) FVEL[NAX] @121;
float attr(READONLY) FACC[NAX] @122;
float attr(READONLY) FCA[NAX] @123;
float attr(READONLY) FCB[NAX] @124;
float attr(READONLY) FCD[NAX] @125;
float attr(READONLY) FCQ[NAX] @126;
float attr(READONLY) PE[NAX] @127;
float attr(PROPWRITE) VIN[NAX] @131;
float attr(PROPWRITE) CIN[NAX] @132;
float attr(READONLY) CDOUT[NAX] @133;
float attr(PROPWRITE) CQOUT[NAX] @134;
float attr(PROPWRITE) COMA[NAX] @138;
float attr(FLASH) ENDIL[NAX] @180;
float attr(FLASH) PEL[NAX] @181;
float attr(FLASH) CURL[NAX] @182;
float attr(FLASH) PWML[NAX] @183;
float attr(FLASH) NSL[NAX] @184;
float attr(FLASH) PSL[NAX] @185;
float attr(FLASH) MTL[NAX] @186;
float attr(FLASH) OTL[NAX] @187;
float attr(FLASH) ENR[NAX] @200;
float EOFF[NAX] @201;
float attr(FLASH) ENR1[NAX] @202;
float EOFF1[NAX] @203;
float attr(FLASH) PKP[NAX] @206;
float attr(FLASH) PKI[NAX] @207;
float attr(FLASH) PLI[NAX] @208;
float attr(FLASH) VKP[NAX] @209;
float attr(FLASH) VKI[NAX] @210;
float attr(FLASH) VLI[NAX] @211;
float attr(FLASH) CKP[NAX] @212;
float attr(FLASH) CKI[NAX] @213;
float attr(FLASH) CLI[NAX] @214;
int attr(FLASH) BQENABLE[NAX] @220;
int attr(FLASH) BQMODE[NAX] @221;
float attr(FLASH) BQA1[NAX] @222;
float attr(FLASH) BQA2[NAX] @223;
float attr(FLASH) BQB0[NAX] @224;
float attr(FLASH) BQB1[NAX] @225;
float attr(FLASH) BQB2[NAX] @226;
int attr(FLASH) BQ1AENABLE[NAX] @227;
int attr(FLASH) BQ1AMODE[NAX] @228;
float attr(FLASH) BQ1AA1[NAX] @229;
float attr(FLASH) BQ1AA2[NAX] @230;
float attr(FLASH) BQ1AB0[NAX] @231;
float attr(FLASH) BQ1AB1[NAX] @232;
float attr(FLASH) BQ1AB2[NAX] @233;
int attr(FLASH) BQ2AENABLE[NAX] @234;
int attr(FLASH) BQ2AMODE[NAX] @235;
float attr(FLASH) BQ2AA1[NAX] @236;
float attr(FLASH) BQ2AA2[NAX] @237;
float attr(FLASH) BQ2AB0[NAX] @238;
float attr(FLASH) BQ2AB1[NAX] @239;
float attr(FLASH) BQ2AB2[NAX] @240;
int attr(FLASH) BQ3AENABLE[NAX] @241;
int attr(FLASH) BQ3AMODE[NAX] @242;
float attr(FLASH) BQ3AA1[NAX] @243;
float attr(FLASH) BQ3AA2[NAX] @244;
float attr(FLASH) BQ3AB0[NAX] @245;
float attr(FLASH) BQ3AB1[NAX] @246;
float attr(FLASH) BQ3AB2[NAX] @247;
int attr(UINT8,PROPWRITE) ROTPOS[NAX] @270;
int attr(UINT8,PROPWRITE) ROTVEL[NAX] @271;
int attr(UINT8,PROPWRITE) RORPOS[NAX] @272;
int attr(UINT8,PROPWRITE) RORVEL[NAX] @273;
float attr(READONLY) AIN[24] @280;
int attr(UINT8) SGMOD[2] @340;
float attr(PROPWRITE) SGPRD[2] @341;
float SGDUT[2] @342;
float SGMIN[2] @343;
float SGMAX[2] @344;
float SGN[2] @345;
