/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:20:14 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "dR_LeftAnkleJoint_src.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }
//INLINE double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(double *p_output1,const double *var1,const double *var2)
{
  double t468;
  double t292;
  double t482;
  double t354;
  double t711;
  double t166;
  double t267;
  double t375;
  double t781;
  double t785;
  double t811;
  double t888;
  double t903;
  double t973;
  double t974;
  double t992;
  double t1056;
  double t1088;
  double t1117;
  double t1168;
  double t1189;
  double t1212;
  double t1338;
  double t89;
  double t159;
  double t1372;
  double t1715;
  double t1264;
  double t1401;
  double t1417;
  double t63;
  double t1760;
  double t1839;
  double t1872;
  double t5;
  double t2295;
  double t2304;
  double t2408;
  double t2629;
  double t2694;
  double t2707;
  double t2285;
  double t2409;
  double t2523;
  double t2607;
  double t2750;
  double t2757;
  double t2769;
  double t2826;
  double t2832;
  double t2084;
  double t2764;
  double t2840;
  double t2887;
  double t3034;
  double t3143;
  double t3154;
  double t3772;
  double t3803;
  double t3591;
  double t3613;
  double t3629;
  double t3818;
  double t3844;
  double t3920;
  double t3938;
  double t3970;
  double t3900;
  double t3990;
  double t3998;
  double t4007;
  double t4009;
  double t4036;
  double t4509;
  double t4551;
  double t4557;
  double t4263;
  double t4292;
  double t4366;
  double t4394;
  double t4397;
  double t4398;
  double t4503;
  double t4562;
  double t4581;
  double t4613;
  double t4616;
  double t4635;
  double t4604;
  double t4802;
  double t4828;
  double t4845;
  double t4943;
  double t4962;
  double t5174;
  double t5176;
  double t5177;
  double t5056;
  double t5076;
  double t5077;
  double t5270;
  double t5309;
  double t5321;
  double t5047;
  double t5161;
  double t5219;
  double t5220;
  double t5252;
  double t5264;
  double t5269;
  double t5331;
  double t5335;
  double t5362;
  double t5377;
  double t5378;
  double t5360;
  double t5379;
  double t5443;
  double t5471;
  double t5486;
  double t5493;
  double t5671;
  double t5676;
  double t5689;
  double t5696;
  double t5697;
  double t5704;
  double t5712;
  double t5734;
  double t5739;
  double t5707;
  double t5742;
  double t5761;
  double t5764;
  double t5766;
  double t5769;
  double t5848;
  double t5917;
  double t5918;
  double t5970;
  double t5976;
  double t5987;
  double t6004;
  double t5986;
  double t6021;
  double t6083;
  double t6126;
  double t6159;
  double t6169;
  double t6176;
  double t6177;
  double t6198;
  double t6200;
  double t6202;
  double t6203;
  double t6206;
  double t6209;
  double t6211;
  double t6214;
  double t6217;
  double t6219;
  double t6227;
  double t6229;
  double t6233;
  double t6237;
  double t6238;
  double t6244;
  double t6246;
  double t6247;
  double t6279;
  double t6280;
  double t6270;
  double t6276;
  double t6277;
  double t6278;
  double t6284;
  double t6286;
  double t6293;
  double t6296;
  double t6297;
  double t6292;
  double t6298;
  double t6302;
  double t6309;
  double t6311;
  double t6313;
  double t6351;
  double t6353;
  double t6343;
  double t6349;
  double t6350;
  double t6355;
  double t6358;
  double t6368;
  double t6374;
  double t6381;
  double t6364;
  double t6382;
  double t6383;
  double t6386;
  double t6387;
  double t6391;
  double t6432;
  double t6435;
  double t6444;
  double t6415;
  double t6416;
  double t6417;
  double t6420;
  double t6422;
  double t6424;
  double t6425;
  double t6451;
  double t6454;
  double t6464;
  double t6465;
  double t6467;
  double t6459;
  double t6469;
  double t6472;
  double t6477;
  double t6479;
  double t6480;
  double t6505;
  double t6510;
  double t6512;
  double t6519;
  double t6520;
  double t6521;
  double t6526;
  double t6527;
  double t6536;
  double t6523;
  double t6539;
  double t6541;
  double t6543;
  double t6544;
  double t6546;
  double t6563;
  double t6564;
  double t6566;
  double t6571;
  double t6573;
  double t6575;
  double t6580;
  double t6574;
  double t6586;
  double t6606;
  double t6607;
  double t6610;
  double t6618;
  double t6619;
  double t6620;
  double t6183;
  double t6184;
  double t6189;
  double t6651;
  double t6682;
  double t6683;
  double t6684;
  double t6688;
  double t6689;
  double t6691;
  double t6695;
  double t6698;
  double t6700;
  double t6701;
  double t6703;
  double t6739;
  double t6742;
  double t6745;
  double t6730;
  double t6733;
  double t6736;
  double t6738;
  double t6747;
  double t6751;
  double t6754;
  double t6757;
  double t6759;
  double t6753;
  double t6761;
  double t6763;
  double t6767;
  double t6768;
  double t6769;
  double t6766;
  double t6771;
  double t6773;
  double t6774;
  double t6776;
  double t6777;
  double t6779;
  double t6783;
  double t6785;
  double t6793;
  double t6797;
  double t6798;
  double t6799;
  double t6804;
  double t6806;
  double t6808;
  double t6810;
  double t6812;
  double t6807;
  double t6813;
  double t6817;
  double t6825;
  double t6827;
  double t6829;
  double t6862;
  double t6863;
  double t6864;
  double t6866;
  double t6867;
  double t6869;
  double t6883;
  double t6868;
  double t6890;
  double t6907;
  double t6908;
  double t6909;
  double t6918;
  double t6920;
  double t6923;
  double t6966;
  double t6967;
  double t6969;
  double t6950;
  double t6951;
  double t6953;
  double t6958;
  double t6959;
  double t6961;
  double t6963;
  double t6970;
  double t6972;
  double t6981;
  double t6982;
  double t6985;
  double t6978;
  double t6991;
  double t6998;
  double t7003;
  double t7004;
  double t7011;
  double t1589;
  double t1917;
  double t1969;
  double t2103;
  double t2173;
  double t2195;
  double t2974;
  double t3158;
  double t3257;
  double t3340;
  double t3419;
  double t3422;
  double t3999;
  double t4037;
  double t4043;
  double t4081;
  double t4184;
  double t4191;
  double t4840;
  double t4963;
  double t4967;
  double t4990;
  double t5005;
  double t5015;
  double t5459;
  double t5504;
  double t5509;
  double t5595;
  double t5613;
  double t5619;
  double t5763;
  double t5770;
  double t5773;
  double t5776;
  double t5787;
  double t5795;
  double t6006;
  double t6018;
  double t6030;
  double t6039;
  double t6160;
  double t6164;
  double t7101;
  double t6241;
  double t6250;
  double t6253;
  double t6257;
  double t6260;
  double t6261;
  double t6303;
  double t6314;
  double t6315;
  double t6320;
  double t6322;
  double t6326;
  double t6385;
  double t6394;
  double t6401;
  double t6403;
  double t6404;
  double t6405;
  double t6475;
  double t6483;
  double t6484;
  double t6489;
  double t6493;
  double t6495;
  double t6542;
  double t6547;
  double t6549;
  double t6553;
  double t6555;
  double t6556;
  double t6582;
  double t6584;
  double t6594;
  double t6597;
  double t6612;
  double t6614;
  double t6624;
  double t6628;
  double t6631;
  double t7178;
  double t6190;
  double t6192;
  double t6699;
  double t6708;
  double t6713;
  double t6721;
  double t6723;
  double t6724;
  double t7208;
  double t7213;
  double t7215;
  double t6818;
  double t6837;
  double t6838;
  double t6841;
  double t6845;
  double t6849;
  double t6884;
  double t6886;
  double t6892;
  double t6893;
  double t6910;
  double t6915;
  double t6931;
  double t6937;
  double t6939;
  double t7242;
  double t7000;
  double t7012;
  double t7016;
  double t7019;
  double t7020;
  double t7023;
  t468 = Cos(var1[3]);
  t292 = Cos(var1[5]);
  t482 = Sin(var1[4]);
  t354 = Sin(var1[3]);
  t711 = Sin(var1[5]);
  t166 = Cos(var1[7]);
  t267 = Cos(var1[6]);
  t375 = -1.*t292*t354;
  t781 = t468*t482*t711;
  t785 = t375 + t781;
  t811 = t267*t785;
  t888 = t468*t292*t482;
  t903 = t354*t711;
  t973 = t888 + t903;
  t974 = Sin(var1[6]);
  t992 = t973*t974;
  t1056 = t811 + t992;
  t1088 = t166*t1056;
  t1117 = Cos(var1[4]);
  t1168 = Sin(var1[7]);
  t1189 = -1.*t468*t1117*t1168;
  t1212 = t1088 + t1189;
  t1338 = Cos(var1[9]);
  t89 = Cos(var1[8]);
  t159 = Sin(var1[9]);
  t1372 = Sin(var1[8]);
  t1715 = Cos(var1[10]);
  t1264 = -1.*t89*t159*t1212;
  t1401 = -1.*t1338*t1212*t1372;
  t1417 = t1264 + t1401;
  t63 = Sin(var1[10]);
  t1760 = t1338*t89*t1212;
  t1839 = -1.*t159*t1212*t1372;
  t1872 = t1760 + t1839;
  t5 = Sin(var1[11]);
  t2295 = t292*t354;
  t2304 = -1.*t468*t482*t711;
  t2408 = t2295 + t2304;
  t2629 = t267*t2408;
  t2694 = -1.*t973*t974;
  t2707 = t2629 + t2694;
  t2285 = t267*t973;
  t2409 = t2408*t974;
  t2523 = t2285 + t2409;
  t2607 = t89*t2523*t1168;
  t2750 = t2707*t1372;
  t2757 = t2607 + t2750;
  t2769 = t89*t2707;
  t2826 = -1.*t2523*t1168*t1372;
  t2832 = t2769 + t2826;
  t2084 = Cos(var1[11]);
  t2764 = -1.*t159*t2757;
  t2840 = t1338*t2832;
  t2887 = t2764 + t2840;
  t3034 = t1338*t2757;
  t3143 = t159*t2832;
  t3154 = t3034 + t3143;
  t3772 = -1.*t267*t785;
  t3803 = t3772 + t2694;
  t3591 = -1.*t785*t974;
  t3613 = t2285 + t3591;
  t3629 = t89*t3613*t1168;
  t3818 = t3803*t1372;
  t3844 = t3629 + t3818;
  t3920 = t89*t3803;
  t3938 = -1.*t3613*t1168*t1372;
  t3970 = t3920 + t3938;
  t3900 = -1.*t159*t3844;
  t3990 = t1338*t3970;
  t3998 = t3900 + t3990;
  t4007 = t1338*t3844;
  t4009 = t159*t3970;
  t4036 = t4007 + t4009;
  t4509 = t468*t1117*t292*t267;
  t4551 = -1.*t468*t1117*t711*t974;
  t4557 = t4509 + t4551;
  t4263 = -1.*t468*t166*t482;
  t4292 = t468*t1117*t267*t711;
  t4366 = t468*t1117*t292*t974;
  t4394 = t4292 + t4366;
  t4397 = t4394*t1168;
  t4398 = t4263 + t4397;
  t4503 = t89*t4398;
  t4562 = t4557*t1372;
  t4581 = t4503 + t4562;
  t4613 = t89*t4557;
  t4616 = -1.*t4398*t1372;
  t4635 = t4613 + t4616;
  t4604 = -1.*t159*t4581;
  t4802 = t1338*t4635;
  t4828 = t4604 + t4802;
  t4845 = t1338*t4581;
  t4943 = t159*t4635;
  t4962 = t4845 + t4943;
  t5174 = -1.*t292*t354*t482;
  t5176 = t468*t711;
  t5177 = t5174 + t5176;
  t5056 = -1.*t468*t292;
  t5076 = -1.*t354*t482*t711;
  t5077 = t5056 + t5076;
  t5270 = t267*t5177;
  t5309 = -1.*t5077*t974;
  t5321 = t5270 + t5309;
  t5047 = -1.*t1117*t166*t354;
  t5161 = t267*t5077;
  t5219 = t5177*t974;
  t5220 = t5161 + t5219;
  t5252 = t5220*t1168;
  t5264 = t5047 + t5252;
  t5269 = t89*t5264;
  t5331 = t5321*t1372;
  t5335 = t5269 + t5331;
  t5362 = t89*t5321;
  t5377 = -1.*t5264*t1372;
  t5378 = t5362 + t5377;
  t5360 = -1.*t159*t5335;
  t5379 = t1338*t5378;
  t5443 = t5360 + t5379;
  t5471 = t1338*t5335;
  t5486 = t159*t5378;
  t5493 = t5471 + t5486;
  t5671 = t468*t1117*t166;
  t5676 = t1056*t1168;
  t5689 = t5671 + t5676;
  t5696 = -1.*t89*t5689;
  t5697 = -1.*t3613*t1372;
  t5704 = t5696 + t5697;
  t5712 = t89*t3613;
  t5734 = -1.*t5689*t1372;
  t5739 = t5712 + t5734;
  t5707 = t159*t5704;
  t5742 = t1338*t5739;
  t5761 = t5707 + t5742;
  t5764 = t1338*t5704;
  t5766 = -1.*t159*t5739;
  t5769 = t5764 + t5766;
  t5848 = t89*t5689;
  t5917 = t3613*t1372;
  t5918 = t5848 + t5917;
  t5970 = -1.*t159*t5918;
  t5976 = t5970 + t5742;
  t5987 = -1.*t1338*t5918;
  t6004 = t5987 + t5766;
  t5986 = -1.*t63*t5976;
  t6021 = t1715*t5976;
  t6083 = t1338*t5918;
  t6126 = t159*t5739;
  t6159 = t6083 + t6126;
  t6169 = -1.*t63*t6159;
  t6176 = t6021 + t6169;
  t6177 = t5*t6176;
  t6198 = t468*t292;
  t6200 = t354*t482*t711;
  t6202 = t6198 + t6200;
  t6203 = t267*t6202;
  t6206 = t292*t354*t482;
  t6209 = -1.*t468*t711;
  t6211 = t6206 + t6209;
  t6214 = t6211*t974;
  t6217 = t6203 + t6214;
  t6219 = t166*t6217;
  t6227 = -1.*t1117*t354*t1168;
  t6229 = t6219 + t6227;
  t6233 = -1.*t89*t159*t6229;
  t6237 = -1.*t1338*t6229*t1372;
  t6238 = t6233 + t6237;
  t6244 = t1338*t89*t6229;
  t6246 = -1.*t159*t6229*t1372;
  t6247 = t6244 + t6246;
  t6279 = -1.*t6211*t974;
  t6280 = t5161 + t6279;
  t6270 = t267*t6211;
  t6276 = t5077*t974;
  t6277 = t6270 + t6276;
  t6278 = t89*t6277*t1168;
  t6284 = t6280*t1372;
  t6286 = t6278 + t6284;
  t6293 = t89*t6280;
  t6296 = -1.*t6277*t1168*t1372;
  t6297 = t6293 + t6296;
  t6292 = -1.*t159*t6286;
  t6298 = t1338*t6297;
  t6302 = t6292 + t6298;
  t6309 = t1338*t6286;
  t6311 = t159*t6297;
  t6313 = t6309 + t6311;
  t6351 = -1.*t267*t6202;
  t6353 = t6351 + t6279;
  t6343 = -1.*t6202*t974;
  t6349 = t6270 + t6343;
  t6350 = t89*t6349*t1168;
  t6355 = t6353*t1372;
  t6358 = t6350 + t6355;
  t6368 = t89*t6353;
  t6374 = -1.*t6349*t1168*t1372;
  t6381 = t6368 + t6374;
  t6364 = -1.*t159*t6358;
  t6382 = t1338*t6381;
  t6383 = t6364 + t6382;
  t6386 = t1338*t6358;
  t6387 = t159*t6381;
  t6391 = t6386 + t6387;
  t6432 = t1117*t292*t267*t354;
  t6435 = -1.*t1117*t354*t711*t974;
  t6444 = t6432 + t6435;
  t6415 = -1.*t166*t354*t482;
  t6416 = t1117*t267*t354*t711;
  t6417 = t1117*t292*t354*t974;
  t6420 = t6416 + t6417;
  t6422 = t6420*t1168;
  t6424 = t6415 + t6422;
  t6425 = t89*t6424;
  t6451 = t6444*t1372;
  t6454 = t6425 + t6451;
  t6464 = t89*t6444;
  t6465 = -1.*t6424*t1372;
  t6467 = t6464 + t6465;
  t6459 = -1.*t159*t6454;
  t6469 = t1338*t6467;
  t6472 = t6459 + t6469;
  t6477 = t1338*t6454;
  t6479 = t159*t6467;
  t6480 = t6477 + t6479;
  t6505 = t1117*t166*t354;
  t6510 = t6217*t1168;
  t6512 = t6505 + t6510;
  t6519 = -1.*t89*t6512;
  t6520 = -1.*t6349*t1372;
  t6521 = t6519 + t6520;
  t6526 = t89*t6349;
  t6527 = -1.*t6512*t1372;
  t6536 = t6526 + t6527;
  t6523 = t159*t6521;
  t6539 = t1338*t6536;
  t6541 = t6523 + t6539;
  t6543 = t1338*t6521;
  t6544 = -1.*t159*t6536;
  t6546 = t6543 + t6544;
  t6563 = t89*t6512;
  t6564 = t6349*t1372;
  t6566 = t6563 + t6564;
  t6571 = -1.*t159*t6566;
  t6573 = t6571 + t6539;
  t6575 = -1.*t1338*t6566;
  t6580 = t6575 + t6544;
  t6574 = -1.*t63*t6573;
  t6586 = t1715*t6573;
  t6606 = t1338*t6566;
  t6607 = t159*t6536;
  t6610 = t6606 + t6607;
  t6618 = -1.*t63*t6610;
  t6619 = t6586 + t6618;
  t6620 = t5*t6619;
  t6183 = t63*t5976;
  t6184 = t1715*t6159;
  t6189 = t6183 + t6184;
  t6651 = t1117*t267*t711;
  t6682 = t1117*t292*t974;
  t6683 = t6651 + t6682;
  t6684 = t166*t6683;
  t6688 = t482*t1168;
  t6689 = t6684 + t6688;
  t6691 = -1.*t89*t159*t6689;
  t6695 = -1.*t1338*t6689*t1372;
  t6698 = t6691 + t6695;
  t6700 = t1338*t89*t6689;
  t6701 = -1.*t159*t6689*t1372;
  t6703 = t6700 + t6701;
  t6739 = -1.*t1117*t267*t711;
  t6742 = -1.*t1117*t292*t974;
  t6745 = t6739 + t6742;
  t6730 = t1117*t292*t267;
  t6733 = -1.*t1117*t711*t974;
  t6736 = t6730 + t6733;
  t6738 = t89*t6736*t1168;
  t6747 = t6745*t1372;
  t6751 = t6738 + t6747;
  t6754 = t89*t6745;
  t6757 = -1.*t6736*t1168*t1372;
  t6759 = t6754 + t6757;
  t6753 = -1.*t159*t6751;
  t6761 = t1338*t6759;
  t6763 = t6753 + t6761;
  t6767 = t1338*t6751;
  t6768 = t159*t6759;
  t6769 = t6767 + t6768;
  t6766 = t63*t6763;
  t6771 = t1715*t6769;
  t6773 = t6766 + t6771;
  t6774 = t5*t6773;
  t6776 = t1715*t6763;
  t6777 = -1.*t63*t6769;
  t6779 = t6776 + t6777;
  t6783 = -1.*t2084*t6779;
  t6785 = t6774 + t6783;
  t6793 = -1.*t166*t482;
  t6797 = t6683*t1168;
  t6798 = t6793 + t6797;
  t6799 = -1.*t89*t6798;
  t6804 = -1.*t6736*t1372;
  t6806 = t6799 + t6804;
  t6808 = t89*t6736;
  t6810 = -1.*t6798*t1372;
  t6812 = t6808 + t6810;
  t6807 = t159*t6806;
  t6813 = t1338*t6812;
  t6817 = t6807 + t6813;
  t6825 = t1338*t6806;
  t6827 = -1.*t159*t6812;
  t6829 = t6825 + t6827;
  t6862 = t89*t6798;
  t6863 = t6736*t1372;
  t6864 = t6862 + t6863;
  t6866 = -1.*t159*t6864;
  t6867 = t6866 + t6813;
  t6869 = -1.*t1338*t6864;
  t6883 = t6869 + t6827;
  t6868 = -1.*t63*t6867;
  t6890 = t1715*t6867;
  t6907 = t1338*t6864;
  t6908 = t159*t6812;
  t6909 = t6907 + t6908;
  t6918 = -1.*t63*t6909;
  t6920 = t6890 + t6918;
  t6923 = t5*t6920;
  t6966 = -1.*t292*t267*t482;
  t6967 = t482*t711*t974;
  t6969 = t6966 + t6967;
  t6950 = -1.*t1117*t166;
  t6951 = -1.*t267*t482*t711;
  t6953 = -1.*t292*t482*t974;
  t6958 = t6951 + t6953;
  t6959 = t6958*t1168;
  t6961 = t6950 + t6959;
  t6963 = t89*t6961;
  t6970 = t6969*t1372;
  t6972 = t6963 + t6970;
  t6981 = t89*t6969;
  t6982 = -1.*t6961*t1372;
  t6985 = t6981 + t6982;
  t6978 = -1.*t159*t6972;
  t6991 = t1338*t6985;
  t6998 = t6978 + t6991;
  t7003 = t1338*t6972;
  t7004 = t159*t6985;
  t7011 = t7003 + t7004;
  t1589 = t63*t1417;
  t1917 = t1715*t1872;
  t1969 = t1589 + t1917;
  t2103 = t1715*t1417;
  t2173 = -1.*t63*t1872;
  t2195 = t2103 + t2173;
  t2974 = t63*t2887;
  t3158 = t1715*t3154;
  t3257 = t2974 + t3158;
  t3340 = t1715*t2887;
  t3419 = -1.*t63*t3154;
  t3422 = t3340 + t3419;
  t3999 = t63*t3998;
  t4037 = t1715*t4036;
  t4043 = t3999 + t4037;
  t4081 = t1715*t3998;
  t4184 = -1.*t63*t4036;
  t4191 = t4081 + t4184;
  t4840 = t63*t4828;
  t4963 = t1715*t4962;
  t4967 = t4840 + t4963;
  t4990 = t1715*t4828;
  t5005 = -1.*t63*t4962;
  t5015 = t4990 + t5005;
  t5459 = t63*t5443;
  t5504 = t1715*t5493;
  t5509 = t5459 + t5504;
  t5595 = t1715*t5443;
  t5613 = -1.*t63*t5493;
  t5619 = t5595 + t5613;
  t5763 = -1.*t63*t5761;
  t5770 = t1715*t5769;
  t5773 = t5763 + t5770;
  t5776 = t1715*t5761;
  t5787 = t63*t5769;
  t5795 = t5776 + t5787;
  t6006 = t1715*t6004;
  t6018 = t5986 + t6006;
  t6030 = t63*t6004;
  t6039 = t6021 + t6030;
  t6160 = -1.*t1715*t6159;
  t6164 = t5986 + t6160;
  t7101 = t2084*t6176;
  t6241 = t63*t6238;
  t6250 = t1715*t6247;
  t6253 = t6241 + t6250;
  t6257 = t1715*t6238;
  t6260 = -1.*t63*t6247;
  t6261 = t6257 + t6260;
  t6303 = t63*t6302;
  t6314 = t1715*t6313;
  t6315 = t6303 + t6314;
  t6320 = t1715*t6302;
  t6322 = -1.*t63*t6313;
  t6326 = t6320 + t6322;
  t6385 = t63*t6383;
  t6394 = t1715*t6391;
  t6401 = t6385 + t6394;
  t6403 = t1715*t6383;
  t6404 = -1.*t63*t6391;
  t6405 = t6403 + t6404;
  t6475 = t63*t6472;
  t6483 = t1715*t6480;
  t6484 = t6475 + t6483;
  t6489 = t1715*t6472;
  t6493 = -1.*t63*t6480;
  t6495 = t6489 + t6493;
  t6542 = -1.*t63*t6541;
  t6547 = t1715*t6546;
  t6549 = t6542 + t6547;
  t6553 = t1715*t6541;
  t6555 = t63*t6546;
  t6556 = t6553 + t6555;
  t6582 = t1715*t6580;
  t6584 = t6574 + t6582;
  t6594 = t63*t6580;
  t6597 = t6586 + t6594;
  t6612 = -1.*t1715*t6610;
  t6614 = t6574 + t6612;
  t6624 = t63*t6573;
  t6628 = t1715*t6610;
  t6631 = t6624 + t6628;
  t7178 = t2084*t6619;
  t6190 = t2084*t6189;
  t6192 = t6190 + t6177;
  t6699 = t63*t6698;
  t6708 = t1715*t6703;
  t6713 = t6699 + t6708;
  t6721 = t1715*t6698;
  t6723 = -1.*t63*t6703;
  t6724 = t6721 + t6723;
  t7208 = t2084*t6773;
  t7213 = t5*t6779;
  t7215 = t7208 + t7213;
  t6818 = -1.*t63*t6817;
  t6837 = t1715*t6829;
  t6838 = t6818 + t6837;
  t6841 = t1715*t6817;
  t6845 = t63*t6829;
  t6849 = t6841 + t6845;
  t6884 = t1715*t6883;
  t6886 = t6868 + t6884;
  t6892 = t63*t6883;
  t6893 = t6890 + t6892;
  t6910 = -1.*t1715*t6909;
  t6915 = t6868 + t6910;
  t6931 = t63*t6867;
  t6937 = t1715*t6909;
  t6939 = t6931 + t6937;
  t7242 = t2084*t6920;
  t7000 = t63*t6998;
  t7012 = t1715*t7011;
  t7016 = t7000 + t7012;
  t7019 = t1715*t6998;
  t7020 = -1.*t63*t7011;
  t7023 = t7019 + t7020;
  p_output1[0]=(t5*t5509 - 1.*t2084*t5619)*var2[3] + (t4967*t5 - 1.*t2084*t5015)*var2[4] + (-1.*t2084*t3422 + t3257*t5)*var2[5] + (-1.*t2084*t4191 + t4043*t5)*var2[6] + (-1.*t2084*t2195 + t1969*t5)*var2[7] + (-1.*t2084*t5773 + t5*t5795)*var2[8] + (-1.*t2084*t6018 + t5*t6039)*var2[9] + (-1.*t2084*t6164 + t6177)*var2[10] + t6192*var2[11];
  p_output1[1]=(-1.*t2084*t6176 + t5*t6189)*var2[3] + (t5*t6484 - 1.*t2084*t6495)*var2[4] + (t5*t6315 - 1.*t2084*t6326)*var2[5] + (t5*t6401 - 1.*t2084*t6405)*var2[6] + (t5*t6253 - 1.*t2084*t6261)*var2[7] + (-1.*t2084*t6549 + t5*t6556)*var2[8] + (-1.*t2084*t6584 + t5*t6597)*var2[9] + (-1.*t2084*t6614 + t6620)*var2[10] + (t6620 + t2084*t6631)*var2[11];
  p_output1[2]=(t5*t7016 - 1.*t2084*t7023)*var2[4] + t6785*var2[5] + t6785*var2[6] + (t5*t6713 - 1.*t2084*t6724)*var2[7] + (-1.*t2084*t6838 + t5*t6849)*var2[8] + (-1.*t2084*t6886 + t5*t6893)*var2[9] + (-1.*t2084*t6915 + t6923)*var2[10] + (t6923 + t2084*t6939)*var2[11];
  p_output1[3]=(t2084*t5509 + t5*t5619)*var2[3] + (t2084*t4967 + t5*t5015)*var2[4] + (t2084*t3257 + t3422*t5)*var2[5] + (t2084*t4043 + t4191*t5)*var2[6] + (t1969*t2084 + t2195*t5)*var2[7] + (t5*t5773 + t2084*t5795)*var2[8] + (t5*t6018 + t2084*t6039)*var2[9] + (t5*t6164 + t7101)*var2[10] + (-1.*t5*t6189 + t7101)*var2[11];
  p_output1[4]=t6192*var2[3] + (t2084*t6484 + t5*t6495)*var2[4] + (t2084*t6315 + t5*t6326)*var2[5] + (t2084*t6401 + t5*t6405)*var2[6] + (t2084*t6253 + t5*t6261)*var2[7] + (t5*t6549 + t2084*t6556)*var2[8] + (t5*t6584 + t2084*t6597)*var2[9] + (t5*t6614 + t7178)*var2[10] + (-1.*t5*t6631 + t7178)*var2[11];
  p_output1[5]=(t2084*t7016 + t5*t7023)*var2[4] + t7215*var2[5] + t7215*var2[6] + (t2084*t6713 + t5*t6724)*var2[7] + (t5*t6838 + t2084*t6849)*var2[8] + (t5*t6886 + t2084*t6893)*var2[9] + (t5*t6915 + t7242)*var2[10] + (-1.*t5*t6939 + t7242)*var2[11];
  p_output1[6]=(-1.*t166*t5220 + t6227)*var2[3] + (-1.*t166*t4394 - 1.*t1168*t468*t482)*var2[4] - 1.*t166*t2523*var2[5] - 1.*t166*t3613*var2[6] + t5689*var2[7];
  p_output1[7]=(-1.*t1056*t166 + t1117*t1168*t468)*var2[3] + (-1.*t1168*t354*t482 - 1.*t166*t6420)*var2[4] - 1.*t166*t6277*var2[5] - 1.*t166*t6349*var2[6] + t6512*var2[7];
  p_output1[8]=(-1.*t1117*t1168 - 1.*t166*t6958)*var2[4] - 1.*t166*t6736*var2[5] - 1.*t166*t6736*var2[6] + t6798*var2[7];
}



void dR_LeftAnkleJoint_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
