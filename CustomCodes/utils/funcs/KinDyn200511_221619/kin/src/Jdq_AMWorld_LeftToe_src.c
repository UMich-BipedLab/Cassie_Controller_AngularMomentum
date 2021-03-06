/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:27:18 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jdq_AMWorld_LeftToe_src.h"

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
  double t590;
  double t716;
  double t786;
  double t738;
  double t805;
  double t674;
  double t921;
  double t928;
  double t929;
  double t752;
  double t874;
  double t894;
  double t939;
  double t573;
  double t1065;
  double t1082;
  double t1091;
  double t595;
  double t633;
  double t639;
  double t908;
  double t943;
  double t947;
  double t958;
  double t997;
  double t1024;
  double t1094;
  double t1176;
  double t1028;
  double t1124;
  double t1153;
  double t477;
  double t1268;
  double t1297;
  double t1305;
  double t1327;
  double t1175;
  double t1307;
  double t1321;
  double t433;
  double t1349;
  double t1351;
  double t1383;
  double t1472;
  double t1325;
  double t1394;
  double t1410;
  double t183;
  double t1521;
  double t1576;
  double t1580;
  double t1679;
  double t133;
  double t1724;
  double t1770;
  double t1783;
  double t1809;
  double t1810;
  double t1855;
  double t1804;
  double t1864;
  double t1872;
  double t1914;
  double t1918;
  double t2013;
  double t2089;
  double t2134;
  double t2156;
  double t1876;
  double t2033;
  double t2061;
  double t2085;
  double t2176;
  double t2192;
  double t2233;
  double t2275;
  double t2282;
  double t2306;
  double t2316;
  double t2471;
  double t2482;
  double t2486;
  double t2452;
  double t2494;
  double t2537;
  double t2565;
  double t2566;
  double t2633;
  double t2657;
  double t2661;
  double t2551;
  double t2571;
  double t2620;
  double t2621;
  double t2693;
  double t2705;
  double t2733;
  double t2734;
  double t2740;
  double t2794;
  double t2806;
  double t3113;
  double t3149;
  double t2711;
  double t2720;
  double t2724;
  double t2757;
  double t2808;
  double t2812;
  double t2840;
  double t2853;
  double t2881;
  double t2903;
  double t2925;
  double t2931;
  double t2993;
  double t3020;
  double t1412;
  double t1588;
  double t1640;
  double t1683;
  double t1693;
  double t1702;
  double t3155;
  double t3193;
  double t3198;
  double t3208;
  double t3210;
  double t3214;
  double t3219;
  double t3221;
  double t3230;
  double t3259;
  double t3271;
  double t2197;
  double t2199;
  double t2213;
  double t2294;
  double t2317;
  double t2331;
  double t2338;
  double t2352;
  double t2379;
  double t2381;
  double t2385;
  double t2388;
  double t2407;
  double t2434;
  double t3763;
  double t3785;
  double t3787;
  double t3591;
  double t3615;
  double t3653;
  double t3914;
  double t3966;
  double t3974;
  double t3587;
  double t3722;
  double t3811;
  double t3829;
  double t3843;
  double t3872;
  double t3875;
  double t3981;
  double t4020;
  double t4055;
  double t4062;
  double t4086;
  double t4042;
  double t4090;
  double t4102;
  double t4170;
  double t4182;
  double t4187;
  double t4150;
  double t4219;
  double t4222;
  double t4243;
  double t4280;
  double t4284;
  double t2437;
  double t3021;
  double t3030;
  double t3321;
  double t3340;
  double t3359;
  double t4240;
  double t4312;
  double t4319;
  double t4389;
  double t4391;
  double t4398;
  double t3421;
  double t3427;
  double t3442;
  double t3499;
  double t4624;
  double t4638;
  double t4645;
  double t4506;
  double t4534;
  double t4561;
  double t4577;
  double t4582;
  double t4587;
  double t4594;
  double t4658;
  double t4661;
  double t4681;
  double t4696;
  double t4709;
  double t4668;
  double t4718;
  double t4725;
  double t4779;
  double t4791;
  double t4797;
  double t4762;
  double t4798;
  double t4834;
  double t4852;
  double t4853;
  double t4942;
  double t4846;
  double t4944;
  double t4968;
  double t5030;
  double t5046;
  double t5069;
  double t3043;
  double t3052;
  double t3089;
  double t5306;
  double t5312;
  double t5325;
  double t1648;
  double t1708;
  double t1714;
  double t5278;
  double t5279;
  double t5288;
  double t5348;
  double t5349;
  double t5350;
  double t3372;
  double t3396;
  double t3420;
  double t4416;
  double t4434;
  double t4439;
  double t5296;
  double t5333;
  double t5343;
  double t5367;
  double t5372;
  double t5374;
  double t4351;
  double t4402;
  double t4410;
  double t5386;
  double t5388;
  double t5392;
  double t5395;
  double t4461;
  double t4474;
  double t4476;
  double t5111;
  double t5119;
  double t5134;
  double t4981;
  double t5075;
  double t5081;
  double t5199;
  double t5210;
  double t5234;
  double t5479;
  double t5490;
  double t5506;
  double t5520;
  double t5523;
  double t5546;
  double t5561;
  double t5570;
  double t5589;
  double t5592;
  double t5514;
  double t5556;
  double t5593;
  double t5594;
  double t5611;
  double t5634;
  double t5641;
  double t5643;
  double t5664;
  double t5666;
  double t5676;
  double t5694;
  double t5697;
  double t5704;
  double t5714;
  double t5721;
  double t5722;
  double t5723;
  double t5813;
  double t5818;
  double t5819;
  double t5820;
  double t5828;
  double t5829;
  double t5846;
  double t5856;
  double t5866;
  t590 = Cos(var1[3]);
  t716 = Cos(var1[5]);
  t786 = Sin(var1[4]);
  t738 = Sin(var1[3]);
  t805 = Sin(var1[5]);
  t674 = Cos(var1[6]);
  t921 = t590*t716*t786;
  t928 = t738*t805;
  t929 = t921 + t928;
  t752 = -1.*t716*t738;
  t874 = t590*t786*t805;
  t894 = t752 + t874;
  t939 = Sin(var1[6]);
  t573 = Cos(var1[8]);
  t1065 = t674*t929;
  t1082 = -1.*t894*t939;
  t1091 = t1065 + t1082;
  t595 = Cos(var1[4]);
  t633 = Cos(var1[7]);
  t639 = t590*t595*t633;
  t908 = t674*t894;
  t943 = t929*t939;
  t947 = t908 + t943;
  t958 = Sin(var1[7]);
  t997 = t947*t958;
  t1024 = t639 + t997;
  t1094 = Sin(var1[8]);
  t1176 = Cos(var1[9]);
  t1028 = t573*t1024;
  t1124 = t1091*t1094;
  t1153 = t1028 + t1124;
  t477 = Sin(var1[9]);
  t1268 = t573*t1091;
  t1297 = -1.*t1024*t1094;
  t1305 = t1268 + t1297;
  t1327 = Cos(var1[10]);
  t1175 = -1.*t477*t1153;
  t1307 = t1176*t1305;
  t1321 = t1175 + t1307;
  t433 = Sin(var1[10]);
  t1349 = t1176*t1153;
  t1351 = t477*t1305;
  t1383 = t1349 + t1351;
  t1472 = Cos(var1[11]);
  t1325 = t433*t1321;
  t1394 = t1327*t1383;
  t1410 = t1325 + t1394;
  t183 = Sin(var1[11]);
  t1521 = t1327*t1321;
  t1576 = -1.*t433*t1383;
  t1580 = t1521 + t1576;
  t1679 = Cos(var1[12]);
  t133 = Sin(var1[12]);
  t1724 = t1679*t183;
  t1770 = t1472*t133;
  t1783 = 0. + t1724 + t1770;
  t1809 = t1472*t1679;
  t1810 = -1.*t183*t133;
  t1855 = 0. + t1809 + t1810;
  t1804 = -1.*t433*t1783;
  t1864 = t1327*t1855;
  t1872 = 0. + t1804 + t1864;
  t1914 = t1327*t1783;
  t1918 = t433*t1855;
  t2013 = 0. + t1914 + t1918;
  t2089 = t477*t1872;
  t2134 = t1176*t2013;
  t2156 = 0. + t2089 + t2134;
  t1876 = t1176*t1872;
  t2033 = -1.*t477*t2013;
  t2061 = 0. + t1876 + t2033;
  t2085 = t573*t2061;
  t2176 = -1.*t2156*t1094;
  t2192 = 0. + t2085 + t2176;
  t2233 = t573*t2156;
  t2275 = t2061*t1094;
  t2282 = 0. + t2233 + t2275;
  t2306 = t958*t2192;
  t2316 = 0. + t2306;
  t2471 = -1.*t1472*t1679;
  t2482 = t183*t133;
  t2486 = 0. + t2471 + t2482;
  t2452 = t433*t1783;
  t2494 = t1327*t2486;
  t2537 = 0. + t2452 + t2494;
  t2565 = -1.*t433*t2486;
  t2566 = 0. + t1914 + t2565;
  t2633 = t1176*t2537;
  t2657 = t477*t2566;
  t2661 = 0. + t2633 + t2657;
  t2551 = -1.*t477*t2537;
  t2571 = t1176*t2566;
  t2620 = 0. + t2551 + t2571;
  t2621 = t573*t2620;
  t2693 = -1.*t2661*t1094;
  t2705 = 0. + t2621 + t2693;
  t2733 = t573*t2661;
  t2734 = t2620*t1094;
  t2740 = 0. + t2733 + t2734;
  t2794 = t958*t2705;
  t2806 = 0. + t2794;
  t3113 = -1.*t633;
  t3149 = 0. + t3113;
  t2711 = t633*t2705;
  t2720 = 0. + t2711;
  t2724 = -1.*t786*t2720;
  t2757 = -1.*t939*t2740;
  t2808 = t674*t2806;
  t2812 = 0. + t2757 + t2808;
  t2840 = t805*t2812;
  t2853 = t674*t2740;
  t2881 = t939*t2806;
  t2903 = 0. + t2853 + t2881;
  t2925 = t716*t2903;
  t2931 = 0. + t2840 + t2925;
  t2993 = t595*t2931;
  t3020 = 0. + t2724 + t2993;
  t1412 = -1.*t183*t1410;
  t1588 = t1472*t1580;
  t1640 = t1412 + t1588;
  t1683 = t1472*t1410;
  t1693 = t183*t1580;
  t1702 = t1683 + t1693;
  t3155 = t674*t3149;
  t3193 = 0. + t3155;
  t3198 = t3193*t805;
  t3208 = t3149*t939;
  t3210 = 0. + t3208;
  t3214 = t716*t3210;
  t3219 = 0. + t3198 + t3214;
  t3221 = t595*t3219;
  t3230 = 0. + t958;
  t3259 = -1.*t786*t3230;
  t3271 = 0. + t3221 + t3259;
  t2197 = t633*t2192;
  t2199 = 0. + t2197;
  t2213 = -1.*t786*t2199;
  t2294 = -1.*t939*t2282;
  t2317 = t674*t2316;
  t2331 = 0. + t2294 + t2317;
  t2338 = t805*t2331;
  t2352 = t674*t2282;
  t2379 = t939*t2316;
  t2381 = 0. + t2352 + t2379;
  t2385 = t716*t2381;
  t2388 = 0. + t2338 + t2385;
  t2407 = t595*t2388;
  t2434 = 0. + t2213 + t2407;
  t3763 = t716*t738*t786;
  t3785 = -1.*t590*t805;
  t3787 = t3763 + t3785;
  t3591 = t590*t716;
  t3615 = t738*t786*t805;
  t3653 = t3591 + t3615;
  t3914 = t674*t3787;
  t3966 = -1.*t3653*t939;
  t3974 = t3914 + t3966;
  t3587 = t595*t633*t738;
  t3722 = t674*t3653;
  t3811 = t3787*t939;
  t3829 = t3722 + t3811;
  t3843 = t3829*t958;
  t3872 = t3587 + t3843;
  t3875 = t573*t3872;
  t3981 = t3974*t1094;
  t4020 = t3875 + t3981;
  t4055 = t573*t3974;
  t4062 = -1.*t3872*t1094;
  t4086 = t4055 + t4062;
  t4042 = -1.*t477*t4020;
  t4090 = t1176*t4086;
  t4102 = t4042 + t4090;
  t4170 = t1176*t4020;
  t4182 = t477*t4086;
  t4187 = t4170 + t4182;
  t4150 = t433*t4102;
  t4219 = t1327*t4187;
  t4222 = t4150 + t4219;
  t4243 = t1327*t4102;
  t4280 = -1.*t433*t4187;
  t4284 = t4243 + t4280;
  t2437 = 0.000171*t2434;
  t3021 = -0.000099*t3020;
  t3030 = t2437 + t3021;
  t3321 = 0.000449*t3271;
  t3340 = -1.e-6*t3020;
  t3359 = t3321 + t3340;
  t4240 = -1.*t183*t4222;
  t4312 = t1472*t4284;
  t4319 = t4240 + t4312;
  t4389 = t1472*t4222;
  t4391 = t183*t4284;
  t4398 = t4389 + t4391;
  t3421 = -1.e-6*t3271;
  t3427 = -0.000099*t2434;
  t3442 = 0.000287*t3020;
  t3499 = t3421 + t3427 + t3442;
  t4624 = t595*t716*t674;
  t4638 = -1.*t595*t805*t939;
  t4645 = t4624 + t4638;
  t4506 = -1.*t633*t786;
  t4534 = t595*t674*t805;
  t4561 = t595*t716*t939;
  t4577 = t4534 + t4561;
  t4582 = t4577*t958;
  t4587 = t4506 + t4582;
  t4594 = t573*t4587;
  t4658 = t4645*t1094;
  t4661 = t4594 + t4658;
  t4681 = t573*t4645;
  t4696 = -1.*t4587*t1094;
  t4709 = t4681 + t4696;
  t4668 = -1.*t477*t4661;
  t4718 = t1176*t4709;
  t4725 = t4668 + t4718;
  t4779 = t1176*t4661;
  t4791 = t477*t4709;
  t4797 = t4779 + t4791;
  t4762 = t433*t4725;
  t4798 = t1327*t4797;
  t4834 = t4762 + t4798;
  t4852 = t1327*t4725;
  t4853 = -1.*t433*t4797;
  t4942 = t4852 + t4853;
  t4846 = -1.*t183*t4834;
  t4944 = t1472*t4942;
  t4968 = t4846 + t4944;
  t5030 = t1472*t4834;
  t5046 = t183*t4942;
  t5069 = t5030 + t5046;
  t3043 = -1.*t633*t947;
  t3052 = t590*t595*t958;
  t3089 = t3043 + t3052;
  t5306 = t716*t2812;
  t5312 = -1.*t805*t2903;
  t5325 = 0. + t5306 + t5312;
  t1648 = t133*t1640;
  t1708 = t1679*t1702;
  t1714 = t1648 + t1708;
  t5278 = t716*t3193;
  t5279 = -1.*t805*t3210;
  t5288 = 0. + t5278 + t5279;
  t5348 = t716*t2331;
  t5349 = -1.*t805*t2381;
  t5350 = 0. + t5348 + t5349;
  t3372 = -1.*t1679*t1640;
  t3396 = t133*t1702;
  t3420 = t3372 + t3396;
  t4416 = -1.*t633*t3829;
  t4434 = t595*t738*t958;
  t4439 = t4416 + t4434;
  t5296 = 0.000449*t5288;
  t5333 = -1.e-6*t5325;
  t5343 = t5296 + t5333;
  t5367 = 0.000171*t5350;
  t5372 = -0.000099*t5325;
  t5374 = t5367 + t5372;
  t4351 = t133*t4319;
  t4402 = t1679*t4398;
  t4410 = t4351 + t4402;
  t5386 = -1.e-6*t5288;
  t5388 = -0.000099*t5350;
  t5392 = 0.000287*t5325;
  t5395 = t5386 + t5388 + t5392;
  t4461 = -1.*t1679*t4319;
  t4474 = t133*t4398;
  t4476 = t4461 + t4474;
  t5111 = -1.*t633*t4577;
  t5119 = -1.*t786*t958;
  t5134 = t5111 + t5119;
  t4981 = t133*t4968;
  t5075 = t1679*t5069;
  t5081 = t4981 + t5075;
  t5199 = -1.*t1679*t4968;
  t5210 = t133*t5069;
  t5234 = t5199 + t5210;
  t5479 = 0.000449*t3230;
  t5490 = -1.e-6*t2720;
  t5506 = t5479 + t5490;
  t5520 = 0.000171*t2199;
  t5523 = -0.000099*t2720;
  t5546 = t5520 + t5523;
  t5561 = -1.e-6*t3230;
  t5570 = -0.000099*t2199;
  t5589 = 0.000287*t2720;
  t5592 = t5561 + t5570 + t5589;
  t5514 = t3089*t5506;
  t5556 = t5546*t1714;
  t5593 = t5592*t3420;
  t5594 = t5514 + t5556 + t5593;
  t5611 = t4439*t5506;
  t5634 = t5546*t4410;
  t5641 = t5592*t4476;
  t5643 = t5611 + t5634 + t5641;
  t5664 = t5134*t5506;
  t5666 = t5546*t5081;
  t5676 = t5592*t5234;
  t5694 = t5664 + t5666 + t5676;
  t5697 = 0.000171*t2282;
  t5704 = -0.000099*t2740;
  t5714 = t5697 + t5704;
  t5721 = -0.000099*t2282;
  t5722 = 0.000287*t2740;
  t5723 = t5721 + t5722;
  t5813 = 0.000449*t3089;
  t5818 = -1.e-6*t3420;
  t5819 = t5813 + t5818;
  t5820 = 0.000449*t4439;
  t5828 = -1.e-6*t4476;
  t5829 = t5820 + t5828;
  t5846 = 0.000449*t5134;
  t5856 = -1.e-6*t5234;
  t5866 = t5846 + t5856;
  p_output1[0]=0;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=0;
  p_output1[9]=t1714*t3030 + t3089*t3359 + t3420*t3499;
  p_output1[10]=t3030*t4410 + t3359*t4439 + t3499*t4476;
  p_output1[11]=t3030*t5081 + t3359*t5134 + t3499*t5234;
  p_output1[12]=t3089*t5343 + t1714*t5374 + t3420*t5395;
  p_output1[13]=t4439*t5343 + t4410*t5374 + t4476*t5395;
  p_output1[14]=t5134*t5343 + t5081*t5374 + t5234*t5395;
  p_output1[15]=t5594;
  p_output1[16]=t5643;
  p_output1[17]=t5694;
  p_output1[18]=t5594;
  p_output1[19]=t5643;
  p_output1[20]=t5694;
  p_output1[21]=-1.e-6*t2740*t3089 + t1714*t5714 + t3420*t5723;
  p_output1[22]=-1.e-6*t2740*t4439 + t4410*t5714 + t4476*t5723;
  p_output1[23]=-1.e-6*t2740*t5134 + t5081*t5714 + t5234*t5723;
  p_output1[24]=t5819;
  p_output1[25]=t5829;
  p_output1[26]=t5866;
  p_output1[27]=t5819;
  p_output1[28]=t5829;
  p_output1[29]=t5866;
  p_output1[30]=t5819;
  p_output1[31]=t5829;
  p_output1[32]=t5866;
  p_output1[33]=t5819;
  p_output1[34]=t5829;
  p_output1[35]=t5866;
  p_output1[36]=t5819;
  p_output1[37]=t5829;
  p_output1[38]=t5866;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
  p_output1[45]=0;
  p_output1[46]=0;
  p_output1[47]=0;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
  p_output1[54]=0;
  p_output1[55]=0;
  p_output1[56]=0;
  p_output1[57]=0;
  p_output1[58]=0;
  p_output1[59]=0;
}



void Jdq_AMWorld_LeftToe_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
