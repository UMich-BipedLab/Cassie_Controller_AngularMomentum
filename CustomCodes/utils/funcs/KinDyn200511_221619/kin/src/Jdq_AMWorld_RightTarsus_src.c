/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:30:31 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jdq_AMWorld_RightTarsus_src.h"

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
  double t427;
  double t430;
  double t448;
  double t882;
  double t774;
  double t785;
  double t903;
  double t1104;
  double t803;
  double t1012;
  double t1022;
  double t751;
  double t1115;
  double t1132;
  double t1135;
  double t359;
  double t402;
  double t409;
  double t519;
  double t1343;
  double t1092;
  double t1141;
  double t1181;
  double t727;
  double t1398;
  double t1418;
  double t1432;
  double t453;
  double t220;
  double t1285;
  double t1445;
  double t1468;
  double t1560;
  double t1621;
  double t1624;
  double t1648;
  double t1760;
  double t624;
  double t1911;
  double t1936;
  double t1947;
  double t1892;
  double t1951;
  double t1958;
  double t2016;
  double t2047;
  double t1975;
  double t2065;
  double t2088;
  double t2122;
  double t2124;
  double t2137;
  double t2157;
  double t2168;
  double t2417;
  double t2464;
  double t340;
  double t367;
  double t461;
  double t487;
  double t494;
  double t540;
  double t555;
  double t629;
  double t651;
  double t679;
  double t695;
  double t1511;
  double t1514;
  double t1542;
  double t1634;
  double t1789;
  double t1832;
  double t1842;
  double t1851;
  double t1854;
  double t1859;
  double t1871;
  double t1881;
  double t1883;
  double t1889;
  double t2116;
  double t2117;
  double t2120;
  double t2154;
  double t2182;
  double t2187;
  double t2199;
  double t2220;
  double t2227;
  double t2265;
  double t2278;
  double t2293;
  double t2317;
  double t2374;
  double t2454;
  double t2473;
  double t2488;
  double t2500;
  double t2518;
  double t2527;
  double t2490;
  double t2551;
  double t2561;
  double t2713;
  double t2720;
  double t2732;
  double t2745;
  double t2751;
  double t2761;
  double t2733;
  double t2765;
  double t2808;
  double t2835;
  double t2838;
  double t2857;
  double t2827;
  double t2859;
  double t2889;
  double t2911;
  double t2915;
  double t2916;
  double t2909;
  double t2927;
  double t2966;
  double t2976;
  double t2993;
  double t3005;
  double t726;
  double t1891;
  double t2391;
  double t2414;
  double t2616;
  double t2630;
  double t2641;
  double t2650;
  double t3374;
  double t3375;
  double t3382;
  double t3417;
  double t3422;
  double t3453;
  double t3393;
  double t3471;
  double t3486;
  double t3626;
  double t3633;
  double t3634;
  double t3698;
  double t3707;
  double t3744;
  double t3687;
  double t3745;
  double t3749;
  double t3799;
  double t3826;
  double t3876;
  double t3768;
  double t3902;
  double t3985;
  double t4011;
  double t4045;
  double t4058;
  double t3093;
  double t3105;
  double t3113;
  double t3114;
  double t3988;
  double t4061;
  double t4062;
  double t4081;
  double t4089;
  double t4123;
  double t4332;
  double t4344;
  double t4353;
  double t4453;
  double t4532;
  double t4541;
  double t4547;
  double t4549;
  double t4580;
  double t4546;
  double t4650;
  double t4658;
  double t4676;
  double t4713;
  double t4810;
  double t4665;
  double t4835;
  double t4837;
  double t4851;
  double t4887;
  double t4968;
  double t4839;
  double t4978;
  double t4985;
  double t4999;
  double t5012;
  double t5047;
  double t2448;
  double t2566;
  double t2570;
  double t5216;
  double t5229;
  double t5371;
  double t5393;
  double t5412;
  double t5415;
  double t5450;
  double t5458;
  double t5473;
  double t2972;
  double t3010;
  double t3062;
  double t3125;
  double t3128;
  double t3156;
  double t5380;
  double t5425;
  double t5493;
  double t5512;
  double t3252;
  double t3515;
  double t3621;
  double t5576;
  double t5609;
  double t5632;
  double t5663;
  double t4075;
  double t4140;
  double t4165;
  double t5675;
  double t5688;
  double t5690;
  double t5712;
  double t4189;
  double t4191;
  double t4232;
  double t4299;
  double t4366;
  double t4408;
  double t4992;
  double t5067;
  double t5094;
  double t5128;
  double t5165;
  double t5176;
  double t6020;
  double t6051;
  double t6068;
  double t6090;
  double t6299;
  double t6307;
  double t6316;
  double t6326;
  double t6345;
  double t6407;
  double t6414;
  double t6441;
  double t6159;
  double t6329;
  double t6442;
  double t6536;
  double t6546;
  double t6565;
  double t6573;
  double t6576;
  double t6626;
  double t6661;
  double t6689;
  double t6694;
  double t6722;
  double t6757;
  double t6758;
  double t6806;
  double t6807;
  double t6814;
  double t6848;
  double t6861;
  double t6897;
  double t7180;
  double t7181;
  double t7197;
  double t7244;
  double t7318;
  double t7342;
  double t7343;
  double t7344;
  double t7347;
  double t7358;
  double t7378;
  double t7385;
  t427 = Cos(var1[14]);
  t430 = -1.*t427;
  t448 = 0. + t430;
  t882 = Cos(var1[17]);
  t774 = Cos(var1[18]);
  t785 = Sin(var1[17]);
  t903 = Sin(var1[18]);
  t1104 = Cos(var1[16]);
  t803 = t774*t785;
  t1012 = t882*t903;
  t1022 = 0. + t803 + t1012;
  t751 = Sin(var1[16]);
  t1115 = t882*t774;
  t1132 = -1.*t785*t903;
  t1135 = 0. + t1115 + t1132;
  t359 = Sin(var1[4]);
  t402 = Cos(var1[4]);
  t409 = Cos(var1[5]);
  t519 = Cos(var1[13]);
  t1343 = Sin(var1[15]);
  t1092 = -1.*t751*t1022;
  t1141 = t1104*t1135;
  t1181 = 0. + t1092 + t1141;
  t727 = Cos(var1[15]);
  t1398 = t1104*t1022;
  t1418 = t751*t1135;
  t1432 = 0. + t1398 + t1418;
  t453 = Sin(var1[13]);
  t220 = Sin(var1[14]);
  t1285 = t727*t1181;
  t1445 = -1.*t1343*t1432;
  t1468 = 0. + t1285 + t1445;
  t1560 = t1343*t1181;
  t1621 = t727*t1432;
  t1624 = 0. + t1560 + t1621;
  t1648 = t220*t1468;
  t1760 = 0. + t1648;
  t624 = Sin(var1[5]);
  t1911 = -1.*t882*t774;
  t1936 = t785*t903;
  t1947 = 0. + t1911 + t1936;
  t1892 = t751*t1022;
  t1951 = t1104*t1947;
  t1958 = 0. + t1892 + t1951;
  t2016 = -1.*t751*t1947;
  t2047 = 0. + t1398 + t2016;
  t1975 = -1.*t1343*t1958;
  t2065 = t727*t2047;
  t2088 = 0. + t1975 + t2065;
  t2122 = t727*t1958;
  t2124 = t1343*t2047;
  t2137 = 0. + t2122 + t2124;
  t2157 = t220*t2088;
  t2168 = 0. + t2157;
  t2417 = Cos(var1[3]);
  t2464 = Sin(var1[3]);
  t340 = 0. + t220;
  t367 = -1.*t340*t359;
  t461 = t448*t453;
  t487 = 0. + t461;
  t494 = t409*t487;
  t540 = t519*t448;
  t555 = 0. + t540;
  t629 = t555*t624;
  t651 = 0. + t494 + t629;
  t679 = t402*t651;
  t695 = 0. + t367 + t679;
  t1511 = t427*t1468;
  t1514 = 0. + t1511;
  t1542 = -1.*t1514*t359;
  t1634 = t519*t1624;
  t1789 = t453*t1760;
  t1832 = 0. + t1634 + t1789;
  t1842 = t409*t1832;
  t1851 = -1.*t453*t1624;
  t1854 = t519*t1760;
  t1859 = 0. + t1851 + t1854;
  t1871 = t1859*t624;
  t1881 = 0. + t1842 + t1871;
  t1883 = t402*t1881;
  t1889 = 0. + t1542 + t1883;
  t2116 = t427*t2088;
  t2117 = 0. + t2116;
  t2120 = -1.*t2117*t359;
  t2154 = t519*t2137;
  t2182 = t453*t2168;
  t2187 = 0. + t2154 + t2182;
  t2199 = t409*t2187;
  t2220 = -1.*t453*t2137;
  t2227 = t519*t2168;
  t2265 = 0. + t2220 + t2227;
  t2278 = t2265*t624;
  t2293 = 0. + t2199 + t2278;
  t2317 = t402*t2293;
  t2374 = 0. + t2120 + t2317;
  t2454 = t2417*t409*t359;
  t2473 = t2464*t624;
  t2488 = t2454 + t2473;
  t2500 = -1.*t409*t2464;
  t2518 = t2417*t359*t624;
  t2527 = t2500 + t2518;
  t2490 = t453*t2488;
  t2551 = t519*t2527;
  t2561 = t2490 + t2551;
  t2713 = t519*t2488;
  t2720 = -1.*t453*t2527;
  t2732 = t2713 + t2720;
  t2745 = t427*t2417*t402;
  t2751 = t220*t2561;
  t2761 = t2745 + t2751;
  t2733 = t1343*t2732;
  t2765 = t727*t2761;
  t2808 = t2733 + t2765;
  t2835 = t727*t2732;
  t2838 = -1.*t1343*t2761;
  t2857 = t2835 + t2838;
  t2827 = -1.*t751*t2808;
  t2859 = t1104*t2857;
  t2889 = t2827 + t2859;
  t2911 = t1104*t2808;
  t2915 = t751*t2857;
  t2916 = t2911 + t2915;
  t2909 = t785*t2889;
  t2927 = t882*t2916;
  t2966 = t2909 + t2927;
  t2976 = t882*t2889;
  t2993 = -1.*t785*t2916;
  t3005 = t2976 + t2993;
  t726 = 0.0239*t695;
  t1891 = -0.000036*t1889;
  t2391 = 0.000063*t2374;
  t2414 = t726 + t1891 + t2391;
  t2616 = 0.000063*t695;
  t2630 = -0.00288*t1889;
  t2641 = 0.00113*t2374;
  t2650 = t2616 + t2630 + t2641;
  t3374 = t409*t2464*t359;
  t3375 = -1.*t2417*t624;
  t3382 = t3374 + t3375;
  t3417 = t2417*t409;
  t3422 = t2464*t359*t624;
  t3453 = t3417 + t3422;
  t3393 = t453*t3382;
  t3471 = t519*t3453;
  t3486 = t3393 + t3471;
  t3626 = t519*t3382;
  t3633 = -1.*t453*t3453;
  t3634 = t3626 + t3633;
  t3698 = t427*t402*t2464;
  t3707 = t220*t3486;
  t3744 = t3698 + t3707;
  t3687 = t1343*t3634;
  t3745 = t727*t3744;
  t3749 = t3687 + t3745;
  t3799 = t727*t3634;
  t3826 = -1.*t1343*t3744;
  t3876 = t3799 + t3826;
  t3768 = -1.*t751*t3749;
  t3902 = t1104*t3876;
  t3985 = t3768 + t3902;
  t4011 = t1104*t3749;
  t4045 = t751*t3876;
  t4058 = t4011 + t4045;
  t3093 = -0.000036*t695;
  t3105 = 0.0231*t1889;
  t3113 = -0.00288*t2374;
  t3114 = t3093 + t3105 + t3113;
  t3988 = t785*t3985;
  t4061 = t882*t4058;
  t4062 = t3988 + t4061;
  t4081 = t882*t3985;
  t4089 = -1.*t785*t4058;
  t4123 = t4081 + t4089;
  t4332 = t402*t409*t453;
  t4344 = t519*t402*t624;
  t4353 = t4332 + t4344;
  t4453 = t519*t402*t409;
  t4532 = -1.*t402*t453*t624;
  t4541 = t4453 + t4532;
  t4547 = -1.*t427*t359;
  t4549 = t220*t4353;
  t4580 = t4547 + t4549;
  t4546 = t1343*t4541;
  t4650 = t727*t4580;
  t4658 = t4546 + t4650;
  t4676 = t727*t4541;
  t4713 = -1.*t1343*t4580;
  t4810 = t4676 + t4713;
  t4665 = -1.*t751*t4658;
  t4835 = t1104*t4810;
  t4837 = t4665 + t4835;
  t4851 = t1104*t4658;
  t4887 = t751*t4810;
  t4968 = t4851 + t4887;
  t4839 = t785*t4837;
  t4978 = t882*t4968;
  t4985 = t4839 + t4978;
  t4999 = t882*t4837;
  t5012 = -1.*t785*t4968;
  t5047 = t4999 + t5012;
  t2448 = t2417*t402*t220;
  t2566 = -1.*t427*t2561;
  t2570 = t2448 + t2566;
  t5216 = t555*t409;
  t5229 = -1.*t487*t624;
  t5371 = 0. + t5216 + t5229;
  t5393 = t409*t1859;
  t5412 = -1.*t1832*t624;
  t5415 = 0. + t5393 + t5412;
  t5450 = t409*t2265;
  t5458 = -1.*t2187*t624;
  t5473 = 0. + t5450 + t5458;
  t2972 = t903*t2966;
  t3010 = -1.*t774*t3005;
  t3062 = t2972 + t3010;
  t3125 = t774*t2966;
  t3128 = t903*t3005;
  t3156 = t3125 + t3128;
  t5380 = 0.0239*t5371;
  t5425 = -0.000036*t5415;
  t5493 = 0.000063*t5473;
  t5512 = t5380 + t5425 + t5493;
  t3252 = t402*t220*t2464;
  t3515 = -1.*t427*t3486;
  t3621 = t3252 + t3515;
  t5576 = 0.000063*t5371;
  t5609 = -0.00288*t5415;
  t5632 = 0.00113*t5473;
  t5663 = t5576 + t5609 + t5632;
  t4075 = t903*t4062;
  t4140 = -1.*t774*t4123;
  t4165 = t4075 + t4140;
  t5675 = -0.000036*t5371;
  t5688 = 0.0231*t5415;
  t5690 = -0.00288*t5473;
  t5712 = t5675 + t5688 + t5690;
  t4189 = t774*t4062;
  t4191 = t903*t4123;
  t4232 = t4189 + t4191;
  t4299 = -1.*t220*t359;
  t4366 = -1.*t427*t4353;
  t4408 = t4299 + t4366;
  t4992 = t903*t4985;
  t5067 = -1.*t774*t5047;
  t5094 = t4992 + t5067;
  t5128 = t774*t4985;
  t5165 = t903*t5047;
  t5176 = t5128 + t5165;
  t6020 = 0.0239*t340;
  t6051 = -0.000036*t1514;
  t6068 = 0.000063*t2117;
  t6090 = t6020 + t6051 + t6068;
  t6299 = 0.000063*t340;
  t6307 = -0.00288*t1514;
  t6316 = 0.00113*t2117;
  t6326 = t6299 + t6307 + t6316;
  t6345 = -0.000036*t340;
  t6407 = 0.0231*t1514;
  t6414 = -0.00288*t2117;
  t6441 = t6345 + t6407 + t6414;
  t6159 = t6090*t2570;
  t6329 = t6326*t3062;
  t6442 = t6441*t3156;
  t6536 = t6159 + t6329 + t6442;
  t6546 = t6090*t3621;
  t6565 = t6326*t4165;
  t6573 = t6441*t4232;
  t6576 = t6546 + t6565 + t6573;
  t6626 = t6090*t4408;
  t6661 = t6326*t5094;
  t6689 = t6441*t5176;
  t6694 = t6626 + t6661 + t6689;
  t6722 = -0.000036*t1624;
  t6757 = 0.000063*t2137;
  t6758 = t6722 + t6757;
  t6806 = -0.00288*t1624;
  t6807 = 0.00113*t2137;
  t6814 = t6806 + t6807;
  t6848 = 0.0231*t1624;
  t6861 = -0.00288*t2137;
  t6897 = t6848 + t6861;
  t7180 = 0.0239*t2570;
  t7181 = 0.000063*t3062;
  t7197 = -0.000036*t3156;
  t7244 = t7180 + t7181 + t7197;
  t7318 = 0.0239*t3621;
  t7342 = 0.000063*t4165;
  t7343 = -0.000036*t4232;
  t7344 = t7318 + t7342 + t7343;
  t7347 = 0.0239*t4408;
  t7358 = 0.000063*t5094;
  t7378 = -0.000036*t5176;
  t7385 = t7347 + t7358 + t7378;
  p_output1[0]=0;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=0;
  p_output1[9]=t2414*t2570 + t2650*t3062 + t3114*t3156;
  p_output1[10]=t2414*t3621 + t2650*t4165 + t3114*t4232;
  p_output1[11]=t2414*t4408 + t2650*t5094 + t3114*t5176;
  p_output1[12]=t2570*t5512 + t3062*t5663 + t3156*t5712;
  p_output1[13]=t3621*t5512 + t4165*t5663 + t4232*t5712;
  p_output1[14]=t4408*t5512 + t5094*t5663 + t5176*t5712;
  p_output1[15]=t6536;
  p_output1[16]=t6576;
  p_output1[17]=t6694;
  p_output1[18]=0;
  p_output1[19]=0;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=0;
  p_output1[31]=0;
  p_output1[32]=0;
  p_output1[33]=0;
  p_output1[34]=0;
  p_output1[35]=0;
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=t6536;
  p_output1[40]=t6576;
  p_output1[41]=t6694;
  p_output1[42]=t2570*t6758 + t3062*t6814 + t3156*t6897;
  p_output1[43]=t3621*t6758 + t4165*t6814 + t4232*t6897;
  p_output1[44]=t4408*t6758 + t5094*t6814 + t5176*t6897;
  p_output1[45]=t7244;
  p_output1[46]=t7344;
  p_output1[47]=t7385;
  p_output1[48]=t7244;
  p_output1[49]=t7344;
  p_output1[50]=t7385;
  p_output1[51]=t7244;
  p_output1[52]=t7344;
  p_output1[53]=t7385;
  p_output1[54]=t7244;
  p_output1[55]=t7344;
  p_output1[56]=t7385;
  p_output1[57]=0;
  p_output1[58]=0;
  p_output1[59]=0;
}



void Jdq_AMWorld_RightTarsus_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
