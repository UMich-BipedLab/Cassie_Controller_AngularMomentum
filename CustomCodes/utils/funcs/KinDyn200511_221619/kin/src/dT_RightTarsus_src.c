/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:30:15 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "dT_RightTarsus_src.h"

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
  double t225;
  double t235;
  double t250;
  double t495;
  double t165;
  double t500;
  double t609;
  double t492;
  double t564;
  double t567;
  double t163;
  double t630;
  double t638;
  double t650;
  double t656;
  double t672;
  double t680;
  double t701;
  double t723;
  double t738;
  double t864;
  double t593;
  double t828;
  double t831;
  double t156;
  double t880;
  double t930;
  double t996;
  double t1030;
  double t852;
  double t1003;
  double t1006;
  double t131;
  double t1041;
  double t1081;
  double t1090;
  double t42;
  double t1225;
  double t1217;
  double t1230;
  double t1232;
  double t1259;
  double t1293;
  double t1327;
  double t1246;
  double t1346;
  double t1349;
  double t1376;
  double t1380;
  double t1382;
  double t1360;
  double t1384;
  double t1425;
  double t1465;
  double t1477;
  double t1478;
  double t1156;
  double t1450;
  double t1484;
  double t1486;
  double t1515;
  double t1517;
  double t1520;
  double t1645;
  double t1657;
  double t1677;
  double t1694;
  double t1703;
  double t1753;
  double t1756;
  double t1757;
  double t1784;
  double t1785;
  double t1796;
  double t1812;
  double t1837;
  double t1852;
  double t1860;
  double t1954;
  double t1956;
  double t1994;
  double t2022;
  double t1968;
  double t2027;
  double t2038;
  double t2047;
  double t2054;
  double t2056;
  double t2041;
  double t2065;
  double t2084;
  double t2120;
  double t2124;
  double t2170;
  double t2224;
  double t2227;
  double t2239;
  double t2219;
  double t2253;
  double t2267;
  double t2304;
  double t2328;
  double t2332;
  double t2286;
  double t2340;
  double t2343;
  double t2363;
  double t2364;
  double t2367;
  double t2467;
  double t2480;
  double t2482;
  double t2484;
  double t2498;
  double t2513;
  double t2551;
  double t2509;
  double t2569;
  double t2617;
  double t2623;
  double t2625;
  double t2637;
  double t2640;
  double t2656;
  double t2731;
  double t2732;
  double t2733;
  double t2753;
  double t2755;
  double t2760;
  double t2744;
  double t2775;
  double t2810;
  double t2822;
  double t2827;
  double t2828;
  double t2833;
  double t2840;
  double t2846;
  double t2821;
  double t2862;
  double t2866;
  double t2884;
  double t2897;
  double t2902;
  double t2872;
  double t2904;
  double t2918;
  double t2930;
  double t2932;
  double t2937;
  double t3010;
  double t3012;
  double t3019;
  double t3023;
  double t3036;
  double t3043;
  double t3047;
  double t3048;
  double t3049;
  double t3020;
  double t3061;
  double t3063;
  double t3069;
  double t3076;
  double t3098;
  double t3065;
  double t3114;
  double t3118;
  double t3152;
  double t3170;
  double t3180;
  double t3243;
  double t3246;
  double t3248;
  double t3264;
  double t3266;
  double t3276;
  double t3281;
  double t3284;
  double t3270;
  double t3286;
  double t3294;
  double t3300;
  double t3301;
  double t3308;
  double t3296;
  double t3309;
  double t3313;
  double t3338;
  double t3347;
  double t3365;
  double t3424;
  double t3426;
  double t3428;
  double t3432;
  double t3433;
  double t3437;
  double t3438;
  double t3458;
  double t3459;
  double t3463;
  double t3468;
  double t3474;
  double t3478;
  double t3483;
  double t3490;
  double t3538;
  double t3539;
  double t3543;
  double t3546;
  double t3540;
  double t3557;
  double t3564;
  double t3569;
  double t3578;
  double t3584;
  double t3566;
  double t3588;
  double t3594;
  double t3600;
  double t3603;
  double t3617;
  double t2683;
  double t2684;
  double t2700;
  double t3720;
  double t3724;
  double t3735;
  double t3712;
  double t3743;
  double t3747;
  double t3769;
  double t3778;
  double t3782;
  double t3759;
  double t3785;
  double t3792;
  double t3803;
  double t3807;
  double t3812;
  double t3861;
  double t3864;
  double t3869;
  double t3870;
  double t3874;
  double t3879;
  double t3883;
  double t3878;
  double t3917;
  double t3950;
  double t3952;
  double t3955;
  double t3985;
  double t3991;
  double t3992;
  double t4038;
  double t4047;
  double t4050;
  double t4052;
  double t4056;
  double t4057;
  double t4058;
  double t4065;
  double t4069;
  double t4079;
  double t4088;
  double t4092;
  double t4121;
  double t4125;
  double t4130;
  double t4133;
  double t4134;
  double t4135;
  double t4131;
  double t4136;
  double t4142;
  double t4144;
  double t4145;
  double t4149;
  double t4143;
  double t4158;
  double t4159;
  double t4167;
  double t4170;
  double t4178;
  double t4160;
  double t4179;
  double t4183;
  double t4184;
  double t4186;
  double t4190;
  double t4192;
  double t4199;
  double t4206;
  double t4214;
  double t4225;
  double t4228;
  double t4211;
  double t4243;
  double t4245;
  double t4249;
  double t4251;
  double t4254;
  double t4247;
  double t4255;
  double t4257;
  double t4266;
  double t4268;
  double t4269;
  double t4308;
  double t4309;
  double t4311;
  double t4313;
  double t4314;
  double t4317;
  double t4318;
  double t4315;
  double t4339;
  double t4374;
  double t4378;
  double t4388;
  double t4401;
  double t4407;
  double t4416;
  double t4465;
  double t4467;
  double t4473;
  double t4475;
  double t4480;
  double t4483;
  double t4486;
  double t4494;
  double t4502;
  double t4474;
  double t4504;
  double t4512;
  double t4517;
  double t4520;
  double t4521;
  double t4515;
  double t4523;
  double t4525;
  double t4540;
  double t4543;
  double t4551;
  double t1015;
  double t1116;
  double t1129;
  double t1168;
  double t1184;
  double t1197;
  double t1514;
  double t1523;
  double t1536;
  double t1569;
  double t1584;
  double t1612;
  double t1824;
  double t1864;
  double t1875;
  double t1898;
  double t1919;
  double t1935;
  double t2090;
  double t2176;
  double t2180;
  double t2189;
  double t2190;
  double t2193;
  double t2347;
  double t2370;
  double t2408;
  double t2422;
  double t2445;
  double t2447;
  double t2561;
  double t2562;
  double t2570;
  double t2579;
  double t2627;
  double t2629;
  double t4723;
  double t2925;
  double t2946;
  double t2961;
  double t2974;
  double t2975;
  double t2985;
  double t3128;
  double t3191;
  double t3193;
  double t3206;
  double t3207;
  double t3211;
  double t3315;
  double t3367;
  double t3372;
  double t3376;
  double t3390;
  double t3400;
  double t3476;
  double t3491;
  double t3502;
  double t3513;
  double t3519;
  double t3526;
  double t3599;
  double t3619;
  double t3636;
  double t3656;
  double t3658;
  double t3659;
  double t2701;
  double t2703;
  double t3797;
  double t3815;
  double t3825;
  double t3844;
  double t3846;
  double t3847;
  double t3884;
  double t3900;
  double t3928;
  double t3933;
  double t3957;
  double t3966;
  double t4003;
  double t4004;
  double t4017;
  double t4959;
  double t4077;
  double t4098;
  double t4100;
  double t4103;
  double t4104;
  double t4107;
  double t5001;
  double t5017;
  double t5020;
  double t4264;
  double t4270;
  double t4280;
  double t4289;
  double t4300;
  double t4301;
  double t4319;
  double t4323;
  double t4346;
  double t4347;
  double t4389;
  double t4390;
  double t4428;
  double t4433;
  double t4442;
  double t5081;
  double t4536;
  double t4552;
  double t4555;
  double t4562;
  double t4563;
  double t4572;
  double t5242;
  double t5243;
  double t5253;
  double t5254;
  double t5299;
  double t5301;
  double t5361;
  double t5362;
  double t5416;
  double t5423;
  double t5483;
  double t5488;
  double t4612;
  double t4624;
  double t4626;
  double t5267;
  double t5268;
  double t5273;
  double t5246;
  double t5247;
  double t5248;
  double t5305;
  double t5323;
  double t5324;
  double t5282;
  double t5283;
  double t5285;
  double t5341;
  double t5347;
  double t5348;
  double t5367;
  double t5381;
  double t5389;
  double t5395;
  double t5403;
  double t5406;
  double t5426;
  double t5431;
  double t5438;
  double t5453;
  double t5468;
  double t5469;
  double t5489;
  double t5490;
  double t5494;
  double t5506;
  double t5508;
  double t5511;
  double t4635;
  double t4637;
  double t4644;
  double t5257;
  double t4652;
  double t4655;
  double t4659;
  double t4675;
  double t4687;
  double t4689;
  double t4700;
  double t4703;
  double t4704;
  double t4706;
  double t4707;
  double t4708;
  double t4711;
  double t4745;
  double t4757;
  double t4758;
  double t5979;
  double t5256;
  double t5265;
  double t4776;
  double t4782;
  double t4796;
  double t4808;
  double t4819;
  double t4825;
  double t4855;
  double t4869;
  double t4871;
  double t5644;
  double t5646;
  double t5647;
  double t5651;
  double t5653;
  double t4876;
  double t4879;
  double t4883;
  double t5716;
  double t5718;
  double t5723;
  double t5724;
  double t5725;
  double t4894;
  double t4901;
  double t4902;
  double t5794;
  double t5796;
  double t5798;
  double t5799;
  double t5802;
  double t4908;
  double t4923;
  double t4926;
  double t5873;
  double t5879;
  double t5886;
  double t5903;
  double t5904;
  double t4938;
  double t4941;
  double t4954;
  double t5945;
  double t5946;
  double t5955;
  double t5957;
  double t5958;
  double t4957;
  double t4967;
  double t5994;
  double t6000;
  double t6001;
  double t6004;
  double t6009;
  double t4971;
  double t4972;
  double t6759;
  double t4983;
  double t4989;
  double t4998;
  double t6907;
  double t6910;
  double t6918;
  double t6924;
  double t6927;
  double t6931;
  double t6939;
  double t6945;
  double t6946;
  double t6948;
  double t6953;
  double t6956;
  double t6958;
  double t6959;
  double t6960;
  double t5033;
  double t5038;
  double t5043;
  double t5066;
  double t5068;
  double t5074;
  double t5080;
  double t5084;
  double t5092;
  double t5098;
  double t7086;
  double t5102;
  double t5103;
  double t5104;
  t225 = Cos(var1[3]);
  t235 = Cos(var1[4]);
  t250 = Cos(var1[5]);
  t495 = Sin(var1[13]);
  t165 = Cos(var1[13]);
  t500 = Sin(var1[5]);
  t609 = Cos(var1[15]);
  t492 = t165*t225*t235*t250;
  t564 = -1.*t225*t235*t495*t500;
  t567 = t492 + t564;
  t163 = Sin(var1[15]);
  t630 = Cos(var1[14]);
  t638 = Sin(var1[4]);
  t650 = -1.*t630*t225*t638;
  t656 = Sin(var1[14]);
  t672 = t225*t235*t250*t495;
  t680 = t165*t225*t235*t500;
  t701 = t672 + t680;
  t723 = t656*t701;
  t738 = t650 + t723;
  t864 = Cos(var1[16]);
  t593 = t163*t567;
  t828 = t609*t738;
  t831 = t593 + t828;
  t156 = Sin(var1[16]);
  t880 = t609*t567;
  t930 = -1.*t163*t738;
  t996 = t880 + t930;
  t1030 = Cos(var1[17]);
  t852 = -1.*t156*t831;
  t1003 = t864*t996;
  t1006 = t852 + t1003;
  t131 = Sin(var1[17]);
  t1041 = t864*t831;
  t1081 = t156*t996;
  t1090 = t1041 + t1081;
  t42 = Sin(var1[18]);
  t1225 = Sin(var1[3]);
  t1217 = t225*t250*t638;
  t1230 = t1225*t500;
  t1232 = t1217 + t1230;
  t1259 = t250*t1225;
  t1293 = -1.*t225*t638*t500;
  t1327 = t1259 + t1293;
  t1246 = -1.*t495*t1232;
  t1346 = t165*t1327;
  t1349 = t1246 + t1346;
  t1376 = t165*t1232;
  t1380 = t495*t1327;
  t1382 = t1376 + t1380;
  t1360 = t163*t1349;
  t1384 = t609*t656*t1382;
  t1425 = t1360 + t1384;
  t1465 = t609*t1349;
  t1477 = -1.*t656*t163*t1382;
  t1478 = t1465 + t1477;
  t1156 = Cos(var1[18]);
  t1450 = -1.*t156*t1425;
  t1484 = t864*t1478;
  t1486 = t1450 + t1484;
  t1515 = t864*t1425;
  t1517 = t156*t1478;
  t1520 = t1515 + t1517;
  t1645 = -1.*t225*t235*t656;
  t1657 = t495*t1232;
  t1677 = -1.*t250*t1225;
  t1694 = t225*t638*t500;
  t1703 = t1677 + t1694;
  t1753 = t165*t1703;
  t1756 = t1657 + t1753;
  t1757 = t630*t1756;
  t1784 = t1645 + t1757;
  t1785 = -1.*t864*t163*t1784;
  t1796 = -1.*t609*t156*t1784;
  t1812 = t1785 + t1796;
  t1837 = t609*t864*t1784;
  t1852 = -1.*t163*t156*t1784;
  t1860 = t1837 + t1852;
  t1954 = -1.*t165*t1703;
  t1956 = t1246 + t1954;
  t1994 = -1.*t495*t1703;
  t2022 = t1376 + t1994;
  t1968 = t163*t1956;
  t2027 = t609*t656*t2022;
  t2038 = t1968 + t2027;
  t2047 = t609*t1956;
  t2054 = -1.*t656*t163*t2022;
  t2056 = t2047 + t2054;
  t2041 = -1.*t156*t2038;
  t2065 = t864*t2056;
  t2084 = t2041 + t2065;
  t2120 = t864*t2038;
  t2124 = t156*t2056;
  t2170 = t2120 + t2124;
  t2224 = t630*t225*t235;
  t2227 = t656*t1756;
  t2239 = t2224 + t2227;
  t2219 = -1.*t163*t2022;
  t2253 = -1.*t609*t2239;
  t2267 = t2219 + t2253;
  t2304 = t609*t2022;
  t2328 = -1.*t163*t2239;
  t2332 = t2304 + t2328;
  t2286 = t156*t2267;
  t2340 = t864*t2332;
  t2343 = t2286 + t2340;
  t2363 = t864*t2267;
  t2364 = -1.*t156*t2332;
  t2367 = t2363 + t2364;
  t2467 = t163*t2022;
  t2480 = t609*t2239;
  t2482 = t2467 + t2480;
  t2484 = -1.*t156*t2482;
  t2498 = t2484 + t2340;
  t2513 = -1.*t864*t2482;
  t2551 = t2513 + t2364;
  t2509 = -1.*t131*t2498;
  t2569 = t1030*t2498;
  t2617 = t864*t2482;
  t2623 = t156*t2332;
  t2625 = t2617 + t2623;
  t2637 = -1.*t131*t2625;
  t2640 = t2569 + t2637;
  t2656 = t42*t2640;
  t2731 = -1.*t250*t1225*t638;
  t2732 = t225*t500;
  t2733 = t2731 + t2732;
  t2753 = -1.*t225*t250;
  t2755 = -1.*t1225*t638*t500;
  t2760 = t2753 + t2755;
  t2744 = t165*t2733;
  t2775 = -1.*t495*t2760;
  t2810 = t2744 + t2775;
  t2822 = -1.*t630*t235*t1225;
  t2827 = t495*t2733;
  t2828 = t165*t2760;
  t2833 = t2827 + t2828;
  t2840 = t656*t2833;
  t2846 = t2822 + t2840;
  t2821 = t163*t2810;
  t2862 = t609*t2846;
  t2866 = t2821 + t2862;
  t2884 = t609*t2810;
  t2897 = -1.*t163*t2846;
  t2902 = t2884 + t2897;
  t2872 = -1.*t156*t2866;
  t2904 = t864*t2902;
  t2918 = t2872 + t2904;
  t2930 = t864*t2866;
  t2932 = t156*t2902;
  t2937 = t2930 + t2932;
  t3010 = t165*t235*t250*t1225;
  t3012 = -1.*t235*t495*t1225*t500;
  t3019 = t3010 + t3012;
  t3023 = -1.*t630*t1225*t638;
  t3036 = t235*t250*t495*t1225;
  t3043 = t165*t235*t1225*t500;
  t3047 = t3036 + t3043;
  t3048 = t656*t3047;
  t3049 = t3023 + t3048;
  t3020 = t163*t3019;
  t3061 = t609*t3049;
  t3063 = t3020 + t3061;
  t3069 = t609*t3019;
  t3076 = -1.*t163*t3049;
  t3098 = t3069 + t3076;
  t3065 = -1.*t156*t3063;
  t3114 = t864*t3098;
  t3118 = t3065 + t3114;
  t3152 = t864*t3063;
  t3170 = t156*t3098;
  t3180 = t3152 + t3170;
  t3243 = t250*t1225*t638;
  t3246 = -1.*t225*t500;
  t3248 = t3243 + t3246;
  t3264 = -1.*t495*t3248;
  t3266 = t3264 + t2828;
  t3276 = t165*t3248;
  t3281 = t495*t2760;
  t3284 = t3276 + t3281;
  t3270 = t163*t3266;
  t3286 = t609*t656*t3284;
  t3294 = t3270 + t3286;
  t3300 = t609*t3266;
  t3301 = -1.*t656*t163*t3284;
  t3308 = t3300 + t3301;
  t3296 = -1.*t156*t3294;
  t3309 = t864*t3308;
  t3313 = t3296 + t3309;
  t3338 = t864*t3294;
  t3347 = t156*t3308;
  t3365 = t3338 + t3347;
  t3424 = -1.*t235*t656*t1225;
  t3426 = t495*t3248;
  t3428 = t225*t250;
  t3432 = t1225*t638*t500;
  t3433 = t3428 + t3432;
  t3437 = t165*t3433;
  t3438 = t3426 + t3437;
  t3458 = t630*t3438;
  t3459 = t3424 + t3458;
  t3463 = -1.*t864*t163*t3459;
  t3468 = -1.*t609*t156*t3459;
  t3474 = t3463 + t3468;
  t3478 = t609*t864*t3459;
  t3483 = -1.*t163*t156*t3459;
  t3490 = t3478 + t3483;
  t3538 = -1.*t165*t3433;
  t3539 = t3264 + t3538;
  t3543 = -1.*t495*t3433;
  t3546 = t3276 + t3543;
  t3540 = t163*t3539;
  t3557 = t609*t656*t3546;
  t3564 = t3540 + t3557;
  t3569 = t609*t3539;
  t3578 = -1.*t656*t163*t3546;
  t3584 = t3569 + t3578;
  t3566 = -1.*t156*t3564;
  t3588 = t864*t3584;
  t3594 = t3566 + t3588;
  t3600 = t864*t3564;
  t3603 = t156*t3584;
  t3617 = t3600 + t3603;
  t2683 = t131*t2498;
  t2684 = t1030*t2625;
  t2700 = t2683 + t2684;
  t3720 = t630*t235*t1225;
  t3724 = t656*t3438;
  t3735 = t3720 + t3724;
  t3712 = -1.*t163*t3546;
  t3743 = -1.*t609*t3735;
  t3747 = t3712 + t3743;
  t3769 = t609*t3546;
  t3778 = -1.*t163*t3735;
  t3782 = t3769 + t3778;
  t3759 = t156*t3747;
  t3785 = t864*t3782;
  t3792 = t3759 + t3785;
  t3803 = t864*t3747;
  t3807 = -1.*t156*t3782;
  t3812 = t3803 + t3807;
  t3861 = t163*t3546;
  t3864 = t609*t3735;
  t3869 = t3861 + t3864;
  t3870 = -1.*t156*t3869;
  t3874 = t3870 + t3785;
  t3879 = -1.*t864*t3869;
  t3883 = t3879 + t3807;
  t3878 = -1.*t131*t3874;
  t3917 = t1030*t3874;
  t3950 = t864*t3869;
  t3952 = t156*t3782;
  t3955 = t3950 + t3952;
  t3985 = -1.*t131*t3955;
  t3991 = t3917 + t3985;
  t3992 = t42*t3991;
  t4038 = t656*t638;
  t4047 = t235*t250*t495;
  t4050 = t165*t235*t500;
  t4052 = t4047 + t4050;
  t4056 = t630*t4052;
  t4057 = t4038 + t4056;
  t4058 = -1.*t864*t163*t4057;
  t4065 = -1.*t609*t156*t4057;
  t4069 = t4058 + t4065;
  t4079 = t609*t864*t4057;
  t4088 = -1.*t163*t156*t4057;
  t4092 = t4079 + t4088;
  t4121 = -1.*t235*t250*t495;
  t4125 = -1.*t165*t235*t500;
  t4130 = t4121 + t4125;
  t4133 = t165*t235*t250;
  t4134 = -1.*t235*t495*t500;
  t4135 = t4133 + t4134;
  t4131 = t163*t4130;
  t4136 = t609*t656*t4135;
  t4142 = t4131 + t4136;
  t4144 = t609*t4130;
  t4145 = -1.*t656*t163*t4135;
  t4149 = t4144 + t4145;
  t4143 = -1.*t156*t4142;
  t4158 = t864*t4149;
  t4159 = t4143 + t4158;
  t4167 = t864*t4142;
  t4170 = t156*t4149;
  t4178 = t4167 + t4170;
  t4160 = t131*t4159;
  t4179 = t1030*t4178;
  t4183 = t4160 + t4179;
  t4184 = t42*t4183;
  t4186 = t1030*t4159;
  t4190 = -1.*t131*t4178;
  t4192 = t4186 + t4190;
  t4199 = -1.*t1156*t4192;
  t4206 = t4184 + t4199;
  t4214 = -1.*t630*t638;
  t4225 = t656*t4052;
  t4228 = t4214 + t4225;
  t4211 = -1.*t163*t4135;
  t4243 = -1.*t609*t4228;
  t4245 = t4211 + t4243;
  t4249 = t609*t4135;
  t4251 = -1.*t163*t4228;
  t4254 = t4249 + t4251;
  t4247 = t156*t4245;
  t4255 = t864*t4254;
  t4257 = t4247 + t4255;
  t4266 = t864*t4245;
  t4268 = -1.*t156*t4254;
  t4269 = t4266 + t4268;
  t4308 = t163*t4135;
  t4309 = t609*t4228;
  t4311 = t4308 + t4309;
  t4313 = -1.*t156*t4311;
  t4314 = t4313 + t4255;
  t4317 = -1.*t864*t4311;
  t4318 = t4317 + t4268;
  t4315 = -1.*t131*t4314;
  t4339 = t1030*t4314;
  t4374 = t864*t4311;
  t4378 = t156*t4254;
  t4388 = t4374 + t4378;
  t4401 = -1.*t131*t4388;
  t4407 = t4339 + t4401;
  t4416 = t42*t4407;
  t4465 = -1.*t165*t250*t638;
  t4467 = t495*t638*t500;
  t4473 = t4465 + t4467;
  t4475 = -1.*t630*t235;
  t4480 = -1.*t250*t495*t638;
  t4483 = -1.*t165*t638*t500;
  t4486 = t4480 + t4483;
  t4494 = t656*t4486;
  t4502 = t4475 + t4494;
  t4474 = t163*t4473;
  t4504 = t609*t4502;
  t4512 = t4474 + t4504;
  t4517 = t609*t4473;
  t4520 = -1.*t163*t4502;
  t4521 = t4517 + t4520;
  t4515 = -1.*t156*t4512;
  t4523 = t864*t4521;
  t4525 = t4515 + t4523;
  t4540 = t864*t4512;
  t4543 = t156*t4521;
  t4551 = t4540 + t4543;
  t1015 = t131*t1006;
  t1116 = t1030*t1090;
  t1129 = t1015 + t1116;
  t1168 = t1030*t1006;
  t1184 = -1.*t131*t1090;
  t1197 = t1168 + t1184;
  t1514 = t131*t1486;
  t1523 = t1030*t1520;
  t1536 = t1514 + t1523;
  t1569 = t1030*t1486;
  t1584 = -1.*t131*t1520;
  t1612 = t1569 + t1584;
  t1824 = t131*t1812;
  t1864 = t1030*t1860;
  t1875 = t1824 + t1864;
  t1898 = t1030*t1812;
  t1919 = -1.*t131*t1860;
  t1935 = t1898 + t1919;
  t2090 = t131*t2084;
  t2176 = t1030*t2170;
  t2180 = t2090 + t2176;
  t2189 = t1030*t2084;
  t2190 = -1.*t131*t2170;
  t2193 = t2189 + t2190;
  t2347 = -1.*t131*t2343;
  t2370 = t1030*t2367;
  t2408 = t2347 + t2370;
  t2422 = t1030*t2343;
  t2445 = t131*t2367;
  t2447 = t2422 + t2445;
  t2561 = t1030*t2551;
  t2562 = t2509 + t2561;
  t2570 = t131*t2551;
  t2579 = t2569 + t2570;
  t2627 = -1.*t1030*t2625;
  t2629 = t2509 + t2627;
  t4723 = t1156*t2640;
  t2925 = t131*t2918;
  t2946 = t1030*t2937;
  t2961 = t2925 + t2946;
  t2974 = t1030*t2918;
  t2975 = -1.*t131*t2937;
  t2985 = t2974 + t2975;
  t3128 = t131*t3118;
  t3191 = t1030*t3180;
  t3193 = t3128 + t3191;
  t3206 = t1030*t3118;
  t3207 = -1.*t131*t3180;
  t3211 = t3206 + t3207;
  t3315 = t131*t3313;
  t3367 = t1030*t3365;
  t3372 = t3315 + t3367;
  t3376 = t1030*t3313;
  t3390 = -1.*t131*t3365;
  t3400 = t3376 + t3390;
  t3476 = t131*t3474;
  t3491 = t1030*t3490;
  t3502 = t3476 + t3491;
  t3513 = t1030*t3474;
  t3519 = -1.*t131*t3490;
  t3526 = t3513 + t3519;
  t3599 = t131*t3594;
  t3619 = t1030*t3617;
  t3636 = t3599 + t3619;
  t3656 = t1030*t3594;
  t3658 = -1.*t131*t3617;
  t3659 = t3656 + t3658;
  t2701 = t1156*t2700;
  t2703 = t2701 + t2656;
  t3797 = -1.*t131*t3792;
  t3815 = t1030*t3812;
  t3825 = t3797 + t3815;
  t3844 = t1030*t3792;
  t3846 = t131*t3812;
  t3847 = t3844 + t3846;
  t3884 = t1030*t3883;
  t3900 = t3878 + t3884;
  t3928 = t131*t3883;
  t3933 = t3917 + t3928;
  t3957 = -1.*t1030*t3955;
  t3966 = t3878 + t3957;
  t4003 = t131*t3874;
  t4004 = t1030*t3955;
  t4017 = t4003 + t4004;
  t4959 = t1156*t3991;
  t4077 = t131*t4069;
  t4098 = t1030*t4092;
  t4100 = t4077 + t4098;
  t4103 = t1030*t4069;
  t4104 = -1.*t131*t4092;
  t4107 = t4103 + t4104;
  t5001 = t1156*t4183;
  t5017 = t42*t4192;
  t5020 = t5001 + t5017;
  t4264 = -1.*t131*t4257;
  t4270 = t1030*t4269;
  t4280 = t4264 + t4270;
  t4289 = t1030*t4257;
  t4300 = t131*t4269;
  t4301 = t4289 + t4300;
  t4319 = t1030*t4318;
  t4323 = t4315 + t4319;
  t4346 = t131*t4318;
  t4347 = t4339 + t4346;
  t4389 = -1.*t1030*t4388;
  t4390 = t4315 + t4389;
  t4428 = t131*t4314;
  t4433 = t1030*t4388;
  t4442 = t4428 + t4433;
  t5081 = t1156*t4407;
  t4536 = t131*t4525;
  t4552 = t1030*t4551;
  t4555 = t4536 + t4552;
  t4562 = t1030*t4525;
  t4563 = -1.*t131*t4551;
  t4572 = t4562 + t4563;
  t5242 = -1.*t165;
  t5243 = 1. + t5242;
  t5253 = -1.*t630;
  t5254 = 1. + t5253;
  t5299 = -1.*t609;
  t5301 = 1. + t5299;
  t5361 = -1.*t864;
  t5362 = 1. + t5361;
  t5416 = -1.*t1030;
  t5423 = 1. + t5416;
  t5483 = -1.*t1156;
  t5488 = 1. + t5483;
  t4612 = t1156*t1129;
  t4624 = t42*t1197;
  t4626 = t4612 + t4624;
  t5267 = -0.135*t5243;
  t5268 = 0.07996*t495;
  t5273 = 0. + t5267 + t5268;
  t5246 = 0.07996*t5243;
  t5247 = 0.135*t495;
  t5248 = 0. + t5246 + t5247;
  t5305 = -0.01004*t5301;
  t5323 = 0.08055*t163;
  t5324 = 0. + t5305 + t5323;
  t5282 = -0.135*t5254;
  t5283 = 0.08055*t656;
  t5285 = 0. + t5282 + t5283;
  t5341 = -0.08055*t5301;
  t5347 = -0.01004*t163;
  t5348 = 0. + t5341 + t5347;
  t5367 = -0.08055*t5362;
  t5381 = -0.13004*t156;
  t5389 = 0. + t5367 + t5381;
  t5395 = -0.13004*t5362;
  t5403 = 0.08055*t156;
  t5406 = 0. + t5395 + t5403;
  t5426 = -0.19074*t5423;
  t5431 = 0.03315*t131;
  t5438 = 0. + t5426 + t5431;
  t5453 = -0.03315*t5423;
  t5468 = -0.19074*t131;
  t5469 = 0. + t5453 + t5468;
  t5489 = -0.01315*t5488;
  t5490 = -0.62554*t42;
  t5494 = 0. + t5489 + t5490;
  t5506 = -0.62554*t5488;
  t5508 = 0.01315*t42;
  t5511 = 0. + t5506 + t5508;
  t4635 = t1156*t1536;
  t4637 = t42*t1612;
  t4644 = t4635 + t4637;
  t5257 = -0.135*t656;
  t4652 = t1156*t1875;
  t4655 = t42*t1935;
  t4659 = t4652 + t4655;
  t4675 = t1156*t2180;
  t4687 = t42*t2193;
  t4689 = t4675 + t4687;
  t4700 = t42*t2408;
  t4703 = t1156*t2447;
  t4704 = t4700 + t4703;
  t4706 = t42*t2562;
  t4707 = t1156*t2579;
  t4708 = t4706 + t4707;
  t4711 = t42*t2629;
  t4745 = t4711 + t4723;
  t4757 = -1.*t42*t2700;
  t4758 = t4757 + t4723;
  t5979 = -1.*t42*t2640;
  t5256 = -0.08055*t5254;
  t5265 = 0. + t5256 + t5257;
  t4776 = t1156*t2961;
  t4782 = t42*t2985;
  t4796 = t4776 + t4782;
  t4808 = t1156*t3193;
  t4819 = t42*t3211;
  t4825 = t4808 + t4819;
  t4855 = t1156*t3372;
  t4869 = t42*t3400;
  t4871 = t4855 + t4869;
  t5644 = -0.135*t630;
  t5646 = -0.08055*t656;
  t5647 = t5644 + t5646;
  t5651 = 0.08055*t630;
  t5653 = t5651 + t5257;
  t4876 = t1156*t3502;
  t4879 = t42*t3526;
  t4883 = t4876 + t4879;
  t5716 = 0.135*t165;
  t5718 = t5716 + t5268;
  t5723 = 0.07996*t165;
  t5724 = -0.135*t495;
  t5725 = t5723 + t5724;
  t4894 = t1156*t3636;
  t4901 = t42*t3659;
  t4902 = t4894 + t4901;
  t5794 = 0.08055*t609;
  t5796 = t5794 + t5347;
  t5798 = -0.01004*t609;
  t5799 = -0.08055*t163;
  t5802 = t5798 + t5799;
  t4908 = t42*t3825;
  t4923 = t1156*t3847;
  t4926 = t4908 + t4923;
  t5873 = -0.13004*t864;
  t5879 = -0.08055*t156;
  t5886 = t5873 + t5879;
  t5903 = 0.08055*t864;
  t5904 = t5903 + t5381;
  t4938 = t42*t3900;
  t4941 = t1156*t3933;
  t4954 = t4938 + t4941;
  t5945 = 0.03315*t1030;
  t5946 = t5945 + t5468;
  t5955 = -0.19074*t1030;
  t5957 = -0.03315*t131;
  t5958 = t5955 + t5957;
  t4957 = t42*t3966;
  t4967 = t4957 + t4959;
  t5994 = -0.62554*t1156;
  t6000 = -0.01315*t42;
  t6001 = t5994 + t6000;
  t6004 = 0.01315*t1156;
  t6009 = t6004 + t5490;
  t4971 = -1.*t42*t4017;
  t4972 = t4971 + t4959;
  t6759 = -1.*t42*t3991;
  t4983 = t1156*t4100;
  t4989 = t42*t4107;
  t4998 = t4983 + t4989;
  t6907 = t5324*t4130;
  t6910 = -0.1318*t630*t4135;
  t6918 = t5285*t4135;
  t6924 = t656*t5348*t4135;
  t6927 = t5389*t4142;
  t6931 = t5406*t4149;
  t6939 = t5438*t4159;
  t6945 = t5469*t4178;
  t6946 = t5494*t4183;
  t6948 = t5511*t4192;
  t6953 = -1.*t42*t4183;
  t6956 = t1156*t4192;
  t6958 = t6953 + t6956;
  t6959 = -0.73604*t6958;
  t6960 = -0.04375*t5020;
  t5033 = t42*t4280;
  t5038 = t1156*t4301;
  t5043 = t5033 + t5038;
  t5066 = t42*t4323;
  t5068 = t1156*t4347;
  t5074 = t5066 + t5068;
  t5080 = t42*t4390;
  t5084 = t5080 + t5081;
  t5092 = -1.*t42*t4442;
  t5098 = t5092 + t5081;
  t7086 = -1.*t42*t4407;
  t5102 = t1156*t4555;
  t5103 = t42*t4572;
  t5104 = t5102 + t5103;
  p_output1[0]=(-1.*t1156*t2985 + t2961*t42)*var2[3] + (-1.*t1156*t1197 + t1129*t42)*var2[4] + (-1.*t1156*t1612 + t1536*t42)*var2[5] + (-1.*t1156*t2193 + t2180*t42)*var2[13] + (-1.*t1156*t1935 + t1875*t42)*var2[14] + (-1.*t1156*t2408 + t2447*t42)*var2[15] + (-1.*t1156*t2562 + t2579*t42)*var2[16] + (-1.*t1156*t2629 + t2656)*var2[17] + t2703*var2[18];
  p_output1[1]=(-1.*t1156*t2640 + t2700*t42)*var2[3] + (-1.*t1156*t3211 + t3193*t42)*var2[4] + (-1.*t1156*t3400 + t3372*t42)*var2[5] + (-1.*t1156*t3659 + t3636*t42)*var2[13] + (-1.*t1156*t3526 + t3502*t42)*var2[14] + (-1.*t1156*t3825 + t3847*t42)*var2[15] + (-1.*t1156*t3900 + t3933*t42)*var2[16] + (-1.*t1156*t3966 + t3992)*var2[17] + (t3992 + t1156*t4017)*var2[18];
  p_output1[2]=(t42*t4555 - 1.*t1156*t4572)*var2[4] + t4206*var2[5] + t4206*var2[13] + (-1.*t1156*t4107 + t4100*t42)*var2[14] + (-1.*t1156*t4280 + t42*t4301)*var2[15] + (-1.*t1156*t4323 + t42*t4347)*var2[16] + (-1.*t1156*t4390 + t4416)*var2[17] + (t4416 + t1156*t4442)*var2[18];
  p_output1[3]=0;
  p_output1[4]=t4796*var2[3] + t4626*var2[4] + t4644*var2[5] + t4689*var2[13] + t4659*var2[14] + t4704*var2[15] + t4708*var2[16] + t4745*var2[17] + t4758*var2[18];
  p_output1[5]=t2703*var2[3] + t4825*var2[4] + t4871*var2[5] + t4902*var2[13] + t4883*var2[14] + t4926*var2[15] + t4954*var2[16] + t4967*var2[17] + t4972*var2[18];
  p_output1[6]=t5104*var2[4] + t5020*var2[5] + t5020*var2[13] + t4998*var2[14] + t5043*var2[15] + t5074*var2[16] + t5084*var2[17] + t5098*var2[18];
  p_output1[7]=0;
  p_output1[8]=(t3424 - 1.*t2833*t630)*var2[3] + (-1.*t225*t638*t656 - 1.*t630*t701)*var2[4] - 1.*t1382*t630*var2[5] - 1.*t2022*t630*var2[13] + t2239*var2[14];
  p_output1[9]=(-1.*t1756*t630 + t225*t235*t656)*var2[3] + (-1.*t3047*t630 - 1.*t1225*t638*t656)*var2[4] - 1.*t3284*t630*var2[5] - 1.*t3546*t630*var2[13] + t3735*var2[14];
  p_output1[10]=(-1.*t4486*t630 - 1.*t235*t656)*var2[4] - 1.*t4135*t630*var2[5] - 1.*t4135*t630*var2[13] + t4228*var2[14];
  p_output1[11]=0;
  p_output1[12]=var2[0] + (-0.73604*(t1156*t2985 - 1.*t2961*t42) - 0.04375*t4796 + t2733*t5248 - 1.*t1225*t235*t5265 + t2760*t5273 + t2833*t5285 + t2810*t5324 + t2846*t5348 + t2866*t5389 + t2902*t5406 + t2918*t5438 + t2937*t5469 + t2961*t5494 + t2985*t5511 - 0.1318*(t2833*t630 + t1225*t235*t656))*var2[3] + (-0.73604*(t1156*t1197 - 1.*t1129*t42) - 0.04375*t4626 + t225*t235*t250*t5248 + t225*t235*t500*t5273 + t1006*t5438 + t1090*t5469 + t1129*t5494 + t1197*t5511 + t5324*t567 - 1.*t225*t5265*t638 + t5285*t701 - 0.1318*(t225*t638*t656 + t630*t701) + t5348*t738 + t5389*t831 + t5406*t996)*var2[4] + (-0.73604*(t1156*t1612 - 1.*t1536*t42) - 0.04375*t4644 + t1327*t5248 + t1232*t5273 + t1382*t5285 + t1349*t5324 + t1425*t5389 + t1478*t5406 + t1486*t5438 + t1520*t5469 + t1536*t5494 + t1612*t5511 - 0.1318*t1382*t630 + t1382*t5348*t656)*var2[5] + (-0.73604*(t1156*t2193 - 1.*t2180*t42) - 0.04375*t4689 + t2022*t5285 + t1956*t5324 + t2038*t5389 + t2056*t5406 + t2084*t5438 + t2170*t5469 + t2180*t5494 + t2193*t5511 + t1232*t5718 + t1703*t5725 - 0.1318*t2022*t630 + t2022*t5348*t656)*var2[13] + (-0.73604*(t1156*t1935 - 1.*t1875*t42) - 0.04375*t4659 + t1784*t5348 - 1.*t163*t1784*t5406 + t1812*t5438 + t1860*t5469 + t1875*t5494 + t1935*t5511 + t225*t235*t5647 + t1756*t5653 + t1784*t5389*t609 - 0.1318*(-1.*t225*t235*t630 - 1.*t1756*t656))*var2[14] + (-0.73604*(t1156*t2408 - 1.*t2447*t42) - 0.04375*t4704 + t2332*t5389 + t2267*t5406 + t2367*t5438 + t2343*t5469 + t2447*t5494 + t2408*t5511 + t2022*t5796 + t2239*t5802)*var2[15] + (-0.73604*(t1156*t2562 - 1.*t2579*t42) - 0.04375*t4708 + t2551*t5438 + t2498*t5469 + t2579*t5494 + t2562*t5511 + t2482*t5886 + t2332*t5904)*var2[16] + (-0.04375*t4745 + t2640*t5494 + t2629*t5511 + t2498*t5946 + t2625*t5958 - 0.73604*(t1156*t2629 + t5979))*var2[17] + (-0.04375*t4758 - 0.73604*(-1.*t1156*t2700 + t5979) + t2700*t6001 + t2640*t6009)*var2[18];
  p_output1[13]=var2[1] + (-0.1318*t1784 - 0.04375*t2703 - 0.73604*t4758 + t1232*t5248 + t225*t235*t5265 + t1703*t5273 + t1756*t5285 + t2022*t5324 + t2239*t5348 + t2482*t5389 + t2332*t5406 + t2498*t5438 + t2625*t5469 + t2700*t5494 + t2640*t5511)*var2[3] + (-0.73604*(t1156*t3211 - 1.*t3193*t42) - 0.04375*t4825 + t1225*t235*t250*t5248 + t1225*t235*t500*t5273 + t3047*t5285 + t3019*t5324 + t3049*t5348 + t3063*t5389 + t3098*t5406 + t3118*t5438 + t3180*t5469 + t3193*t5494 + t3211*t5511 - 1.*t1225*t5265*t638 - 0.1318*(t3047*t630 + t1225*t638*t656))*var2[4] + (-0.73604*(t1156*t3400 - 1.*t3372*t42) - 0.04375*t4871 + t2760*t5248 + t3248*t5273 + t3284*t5285 + t3266*t5324 + t3294*t5389 + t3308*t5406 + t3313*t5438 + t3365*t5469 + t3372*t5494 + t3400*t5511 - 0.1318*t3284*t630 + t3284*t5348*t656)*var2[5] + (-0.73604*(t1156*t3659 - 1.*t3636*t42) - 0.04375*t4902 + t3546*t5285 + t3539*t5324 + t3564*t5389 + t3584*t5406 + t3594*t5438 + t3617*t5469 + t3636*t5494 + t3659*t5511 + t3248*t5718 + t3433*t5725 - 0.1318*t3546*t630 + t3546*t5348*t656)*var2[13] + (-0.73604*(t1156*t3526 - 1.*t3502*t42) - 0.04375*t4883 + t3459*t5348 - 1.*t163*t3459*t5406 + t3474*t5438 + t3490*t5469 + t3502*t5494 + t3526*t5511 + t1225*t235*t5647 + t3438*t5653 + t3459*t5389*t609 - 0.1318*(t2822 - 1.*t3438*t656))*var2[14] + (-0.73604*(t1156*t3825 - 1.*t3847*t42) - 0.04375*t4926 + t3782*t5389 + t3747*t5406 + t3812*t5438 + t3792*t5469 + t3847*t5494 + t3825*t5511 + t3546*t5796 + t3735*t5802)*var2[15] + (-0.73604*(t1156*t3900 - 1.*t3933*t42) - 0.04375*t4954 + t3883*t5438 + t3874*t5469 + t3933*t5494 + t3900*t5511 + t3869*t5886 + t3782*t5904)*var2[16] + (-0.04375*t4967 + t3991*t5494 + t3966*t5511 + t3874*t5946 + t3955*t5958 - 0.73604*(t1156*t3966 + t6759))*var2[17] + (-0.04375*t4972 + t4017*t6001 + t3991*t6009 - 0.73604*(-1.*t1156*t4017 + t6759))*var2[18];
  p_output1[14]=var2[2] + (-0.73604*(-1.*t42*t4555 + t1156*t4572) - 0.04375*t5104 - 1.*t235*t5265 + t4486*t5285 + t4473*t5324 + t4502*t5348 + t4512*t5389 + t4521*t5406 + t4525*t5438 + t4551*t5469 + t4555*t5494 + t4572*t5511 - 1.*t250*t5248*t638 - 1.*t500*t5273*t638 - 0.1318*(t4486*t630 + t235*t656))*var2[4] + (-1.*t235*t500*t5248 + t235*t250*t5273 + t6907 + t6910 + t6918 + t6924 + t6927 + t6931 + t6939 + t6945 + t6946 + t6948 + t6959 + t6960)*var2[5] + (t235*t250*t5718 + t235*t500*t5725 + t6907 + t6910 + t6918 + t6924 + t6927 + t6931 + t6939 + t6945 + t6946 + t6948 + t6959 + t6960)*var2[13] + (-0.73604*(t1156*t4107 - 1.*t4100*t42) - 0.04375*t4998 + t4057*t5348 - 1.*t163*t4057*t5406 + t4069*t5438 + t4092*t5469 + t4100*t5494 + t4107*t5511 + t4052*t5653 + t4057*t5389*t609 - 1.*t5647*t638 - 0.1318*(t630*t638 - 1.*t4052*t656))*var2[14] + (-0.73604*(t1156*t4280 - 1.*t42*t4301) - 0.04375*t5043 + t4254*t5389 + t4245*t5406 + t4269*t5438 + t4257*t5469 + t4301*t5494 + t4280*t5511 + t4135*t5796 + t4228*t5802)*var2[15] + (-0.73604*(t1156*t4323 - 1.*t42*t4347) - 0.04375*t5074 + t4318*t5438 + t4314*t5469 + t4347*t5494 + t4323*t5511 + t4311*t5886 + t4254*t5904)*var2[16] + (-0.04375*t5084 + t4407*t5494 + t4390*t5511 + t4314*t5946 + t4388*t5958 - 0.73604*(t1156*t4390 + t7086))*var2[17] + (-0.04375*t5098 + t4442*t6001 + t4407*t6009 - 0.73604*(-1.*t1156*t4442 + t7086))*var2[18];
  p_output1[15]=0;
}



void dT_RightTarsus_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
