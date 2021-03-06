/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:26:20 GMT-04:00
 */

#ifdef MATLAB_MEX_FILE
#include <stdexcept>
#include <cmath>
#include<math.h>
/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
inline double Power(double x, double y) { return pow(x, y); }
inline double Sqrt(double x) { return sqrt(x); }

inline double Abs(double x) { return fabs(x); }

inline double Exp(double x) { return exp(x); }
inline double Log(double x) { return log(x); }

inline double Sin(double x) { return sin(x); }
inline double Cos(double x) { return cos(x); }
inline double Tan(double x) { return tan(x); }

inline double ArcSin(double x) { return asin(x); }
inline double ArcCos(double x) { return acos(x); }
inline double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
inline double ArcTan(double x, double y) { return atan2(y,x); }

inline double Sinh(double x) { return sinh(x); }
inline double Cosh(double x) { return cosh(x); }
inline double Tanh(double x) { return tanh(x); }

const double E	= 2.71828182845904523536029;
const double Pi = 3.14159265358979323846264;
const double Degree = 0.01745329251994329576924;

inline double Sec(double x) { return 1/cos(x); }
inline double Csc(double x) { return 1/sin(x); }

#endif

/*
 * Sub functions
 */
static void output1(double *p_output1,const double *var1,const double *var2)
{
  double t573;
  double t498;
  double t587;
  double t544;
  double t588;
  double t408;
  double t468;
  double t569;
  double t605;
  double t631;
  double t652;
  double t692;
  double t721;
  double t752;
  double t798;
  double t817;
  double t860;
  double t872;
  double t884;
  double t887;
  double t928;
  double t960;
  double t1001;
  double t364;
  double t382;
  double t1022;
  double t1105;
  double t961;
  double t1062;
  double t1088;
  double t326;
  double t1139;
  double t1149;
  double t1182;
  double t20;
  double t1403;
  double t1415;
  double t1416;
  double t1470;
  double t1472;
  double t1508;
  double t1392;
  double t1438;
  double t1463;
  double t1465;
  double t1512;
  double t1532;
  double t1546;
  double t1587;
  double t1590;
  double t1287;
  double t1534;
  double t1595;
  double t1596;
  double t1631;
  double t1632;
  double t1634;
  double t1770;
  double t1771;
  double t1736;
  double t1748;
  double t1750;
  double t1777;
  double t1780;
  double t1786;
  double t1789;
  double t1790;
  double t1784;
  double t1812;
  double t1822;
  double t1842;
  double t1843;
  double t1847;
  double t2061;
  double t2077;
  double t2080;
  double t1963;
  double t1974;
  double t1981;
  double t1984;
  double t2012;
  double t2034;
  double t2053;
  double t2085;
  double t2109;
  double t2144;
  double t2158;
  double t2170;
  double t2132;
  double t2179;
  double t2183;
  double t2188;
  double t2192;
  double t2196;
  double t2305;
  double t2322;
  double t2324;
  double t2273;
  double t2280;
  double t2281;
  double t2366;
  double t2380;
  double t2392;
  double t2268;
  double t2293;
  double t2325;
  double t2340;
  double t2344;
  double t2346;
  double t2363;
  double t2395;
  double t2402;
  double t2404;
  double t2406;
  double t2409;
  double t2403;
  double t2414;
  double t2428;
  double t2475;
  double t2484;
  double t2485;
  double t2545;
  double t2557;
  double t2561;
  double t2577;
  double t2583;
  double t2593;
  double t2604;
  double t2621;
  double t2635;
  double t2597;
  double t2638;
  double t2639;
  double t2652;
  double t2654;
  double t2655;
  double t2693;
  double t2694;
  double t2700;
  double t2719;
  double t2722;
  double t2728;
  double t2731;
  double t2723;
  double t2749;
  double t2782;
  double t2787;
  double t2794;
  double t2814;
  double t2819;
  double t2825;
  double t2875;
  double t2879;
  double t2886;
  double t2888;
  double t2892;
  double t2907;
  double t2913;
  double t2923;
  double t2925;
  double t2937;
  double t2939;
  double t2940;
  double t2952;
  double t2953;
  double t2976;
  double t2984;
  double t3000;
  double t3019;
  double t3070;
  double t3076;
  double t3053;
  double t3064;
  double t3065;
  double t3068;
  double t3079;
  double t3081;
  double t3085;
  double t3092;
  double t3100;
  double t3083;
  double t3101;
  double t3103;
  double t3105;
  double t3106;
  double t3107;
  double t3188;
  double t3196;
  double t3161;
  double t3177;
  double t3182;
  double t3205;
  double t3207;
  double t3215;
  double t3224;
  double t3225;
  double t3211;
  double t3226;
  double t3227;
  double t3234;
  double t3244;
  double t3249;
  double t3325;
  double t3332;
  double t3335;
  double t3289;
  double t3296;
  double t3297;
  double t3298;
  double t3299;
  double t3302;
  double t3314;
  double t3338;
  double t3341;
  double t3351;
  double t3352;
  double t3354;
  double t3343;
  double t3360;
  double t3365;
  double t3374;
  double t3378;
  double t3381;
  double t3432;
  double t3445;
  double t3452;
  double t3454;
  double t3456;
  double t3459;
  double t3463;
  double t3464;
  double t3474;
  double t3460;
  double t3475;
  double t3483;
  double t3492;
  double t3497;
  double t3501;
  double t3551;
  double t3552;
  double t3558;
  double t3566;
  double t3567;
  double t3571;
  double t3572;
  double t3568;
  double t3584;
  double t3627;
  double t3632;
  double t3638;
  double t3659;
  double t3660;
  double t3666;
  double t2841;
  double t2845;
  double t2855;
  double t3723;
  double t3738;
  double t3739;
  double t3746;
  double t3747;
  double t3749;
  double t3750;
  double t3753;
  double t3756;
  double t3759;
  double t3760;
  double t3762;
  double t3835;
  double t3836;
  double t3839;
  double t3823;
  double t3827;
  double t3830;
  double t3833;
  double t3840;
  double t3844;
  double t3859;
  double t3860;
  double t3867;
  double t3852;
  double t3870;
  double t3872;
  double t3880;
  double t3885;
  double t3887;
  double t3879;
  double t3888;
  double t3895;
  double t3897;
  double t3901;
  double t3905;
  double t3909;
  double t3912;
  double t3916;
  double t3933;
  double t3937;
  double t3952;
  double t3955;
  double t3957;
  double t3958;
  double t3972;
  double t3977;
  double t3983;
  double t3967;
  double t3986;
  double t3990;
  double t3998;
  double t4007;
  double t4010;
  double t4052;
  double t4053;
  double t4056;
  double t4058;
  double t4061;
  double t4080;
  double t4082;
  double t4063;
  double t4103;
  double t4127;
  double t4129;
  double t4131;
  double t4160;
  double t4163;
  double t4164;
  double t4218;
  double t4221;
  double t4227;
  double t4195;
  double t4199;
  double t4204;
  double t4211;
  double t4212;
  double t4215;
  double t4216;
  double t4229;
  double t4232;
  double t4237;
  double t4238;
  double t4240;
  double t4236;
  double t4243;
  double t4244;
  double t4254;
  double t4255;
  double t4260;
  double t1096;
  double t1195;
  double t1213;
  double t1296;
  double t1308;
  double t1332;
  double t1608;
  double t1643;
  double t1648;
  double t1663;
  double t1673;
  double t1687;
  double t1826;
  double t1862;
  double t1866;
  double t1894;
  double t1896;
  double t1910;
  double t2187;
  double t2199;
  double t2200;
  double t2205;
  double t2222;
  double t2230;
  double t2469;
  double t2509;
  double t2515;
  double t2523;
  double t2528;
  double t2529;
  double t2650;
  double t2661;
  double t2663;
  double t2670;
  double t2671;
  double t2673;
  double t2732;
  double t2733;
  double t2755;
  double t2756;
  double t2796;
  double t2802;
  double t4378;
  double t2978;
  double t3022;
  double t3025;
  double t3030;
  double t3036;
  double t3038;
  double t3104;
  double t3111;
  double t3113;
  double t3117;
  double t3120;
  double t3124;
  double t3233;
  double t3250;
  double t3251;
  double t3262;
  double t3263;
  double t3266;
  double t3370;
  double t3390;
  double t3399;
  double t3405;
  double t3409;
  double t3411;
  double t3487;
  double t3505;
  double t3508;
  double t3528;
  double t3529;
  double t3530;
  double t3578;
  double t3579;
  double t3590;
  double t3610;
  double t3641;
  double t3653;
  double t3677;
  double t3683;
  double t3690;
  double t4500;
  double t2858;
  double t2862;
  double t3757;
  double t3771;
  double t3776;
  double t3787;
  double t3796;
  double t3807;
  double t4543;
  double t4551;
  double t4553;
  double t3996;
  double t4011;
  double t4014;
  double t4022;
  double t4028;
  double t4035;
  double t4089;
  double t4096;
  double t4105;
  double t4106;
  double t4135;
  double t4137;
  double t4175;
  double t4176;
  double t4179;
  double t4599;
  double t4249;
  double t4264;
  double t4267;
  double t4269;
  double t4274;
  double t4276;
  double t4765;
  double t4769;
  double t4834;
  double t4836;
  double t4856;
  double t4858;
  double t4294;
  double t4296;
  double t4297;
  double t4915;
  double t4916;
  double t4797;
  double t4798;
  double t4808;
  double t4809;
  double t4812;
  double t4771;
  double t4772;
  double t4773;
  double t4816;
  double t4821;
  double t4829;
  double t4838;
  double t4840;
  double t4842;
  double t4844;
  double t4849;
  double t4850;
  double t4859;
  double t4861;
  double t4864;
  double t4877;
  double t4882;
  double t4884;
  double t4300;
  double t4301;
  double t4303;
  double t4936;
  double t4949;
  double t4956;
  double t4960;
  double t4962;
  double t4968;
  double t4979;
  double t4983;
  double t4990;
  double t4310;
  double t4311;
  double t4313;
  double t4917;
  double t4918;
  double t4921;
  double t4932;
  double t4938;
  double t4753;
  double t4316;
  double t4317;
  double t4321;
  double t5118;
  double t5119;
  double t4328;
  double t4333;
  double t4334;
  double t4338;
  double t4342;
  double t4343;
  double t4350;
  double t4359;
  double t4365;
  double t4377;
  double t4384;
  double t4396;
  double t4397;
  double t5456;
  double t4742;
  double t4743;
  double t4748;
  double t4752;
  double t4760;
  double t4406;
  double t4416;
  double t4422;
  double t4427;
  double t4428;
  double t4433;
  double t5028;
  double t5034;
  double t5039;
  double t5041;
  double t5046;
  double t4443;
  double t4444;
  double t4446;
  double t4454;
  double t4455;
  double t4457;
  double t5294;
  double t5297;
  double t5303;
  double t5305;
  double t5307;
  double t4466;
  double t4469;
  double t4473;
  double t5365;
  double t5369;
  double t5373;
  double t5377;
  double t5378;
  double t4478;
  double t4484;
  double t4485;
  double t5416;
  double t5417;
  double t5421;
  double t5432;
  double t5433;
  double t4494;
  double t4501;
  double t5469;
  double t5475;
  double t5479;
  double t5489;
  double t5491;
  double t4504;
  double t4506;
  double t5904;
  double t4528;
  double t4532;
  double t4533;
  double t6116;
  double t6120;
  double t6121;
  double t6123;
  double t6126;
  double t6127;
  double t6129;
  double t6130;
  double t6133;
  double t6139;
  double t6140;
  double t6141;
  double t6143;
  double t6144;
  double t6146;
  double t4564;
  double t4567;
  double t4571;
  double t4577;
  double t4580;
  double t4581;
  double t4589;
  double t4601;
  double t4605;
  double t4606;
  double t6242;
  double t4608;
  double t4611;
  double t4613;
  t573 = Cos(var1[3]);
  t498 = Cos(var1[5]);
  t587 = Sin(var1[4]);
  t544 = Sin(var1[3]);
  t588 = Sin(var1[5]);
  t408 = Cos(var1[7]);
  t468 = Cos(var1[6]);
  t569 = -1.*t498*t544;
  t605 = t573*t587*t588;
  t631 = t569 + t605;
  t652 = t468*t631;
  t692 = t573*t498*t587;
  t721 = t544*t588;
  t752 = t692 + t721;
  t798 = Sin(var1[6]);
  t817 = t752*t798;
  t860 = t652 + t817;
  t872 = t408*t860;
  t884 = Cos(var1[4]);
  t887 = Sin(var1[7]);
  t928 = -1.*t573*t884*t887;
  t960 = t872 + t928;
  t1001 = Cos(var1[9]);
  t364 = Cos(var1[8]);
  t382 = Sin(var1[9]);
  t1022 = Sin(var1[8]);
  t1105 = Cos(var1[10]);
  t961 = -1.*t364*t382*t960;
  t1062 = -1.*t1001*t960*t1022;
  t1088 = t961 + t1062;
  t326 = Sin(var1[10]);
  t1139 = t1001*t364*t960;
  t1149 = -1.*t382*t960*t1022;
  t1182 = t1139 + t1149;
  t20 = Sin(var1[11]);
  t1403 = t498*t544;
  t1415 = -1.*t573*t587*t588;
  t1416 = t1403 + t1415;
  t1470 = t468*t1416;
  t1472 = -1.*t752*t798;
  t1508 = t1470 + t1472;
  t1392 = t468*t752;
  t1438 = t1416*t798;
  t1463 = t1392 + t1438;
  t1465 = t364*t1463*t887;
  t1512 = t1508*t1022;
  t1532 = t1465 + t1512;
  t1546 = t364*t1508;
  t1587 = -1.*t1463*t887*t1022;
  t1590 = t1546 + t1587;
  t1287 = Cos(var1[11]);
  t1534 = -1.*t382*t1532;
  t1595 = t1001*t1590;
  t1596 = t1534 + t1595;
  t1631 = t1001*t1532;
  t1632 = t382*t1590;
  t1634 = t1631 + t1632;
  t1770 = -1.*t468*t631;
  t1771 = t1770 + t1472;
  t1736 = -1.*t631*t798;
  t1748 = t1392 + t1736;
  t1750 = t364*t1748*t887;
  t1777 = t1771*t1022;
  t1780 = t1750 + t1777;
  t1786 = t364*t1771;
  t1789 = -1.*t1748*t887*t1022;
  t1790 = t1786 + t1789;
  t1784 = -1.*t382*t1780;
  t1812 = t1001*t1790;
  t1822 = t1784 + t1812;
  t1842 = t1001*t1780;
  t1843 = t382*t1790;
  t1847 = t1842 + t1843;
  t2061 = t573*t884*t498*t468;
  t2077 = -1.*t573*t884*t588*t798;
  t2080 = t2061 + t2077;
  t1963 = -1.*t573*t408*t587;
  t1974 = t573*t884*t468*t588;
  t1981 = t573*t884*t498*t798;
  t1984 = t1974 + t1981;
  t2012 = t1984*t887;
  t2034 = t1963 + t2012;
  t2053 = t364*t2034;
  t2085 = t2080*t1022;
  t2109 = t2053 + t2085;
  t2144 = t364*t2080;
  t2158 = -1.*t2034*t1022;
  t2170 = t2144 + t2158;
  t2132 = -1.*t382*t2109;
  t2179 = t1001*t2170;
  t2183 = t2132 + t2179;
  t2188 = t1001*t2109;
  t2192 = t382*t2170;
  t2196 = t2188 + t2192;
  t2305 = -1.*t498*t544*t587;
  t2322 = t573*t588;
  t2324 = t2305 + t2322;
  t2273 = -1.*t573*t498;
  t2280 = -1.*t544*t587*t588;
  t2281 = t2273 + t2280;
  t2366 = t468*t2324;
  t2380 = -1.*t2281*t798;
  t2392 = t2366 + t2380;
  t2268 = -1.*t884*t408*t544;
  t2293 = t468*t2281;
  t2325 = t2324*t798;
  t2340 = t2293 + t2325;
  t2344 = t2340*t887;
  t2346 = t2268 + t2344;
  t2363 = t364*t2346;
  t2395 = t2392*t1022;
  t2402 = t2363 + t2395;
  t2404 = t364*t2392;
  t2406 = -1.*t2346*t1022;
  t2409 = t2404 + t2406;
  t2403 = -1.*t382*t2402;
  t2414 = t1001*t2409;
  t2428 = t2403 + t2414;
  t2475 = t1001*t2402;
  t2484 = t382*t2409;
  t2485 = t2475 + t2484;
  t2545 = t573*t884*t408;
  t2557 = t860*t887;
  t2561 = t2545 + t2557;
  t2577 = -1.*t364*t2561;
  t2583 = -1.*t1748*t1022;
  t2593 = t2577 + t2583;
  t2604 = t364*t1748;
  t2621 = -1.*t2561*t1022;
  t2635 = t2604 + t2621;
  t2597 = t382*t2593;
  t2638 = t1001*t2635;
  t2639 = t2597 + t2638;
  t2652 = t1001*t2593;
  t2654 = -1.*t382*t2635;
  t2655 = t2652 + t2654;
  t2693 = t364*t2561;
  t2694 = t1748*t1022;
  t2700 = t2693 + t2694;
  t2719 = -1.*t382*t2700;
  t2722 = t2719 + t2638;
  t2728 = -1.*t1001*t2700;
  t2731 = t2728 + t2654;
  t2723 = -1.*t326*t2722;
  t2749 = t1105*t2722;
  t2782 = t1001*t2700;
  t2787 = t382*t2635;
  t2794 = t2782 + t2787;
  t2814 = -1.*t326*t2794;
  t2819 = t2749 + t2814;
  t2825 = t20*t2819;
  t2875 = t573*t498;
  t2879 = t544*t587*t588;
  t2886 = t2875 + t2879;
  t2888 = t468*t2886;
  t2892 = t498*t544*t587;
  t2907 = -1.*t573*t588;
  t2913 = t2892 + t2907;
  t2923 = t2913*t798;
  t2925 = t2888 + t2923;
  t2937 = t408*t2925;
  t2939 = -1.*t884*t544*t887;
  t2940 = t2937 + t2939;
  t2952 = -1.*t364*t382*t2940;
  t2953 = -1.*t1001*t2940*t1022;
  t2976 = t2952 + t2953;
  t2984 = t1001*t364*t2940;
  t3000 = -1.*t382*t2940*t1022;
  t3019 = t2984 + t3000;
  t3070 = -1.*t2913*t798;
  t3076 = t2293 + t3070;
  t3053 = t468*t2913;
  t3064 = t2281*t798;
  t3065 = t3053 + t3064;
  t3068 = t364*t3065*t887;
  t3079 = t3076*t1022;
  t3081 = t3068 + t3079;
  t3085 = t364*t3076;
  t3092 = -1.*t3065*t887*t1022;
  t3100 = t3085 + t3092;
  t3083 = -1.*t382*t3081;
  t3101 = t1001*t3100;
  t3103 = t3083 + t3101;
  t3105 = t1001*t3081;
  t3106 = t382*t3100;
  t3107 = t3105 + t3106;
  t3188 = -1.*t468*t2886;
  t3196 = t3188 + t3070;
  t3161 = -1.*t2886*t798;
  t3177 = t3053 + t3161;
  t3182 = t364*t3177*t887;
  t3205 = t3196*t1022;
  t3207 = t3182 + t3205;
  t3215 = t364*t3196;
  t3224 = -1.*t3177*t887*t1022;
  t3225 = t3215 + t3224;
  t3211 = -1.*t382*t3207;
  t3226 = t1001*t3225;
  t3227 = t3211 + t3226;
  t3234 = t1001*t3207;
  t3244 = t382*t3225;
  t3249 = t3234 + t3244;
  t3325 = t884*t498*t468*t544;
  t3332 = -1.*t884*t544*t588*t798;
  t3335 = t3325 + t3332;
  t3289 = -1.*t408*t544*t587;
  t3296 = t884*t468*t544*t588;
  t3297 = t884*t498*t544*t798;
  t3298 = t3296 + t3297;
  t3299 = t3298*t887;
  t3302 = t3289 + t3299;
  t3314 = t364*t3302;
  t3338 = t3335*t1022;
  t3341 = t3314 + t3338;
  t3351 = t364*t3335;
  t3352 = -1.*t3302*t1022;
  t3354 = t3351 + t3352;
  t3343 = -1.*t382*t3341;
  t3360 = t1001*t3354;
  t3365 = t3343 + t3360;
  t3374 = t1001*t3341;
  t3378 = t382*t3354;
  t3381 = t3374 + t3378;
  t3432 = t884*t408*t544;
  t3445 = t2925*t887;
  t3452 = t3432 + t3445;
  t3454 = -1.*t364*t3452;
  t3456 = -1.*t3177*t1022;
  t3459 = t3454 + t3456;
  t3463 = t364*t3177;
  t3464 = -1.*t3452*t1022;
  t3474 = t3463 + t3464;
  t3460 = t382*t3459;
  t3475 = t1001*t3474;
  t3483 = t3460 + t3475;
  t3492 = t1001*t3459;
  t3497 = -1.*t382*t3474;
  t3501 = t3492 + t3497;
  t3551 = t364*t3452;
  t3552 = t3177*t1022;
  t3558 = t3551 + t3552;
  t3566 = -1.*t382*t3558;
  t3567 = t3566 + t3475;
  t3571 = -1.*t1001*t3558;
  t3572 = t3571 + t3497;
  t3568 = -1.*t326*t3567;
  t3584 = t1105*t3567;
  t3627 = t1001*t3558;
  t3632 = t382*t3474;
  t3638 = t3627 + t3632;
  t3659 = -1.*t326*t3638;
  t3660 = t3584 + t3659;
  t3666 = t20*t3660;
  t2841 = t326*t2722;
  t2845 = t1105*t2794;
  t2855 = t2841 + t2845;
  t3723 = t884*t468*t588;
  t3738 = t884*t498*t798;
  t3739 = t3723 + t3738;
  t3746 = t408*t3739;
  t3747 = t587*t887;
  t3749 = t3746 + t3747;
  t3750 = -1.*t364*t382*t3749;
  t3753 = -1.*t1001*t3749*t1022;
  t3756 = t3750 + t3753;
  t3759 = t1001*t364*t3749;
  t3760 = -1.*t382*t3749*t1022;
  t3762 = t3759 + t3760;
  t3835 = -1.*t884*t468*t588;
  t3836 = -1.*t884*t498*t798;
  t3839 = t3835 + t3836;
  t3823 = t884*t498*t468;
  t3827 = -1.*t884*t588*t798;
  t3830 = t3823 + t3827;
  t3833 = t364*t3830*t887;
  t3840 = t3839*t1022;
  t3844 = t3833 + t3840;
  t3859 = t364*t3839;
  t3860 = -1.*t3830*t887*t1022;
  t3867 = t3859 + t3860;
  t3852 = -1.*t382*t3844;
  t3870 = t1001*t3867;
  t3872 = t3852 + t3870;
  t3880 = t1001*t3844;
  t3885 = t382*t3867;
  t3887 = t3880 + t3885;
  t3879 = t326*t3872;
  t3888 = t1105*t3887;
  t3895 = t3879 + t3888;
  t3897 = t20*t3895;
  t3901 = t1105*t3872;
  t3905 = -1.*t326*t3887;
  t3909 = t3901 + t3905;
  t3912 = -1.*t1287*t3909;
  t3916 = t3897 + t3912;
  t3933 = -1.*t408*t587;
  t3937 = t3739*t887;
  t3952 = t3933 + t3937;
  t3955 = -1.*t364*t3952;
  t3957 = -1.*t3830*t1022;
  t3958 = t3955 + t3957;
  t3972 = t364*t3830;
  t3977 = -1.*t3952*t1022;
  t3983 = t3972 + t3977;
  t3967 = t382*t3958;
  t3986 = t1001*t3983;
  t3990 = t3967 + t3986;
  t3998 = t1001*t3958;
  t4007 = -1.*t382*t3983;
  t4010 = t3998 + t4007;
  t4052 = t364*t3952;
  t4053 = t3830*t1022;
  t4056 = t4052 + t4053;
  t4058 = -1.*t382*t4056;
  t4061 = t4058 + t3986;
  t4080 = -1.*t1001*t4056;
  t4082 = t4080 + t4007;
  t4063 = -1.*t326*t4061;
  t4103 = t1105*t4061;
  t4127 = t1001*t4056;
  t4129 = t382*t3983;
  t4131 = t4127 + t4129;
  t4160 = -1.*t326*t4131;
  t4163 = t4103 + t4160;
  t4164 = t20*t4163;
  t4218 = -1.*t498*t468*t587;
  t4221 = t587*t588*t798;
  t4227 = t4218 + t4221;
  t4195 = -1.*t884*t408;
  t4199 = -1.*t468*t587*t588;
  t4204 = -1.*t498*t587*t798;
  t4211 = t4199 + t4204;
  t4212 = t4211*t887;
  t4215 = t4195 + t4212;
  t4216 = t364*t4215;
  t4229 = t4227*t1022;
  t4232 = t4216 + t4229;
  t4237 = t364*t4227;
  t4238 = -1.*t4215*t1022;
  t4240 = t4237 + t4238;
  t4236 = -1.*t382*t4232;
  t4243 = t1001*t4240;
  t4244 = t4236 + t4243;
  t4254 = t1001*t4232;
  t4255 = t382*t4240;
  t4260 = t4254 + t4255;
  t1096 = t326*t1088;
  t1195 = t1105*t1182;
  t1213 = t1096 + t1195;
  t1296 = t1105*t1088;
  t1308 = -1.*t326*t1182;
  t1332 = t1296 + t1308;
  t1608 = t326*t1596;
  t1643 = t1105*t1634;
  t1648 = t1608 + t1643;
  t1663 = t1105*t1596;
  t1673 = -1.*t326*t1634;
  t1687 = t1663 + t1673;
  t1826 = t326*t1822;
  t1862 = t1105*t1847;
  t1866 = t1826 + t1862;
  t1894 = t1105*t1822;
  t1896 = -1.*t326*t1847;
  t1910 = t1894 + t1896;
  t2187 = t326*t2183;
  t2199 = t1105*t2196;
  t2200 = t2187 + t2199;
  t2205 = t1105*t2183;
  t2222 = -1.*t326*t2196;
  t2230 = t2205 + t2222;
  t2469 = t326*t2428;
  t2509 = t1105*t2485;
  t2515 = t2469 + t2509;
  t2523 = t1105*t2428;
  t2528 = -1.*t326*t2485;
  t2529 = t2523 + t2528;
  t2650 = -1.*t326*t2639;
  t2661 = t1105*t2655;
  t2663 = t2650 + t2661;
  t2670 = t1105*t2639;
  t2671 = t326*t2655;
  t2673 = t2670 + t2671;
  t2732 = t1105*t2731;
  t2733 = t2723 + t2732;
  t2755 = t326*t2731;
  t2756 = t2749 + t2755;
  t2796 = -1.*t1105*t2794;
  t2802 = t2723 + t2796;
  t4378 = t1287*t2819;
  t2978 = t326*t2976;
  t3022 = t1105*t3019;
  t3025 = t2978 + t3022;
  t3030 = t1105*t2976;
  t3036 = -1.*t326*t3019;
  t3038 = t3030 + t3036;
  t3104 = t326*t3103;
  t3111 = t1105*t3107;
  t3113 = t3104 + t3111;
  t3117 = t1105*t3103;
  t3120 = -1.*t326*t3107;
  t3124 = t3117 + t3120;
  t3233 = t326*t3227;
  t3250 = t1105*t3249;
  t3251 = t3233 + t3250;
  t3262 = t1105*t3227;
  t3263 = -1.*t326*t3249;
  t3266 = t3262 + t3263;
  t3370 = t326*t3365;
  t3390 = t1105*t3381;
  t3399 = t3370 + t3390;
  t3405 = t1105*t3365;
  t3409 = -1.*t326*t3381;
  t3411 = t3405 + t3409;
  t3487 = -1.*t326*t3483;
  t3505 = t1105*t3501;
  t3508 = t3487 + t3505;
  t3528 = t1105*t3483;
  t3529 = t326*t3501;
  t3530 = t3528 + t3529;
  t3578 = t1105*t3572;
  t3579 = t3568 + t3578;
  t3590 = t326*t3572;
  t3610 = t3584 + t3590;
  t3641 = -1.*t1105*t3638;
  t3653 = t3568 + t3641;
  t3677 = t326*t3567;
  t3683 = t1105*t3638;
  t3690 = t3677 + t3683;
  t4500 = t1287*t3660;
  t2858 = t1287*t2855;
  t2862 = t2858 + t2825;
  t3757 = t326*t3756;
  t3771 = t1105*t3762;
  t3776 = t3757 + t3771;
  t3787 = t1105*t3756;
  t3796 = -1.*t326*t3762;
  t3807 = t3787 + t3796;
  t4543 = t1287*t3895;
  t4551 = t20*t3909;
  t4553 = t4543 + t4551;
  t3996 = -1.*t326*t3990;
  t4011 = t1105*t4010;
  t4014 = t3996 + t4011;
  t4022 = t1105*t3990;
  t4028 = t326*t4010;
  t4035 = t4022 + t4028;
  t4089 = t1105*t4082;
  t4096 = t4063 + t4089;
  t4105 = t326*t4082;
  t4106 = t4103 + t4105;
  t4135 = -1.*t1105*t4131;
  t4137 = t4063 + t4135;
  t4175 = t326*t4061;
  t4176 = t1105*t4131;
  t4179 = t4175 + t4176;
  t4599 = t1287*t4163;
  t4249 = t326*t4244;
  t4264 = t1105*t4260;
  t4267 = t4249 + t4264;
  t4269 = t1105*t4244;
  t4274 = -1.*t326*t4260;
  t4276 = t4269 + t4274;
  t4765 = -1.*t1001;
  t4769 = 1. + t4765;
  t4834 = -1.*t1105;
  t4836 = 1. + t4834;
  t4856 = -1.*t1287;
  t4858 = 1. + t4856;
  t4294 = t1287*t1213;
  t4296 = t20*t1332;
  t4297 = t4294 + t4296;
  t4915 = -1.*t468;
  t4916 = 1. + t4915;
  t4797 = -1.*t364;
  t4798 = 1. + t4797;
  t4808 = -0.08055*t4798;
  t4809 = -0.01004*t1022;
  t4812 = 0. + t4808 + t4809;
  t4771 = -0.08055*t4769;
  t4772 = -0.13004*t382;
  t4773 = 0. + t4771 + t4772;
  t4816 = -0.13004*t4769;
  t4821 = 0.08055*t382;
  t4829 = 0. + t4816 + t4821;
  t4838 = -0.19074*t4836;
  t4840 = 0.03315*t326;
  t4842 = 0. + t4838 + t4840;
  t4844 = -0.03315*t4836;
  t4849 = -0.19074*t326;
  t4850 = 0. + t4844 + t4849;
  t4859 = -0.01315*t4858;
  t4861 = -0.62554*t20;
  t4864 = 0. + t4859 + t4861;
  t4877 = -0.62554*t4858;
  t4882 = 0.01315*t20;
  t4884 = 0. + t4877 + t4882;
  t4300 = t1287*t1648;
  t4301 = t20*t1687;
  t4303 = t4300 + t4301;
  t4936 = 0.07996*t798;
  t4949 = -1.*t408;
  t4956 = 1. + t4949;
  t4960 = 0.135*t4956;
  t4962 = 0.08055*t887;
  t4968 = 0. + t4960 + t4962;
  t4979 = -0.01004*t4798;
  t4983 = 0.08055*t1022;
  t4990 = 0. + t4979 + t4983;
  t4310 = t1287*t1866;
  t4311 = t20*t1910;
  t4313 = t4310 + t4311;
  t4917 = 0.07996*t4916;
  t4918 = -0.135*t798;
  t4921 = 0. + t4917 + t4918;
  t4932 = 0.135*t4916;
  t4938 = 0. + t4932 + t4936;
  t4753 = 0.135*t887;
  t4316 = t1287*t2200;
  t4317 = t20*t2230;
  t4321 = t4316 + t4317;
  t5118 = -0.08055*t4956;
  t5119 = 0. + t5118 + t4753;
  t4328 = t1287*t2515;
  t4333 = t20*t2529;
  t4334 = t4328 + t4333;
  t4338 = t20*t2663;
  t4342 = t1287*t2673;
  t4343 = t4338 + t4342;
  t4350 = t20*t2733;
  t4359 = t1287*t2756;
  t4365 = t4350 + t4359;
  t4377 = t20*t2802;
  t4384 = t4377 + t4378;
  t4396 = -1.*t20*t2855;
  t4397 = t4396 + t4378;
  t5456 = -1.*t20*t2819;
  t4742 = 0.135*t408;
  t4743 = -0.08055*t887;
  t4748 = t4742 + t4743;
  t4752 = 0.08055*t408;
  t4760 = t4752 + t4753;
  t4406 = t1287*t3025;
  t4416 = t20*t3038;
  t4422 = t4406 + t4416;
  t4427 = t1287*t3113;
  t4428 = t20*t3124;
  t4433 = t4427 + t4428;
  t5028 = -0.135*t468;
  t5034 = t5028 + t4936;
  t5039 = 0.07996*t468;
  t5041 = 0.135*t798;
  t5046 = t5039 + t5041;
  t4443 = t1287*t3251;
  t4444 = t20*t3266;
  t4446 = t4443 + t4444;
  t4454 = t1287*t3399;
  t4455 = t20*t3411;
  t4457 = t4454 + t4455;
  t5294 = -0.01004*t364;
  t5297 = -0.08055*t1022;
  t5303 = t5294 + t5297;
  t5305 = 0.08055*t364;
  t5307 = t5305 + t4809;
  t4466 = t20*t3508;
  t4469 = t1287*t3530;
  t4473 = t4466 + t4469;
  t5365 = -0.13004*t1001;
  t5369 = -0.08055*t382;
  t5373 = t5365 + t5369;
  t5377 = 0.08055*t1001;
  t5378 = t5377 + t4772;
  t4478 = t20*t3579;
  t4484 = t1287*t3610;
  t4485 = t4478 + t4484;
  t5416 = 0.03315*t1105;
  t5417 = t5416 + t4849;
  t5421 = -0.19074*t1105;
  t5432 = -0.03315*t326;
  t5433 = t5421 + t5432;
  t4494 = t20*t3653;
  t4501 = t4494 + t4500;
  t5469 = -0.62554*t1287;
  t5475 = -0.01315*t20;
  t5479 = t5469 + t5475;
  t5489 = 0.01315*t1287;
  t5491 = t5489 + t4861;
  t4504 = -1.*t20*t3690;
  t4506 = t4504 + t4500;
  t5904 = -1.*t20*t3660;
  t4528 = t1287*t3776;
  t4532 = t20*t3807;
  t4533 = t4528 + t4532;
  t6116 = 0.1318*t408*t3830;
  t6120 = t3830*t4968;
  t6121 = t3830*t887*t4812;
  t6123 = t3839*t4990;
  t6126 = t4773*t3844;
  t6127 = t4829*t3867;
  t6129 = t4842*t3872;
  t6130 = t4850*t3887;
  t6133 = t4864*t3895;
  t6139 = t4884*t3909;
  t6140 = -1.*t20*t3895;
  t6141 = t1287*t3909;
  t6143 = t6140 + t6141;
  t6144 = -0.73604*t6143;
  t6146 = -0.04375*t4553;
  t4564 = t20*t4014;
  t4567 = t1287*t4035;
  t4571 = t4564 + t4567;
  t4577 = t20*t4096;
  t4580 = t1287*t4106;
  t4581 = t4577 + t4580;
  t4589 = t20*t4137;
  t4601 = t4589 + t4599;
  t4605 = -1.*t20*t4179;
  t4606 = t4605 + t4599;
  t6242 = -1.*t20*t4163;
  t4608 = t1287*t4267;
  t4611 = t20*t4276;
  t4613 = t4608 + t4611;
  p_output1[0]=(t20*t2515 - 1.*t1287*t2529)*var2[3] + (t20*t2200 - 1.*t1287*t2230)*var2[4] + (-1.*t1287*t1687 + t1648*t20)*var2[5] + (-1.*t1287*t1910 + t1866*t20)*var2[6] + (-1.*t1287*t1332 + t1213*t20)*var2[7] + (-1.*t1287*t2663 + t20*t2673)*var2[8] + (-1.*t1287*t2733 + t20*t2756)*var2[9] + (-1.*t1287*t2802 + t2825)*var2[10] + t2862*var2[11];
  p_output1[1]=(-1.*t1287*t2819 + t20*t2855)*var2[3] + (t20*t3399 - 1.*t1287*t3411)*var2[4] + (t20*t3113 - 1.*t1287*t3124)*var2[5] + (t20*t3251 - 1.*t1287*t3266)*var2[6] + (t20*t3025 - 1.*t1287*t3038)*var2[7] + (-1.*t1287*t3508 + t20*t3530)*var2[8] + (-1.*t1287*t3579 + t20*t3610)*var2[9] + (-1.*t1287*t3653 + t3666)*var2[10] + (t3666 + t1287*t3690)*var2[11];
  p_output1[2]=(t20*t4267 - 1.*t1287*t4276)*var2[4] + t3916*var2[5] + t3916*var2[6] + (t20*t3776 - 1.*t1287*t3807)*var2[7] + (-1.*t1287*t4014 + t20*t4035)*var2[8] + (-1.*t1287*t4096 + t20*t4106)*var2[9] + (-1.*t1287*t4137 + t4164)*var2[10] + (t4164 + t1287*t4179)*var2[11];
  p_output1[3]=0;
  p_output1[4]=t4334*var2[3] + t4321*var2[4] + t4303*var2[5] + t4313*var2[6] + t4297*var2[7] + t4343*var2[8] + t4365*var2[9] + t4384*var2[10] + t4397*var2[11];
  p_output1[5]=t2862*var2[3] + t4457*var2[4] + t4433*var2[5] + t4446*var2[6] + t4422*var2[7] + t4473*var2[8] + t4485*var2[9] + t4501*var2[10] + t4506*var2[11];
  p_output1[6]=t4613*var2[4] + t4553*var2[5] + t4553*var2[6] + t4533*var2[7] + t4571*var2[8] + t4581*var2[9] + t4601*var2[10] + t4606*var2[11];
  p_output1[7]=0;
  p_output1[8]=(t2939 - 1.*t2340*t408)*var2[3] + (-1.*t1984*t408 - 1.*t573*t587*t887)*var2[4] - 1.*t1463*t408*var2[5] - 1.*t1748*t408*var2[6] + t2561*var2[7];
  p_output1[9]=(-1.*t408*t860 + t573*t884*t887)*var2[3] + (-1.*t3298*t408 - 1.*t544*t587*t887)*var2[4] - 1.*t3065*t408*var2[5] - 1.*t3177*t408*var2[6] + t3452*var2[7];
  p_output1[10]=(-1.*t408*t4211 - 1.*t884*t887)*var2[4] - 1.*t3830*t408*var2[5] - 1.*t3830*t408*var2[6] + t3952*var2[7];
  p_output1[11]=0;
  p_output1[12]=var2[0] + (-0.73604*(-1.*t20*t2515 + t1287*t2529) - 0.04375*t4334 + t2402*t4773 + t2346*t4812 + t2409*t4829 + t2428*t4842 + t2485*t4850 + t2515*t4864 + t2529*t4884 + t2324*t4921 + t2281*t4938 + t2340*t4968 + t2392*t4990 - 1.*t5119*t544*t884 + 0.1318*(t2340*t408 + t544*t884*t887))*var2[3] + (-0.73604*(-1.*t20*t2200 + t1287*t2230) - 0.04375*t4321 + t2109*t4773 + t2034*t4812 + t2170*t4829 + t2183*t4842 + t2196*t4850 + t2200*t4864 + t2230*t4884 + t1984*t4968 + t2080*t4990 - 1.*t5119*t573*t587 + t4921*t498*t573*t884 + t4938*t573*t588*t884 + 0.1318*(t1984*t408 + t573*t587*t887))*var2[4] + (-0.73604*(t1287*t1687 - 1.*t1648*t20) + 0.1318*t1463*t408 - 0.04375*t4303 + t1532*t4773 + t1590*t4829 + t1596*t4842 + t1634*t4850 + t1648*t4864 + t1687*t4884 + t1416*t4921 + t1463*t4968 + t1508*t4990 + t4938*t752 + t1463*t4812*t887)*var2[5] + (-0.73604*(t1287*t1910 - 1.*t1866*t20) + 0.1318*t1748*t408 - 0.04375*t4313 + t1780*t4773 + t1790*t4829 + t1822*t4842 + t1847*t4850 + t1866*t4864 + t1910*t4884 + t1748*t4968 + t1771*t4990 + t5046*t631 + t5034*t752 + t1748*t4812*t887)*var2[6] + (-0.73604*(t1287*t1332 - 1.*t1213*t20) - 0.04375*t4297 + t1088*t4842 + t1182*t4850 + t1213*t4864 + t1332*t4884 + t4760*t860 + t4748*t573*t884 + 0.1318*(-1.*t408*t573*t884 - 1.*t860*t887) + t364*t4773*t960 + t4812*t960 - 1.*t1022*t4829*t960)*var2[7] + (-0.73604*(t1287*t2663 - 1.*t20*t2673) - 0.04375*t4343 + t2635*t4773 + t2593*t4829 + t2655*t4842 + t2639*t4850 + t2673*t4864 + t2663*t4884 + t2561*t5303 + t1748*t5307)*var2[8] + (-0.73604*(t1287*t2733 - 1.*t20*t2756) - 0.04375*t4365 + t2731*t4842 + t2722*t4850 + t2756*t4864 + t2733*t4884 + t2700*t5373 + t2635*t5378)*var2[9] + (-0.04375*t4384 + t2819*t4864 + t2802*t4884 + t2722*t5417 + t2794*t5433 - 0.73604*(t1287*t2802 + t5456))*var2[10] + (-0.04375*t4397 - 0.73604*(-1.*t1287*t2855 + t5456) + t2855*t5479 + t2819*t5491)*var2[11];
  p_output1[13]=var2[1] + (-0.04375*t2862 - 0.73604*t4397 + t2700*t4773 + t2561*t4812 + t2635*t4829 + t2722*t4842 + t2794*t4850 + t2855*t4864 + t2819*t4884 + t1748*t4990 + t4938*t631 + t4921*t752 + t4968*t860 + t5119*t573*t884 + 0.1318*t960)*var2[3] + (-0.73604*(-1.*t20*t3399 + t1287*t3411) - 0.04375*t4457 + t3341*t4773 + t3302*t4812 + t3354*t4829 + t3365*t4842 + t3381*t4850 + t3399*t4864 + t3411*t4884 + t3298*t4968 + t3335*t4990 - 1.*t5119*t544*t587 + t4921*t498*t544*t884 + t4938*t544*t588*t884 + 0.1318*(t3298*t408 + t544*t587*t887))*var2[4] + (-0.73604*(-1.*t20*t3113 + t1287*t3124) + 0.1318*t3065*t408 - 0.04375*t4433 + t3081*t4773 + t3100*t4829 + t3103*t4842 + t3107*t4850 + t3113*t4864 + t3124*t4884 + t2281*t4921 + t2913*t4938 + t3065*t4968 + t3076*t4990 + t3065*t4812*t887)*var2[5] + (-0.73604*(-1.*t20*t3251 + t1287*t3266) + 0.1318*t3177*t408 - 0.04375*t4446 + t3207*t4773 + t3225*t4829 + t3227*t4842 + t3249*t4850 + t3251*t4864 + t3266*t4884 + t3177*t4968 + t3196*t4990 + t2913*t5034 + t2886*t5046 + t3177*t4812*t887)*var2[6] + (-0.73604*(-1.*t20*t3025 + t1287*t3038) - 0.04375*t4422 + t2925*t4760 + t2940*t364*t4773 + t2940*t4812 - 1.*t1022*t2940*t4829 + t2976*t4842 + t3019*t4850 + t3025*t4864 + t3038*t4884 + t4748*t544*t884 + 0.1318*(t2268 - 1.*t2925*t887))*var2[7] + (-0.73604*(t1287*t3508 - 1.*t20*t3530) - 0.04375*t4473 + t3474*t4773 + t3459*t4829 + t3501*t4842 + t3483*t4850 + t3530*t4864 + t3508*t4884 + t3452*t5303 + t3177*t5307)*var2[8] + (-0.73604*(t1287*t3579 - 1.*t20*t3610) - 0.04375*t4485 + t3572*t4842 + t3567*t4850 + t3610*t4864 + t3579*t4884 + t3558*t5373 + t3474*t5378)*var2[9] + (-0.04375*t4501 + t3660*t4864 + t3653*t4884 + t3567*t5417 + t3638*t5433 - 0.73604*(t1287*t3653 + t5904))*var2[10] + (-0.04375*t4506 + t3690*t5479 + t3660*t5491 - 0.73604*(-1.*t1287*t3690 + t5904))*var2[11];
  p_output1[14]=var2[2] + (-0.73604*(-1.*t20*t4267 + t1287*t4276) - 0.04375*t4613 + t4232*t4773 + t4215*t4812 + t4240*t4829 + t4244*t4842 + t4260*t4850 + t4267*t4864 + t4276*t4884 + t4211*t4968 + t4227*t4990 - 1.*t4921*t498*t587 - 1.*t4938*t587*t588 - 1.*t5119*t884 + 0.1318*(t408*t4211 + t884*t887))*var2[4] + (t6116 + t6120 + t6121 + t6123 + t6126 + t6127 + t6129 + t6130 + t6133 + t6139 + t6144 + t6146 + t4938*t498*t884 - 1.*t4921*t588*t884)*var2[5] + (t6116 + t6120 + t6121 + t6123 + t6126 + t6127 + t6129 + t6130 + t6133 + t6139 + t6144 + t6146 + t498*t5034*t884 + t5046*t588*t884)*var2[6] + (-0.73604*(-1.*t20*t3776 + t1287*t3807) - 0.04375*t4533 + t3739*t4760 + t364*t3749*t4773 + t3749*t4812 - 1.*t1022*t3749*t4829 + t3756*t4842 + t3762*t4850 + t3776*t4864 + t3807*t4884 - 1.*t4748*t587 + 0.1318*(t408*t587 - 1.*t3739*t887))*var2[7] + (-0.73604*(t1287*t4014 - 1.*t20*t4035) - 0.04375*t4571 + t3983*t4773 + t3958*t4829 + t4010*t4842 + t3990*t4850 + t4035*t4864 + t4014*t4884 + t3952*t5303 + t3830*t5307)*var2[8] + (-0.73604*(t1287*t4096 - 1.*t20*t4106) - 0.04375*t4581 + t4082*t4842 + t4061*t4850 + t4106*t4864 + t4096*t4884 + t4056*t5373 + t3983*t5378)*var2[9] + (-0.04375*t4601 + t4163*t4864 + t4137*t4884 + t4061*t5417 + t4131*t5433 - 0.73604*(t1287*t4137 + t6242))*var2[10] + (-0.04375*t4606 + t4179*t5479 + t4163*t5491 - 0.73604*(-1.*t1287*t4179 + t6242))*var2[11];
  p_output1[15]=0;
}



#ifdef MATLAB_MEX_FILE

#include "mex.h"
/*
 * Main function
 */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  size_t mrows, ncols;

  double *var1,*var2;
  double *p_output1;

  /*  Check for proper number of arguments.  */ 
  if( nrhs != 2)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "Two input(s) required (var1,var2).");
    }
  else if( nlhs > 1)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:maxlhs", "Too many output arguments.");
    }

  /*  The input must be a noncomplex double vector or scaler.  */
  mrows = mxGetM(prhs[0]);
  ncols = mxGetN(prhs[0]);
  if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
    ( !(mrows == 20 && ncols == 1) && 
      !(mrows == 1 && ncols == 20))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "var1 is wrong.");
    }
  mrows = mxGetM(prhs[1]);
  ncols = mxGetN(prhs[1]);
  if( !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) ||
    ( !(mrows == 20 && ncols == 1) && 
      !(mrows == 1 && ncols == 20))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "var2 is wrong.");
    }

  /*  Assign pointers to each input.  */
  var1 = mxGetPr(prhs[0]);
  var2 = mxGetPr(prhs[1]);
   


   
  /*  Create matrices for return arguments.  */
  plhs[0] = mxCreateDoubleMatrix((mwSize) 4, (mwSize) 4, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,var1,var2);


}

#else // MATLAB_MEX_FILE

#include "dT_LeftTarsus_mex.hh"

namespace SymExpression
{

void dT_LeftTarsus_mex_raw(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}

}

#endif // MATLAB_MEX_FILE
