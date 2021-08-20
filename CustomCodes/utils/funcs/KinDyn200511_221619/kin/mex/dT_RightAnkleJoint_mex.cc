/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:22:31 GMT-04:00
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
  double t455;
  double t473;
  double t478;
  double t606;
  double t423;
  double t615;
  double t747;
  double t568;
  double t663;
  double t694;
  double t385;
  double t769;
  double t784;
  double t793;
  double t798;
  double t799;
  double t804;
  double t807;
  double t823;
  double t836;
  double t941;
  double t724;
  double t868;
  double t891;
  double t374;
  double t952;
  double t967;
  double t976;
  double t1012;
  double t901;
  double t1005;
  double t1007;
  double t311;
  double t1016;
  double t1041;
  double t1051;
  double t240;
  double t1227;
  double t1203;
  double t1242;
  double t1245;
  double t1305;
  double t1317;
  double t1322;
  double t1284;
  double t1328;
  double t1342;
  double t1385;
  double t1391;
  double t1394;
  double t1373;
  double t1418;
  double t1419;
  double t1421;
  double t1434;
  double t1447;
  double t1125;
  double t1420;
  double t1449;
  double t1496;
  double t1504;
  double t1505;
  double t1506;
  double t1582;
  double t1586;
  double t1592;
  double t1594;
  double t1598;
  double t1605;
  double t1621;
  double t1625;
  double t1637;
  double t1641;
  double t1644;
  double t1649;
  double t1659;
  double t1660;
  double t1666;
  double t1753;
  double t1759;
  double t1767;
  double t1768;
  double t1766;
  double t1777;
  double t1801;
  double t1818;
  double t1821;
  double t1829;
  double t1806;
  double t1832;
  double t1835;
  double t1851;
  double t1855;
  double t1892;
  double t1955;
  double t1962;
  double t1963;
  double t1952;
  double t1968;
  double t1975;
  double t1983;
  double t1988;
  double t1992;
  double t1980;
  double t2003;
  double t2011;
  double t2018;
  double t2021;
  double t2031;
  double t2074;
  double t2086;
  double t2106;
  double t2113;
  double t2120;
  double t2127;
  double t2130;
  double t2123;
  double t2159;
  double t2189;
  double t2190;
  double t2192;
  double t2210;
  double t2212;
  double t2214;
  double t2270;
  double t2277;
  double t2279;
  double t2303;
  double t2306;
  double t2310;
  double t2294;
  double t2311;
  double t2314;
  double t2327;
  double t2331;
  double t2333;
  double t2334;
  double t2347;
  double t2351;
  double t2325;
  double t2353;
  double t2356;
  double t2359;
  double t2360;
  double t2361;
  double t2357;
  double t2381;
  double t2386;
  double t2402;
  double t2407;
  double t2412;
  double t2462;
  double t2479;
  double t2483;
  double t2498;
  double t2499;
  double t2503;
  double t2513;
  double t2514;
  double t2520;
  double t2490;
  double t2521;
  double t2527;
  double t2533;
  double t2537;
  double t2563;
  double t2531;
  double t2564;
  double t2569;
  double t2573;
  double t2575;
  double t2576;
  double t2607;
  double t2614;
  double t2615;
  double t2627;
  double t2628;
  double t2636;
  double t2638;
  double t2641;
  double t2633;
  double t2646;
  double t2647;
  double t2654;
  double t2667;
  double t2669;
  double t2653;
  double t2679;
  double t2681;
  double t2692;
  double t2700;
  double t2713;
  double t2762;
  double t2774;
  double t2778;
  double t2782;
  double t2789;
  double t2791;
  double t2798;
  double t2799;
  double t2802;
  double t2803;
  double t2807;
  double t2812;
  double t2818;
  double t2827;
  double t2829;
  double t2862;
  double t2863;
  double t2867;
  double t2868;
  double t2864;
  double t2873;
  double t2874;
  double t2887;
  double t2894;
  double t2905;
  double t2882;
  double t2908;
  double t2910;
  double t2922;
  double t2924;
  double t2935;
  double t2240;
  double t2241;
  double t2253;
  double t2987;
  double t2990;
  double t2991;
  double t2985;
  double t2998;
  double t2999;
  double t3008;
  double t3018;
  double t3022;
  double t3004;
  double t3024;
  double t3027;
  double t3038;
  double t3039;
  double t3040;
  double t3095;
  double t3103;
  double t3108;
  double t3110;
  double t3117;
  double t3125;
  double t3127;
  double t3119;
  double t3144;
  double t3190;
  double t3196;
  double t3197;
  double t3224;
  double t3226;
  double t3227;
  double t3256;
  double t3261;
  double t3265;
  double t3268;
  double t3272;
  double t3273;
  double t3274;
  double t3275;
  double t3282;
  double t3284;
  double t3287;
  double t3290;
  double t3360;
  double t3361;
  double t3365;
  double t3373;
  double t3389;
  double t3397;
  double t3371;
  double t3400;
  double t3401;
  double t3407;
  double t3414;
  double t3435;
  double t3406;
  double t3436;
  double t3444;
  double t3458;
  double t3469;
  double t3471;
  double t3453;
  double t3474;
  double t3479;
  double t3482;
  double t3484;
  double t3488;
  double t3489;
  double t3492;
  double t3494;
  double t3504;
  double t3507;
  double t3509;
  double t3502;
  double t3515;
  double t3516;
  double t3522;
  double t3527;
  double t3532;
  double t3518;
  double t3540;
  double t3545;
  double t3560;
  double t3563;
  double t3571;
  double t3611;
  double t3613;
  double t3616;
  double t3620;
  double t3623;
  double t3631;
  double t3634;
  double t3629;
  double t3656;
  double t3670;
  double t3671;
  double t3672;
  double t3690;
  double t3698;
  double t3700;
  double t3731;
  double t3742;
  double t3752;
  double t3755;
  double t3756;
  double t3758;
  double t3770;
  double t3772;
  double t3778;
  double t3754;
  double t3782;
  double t3786;
  double t3796;
  double t3798;
  double t3802;
  double t3790;
  double t3803;
  double t3807;
  double t3810;
  double t3815;
  double t3820;
  double t1008;
  double t1091;
  double t1110;
  double t1127;
  double t1161;
  double t1167;
  double t1501;
  double t1509;
  double t1512;
  double t1551;
  double t1554;
  double t1555;
  double t1653;
  double t1671;
  double t1672;
  double t1713;
  double t1715;
  double t1730;
  double t1838;
  double t1893;
  double t1894;
  double t1898;
  double t1901;
  double t1902;
  double t2017;
  double t2035;
  double t2038;
  double t2044;
  double t2052;
  double t2060;
  double t2135;
  double t2144;
  double t2177;
  double t2178;
  double t2193;
  double t2199;
  double t3950;
  double t2393;
  double t2419;
  double t2427;
  double t2430;
  double t2432;
  double t2433;
  double t2571;
  double t2577;
  double t2578;
  double t2584;
  double t2586;
  double t2591;
  double t2690;
  double t2732;
  double t2738;
  double t2742;
  double t2744;
  double t2753;
  double t2815;
  double t2830;
  double t2832;
  double t2840;
  double t2843;
  double t2845;
  double t2911;
  double t2936;
  double t2940;
  double t2945;
  double t2948;
  double t2949;
  double t2254;
  double t2258;
  double t3033;
  double t3041;
  double t3044;
  double t3055;
  double t3070;
  double t3083;
  double t3132;
  double t3134;
  double t3149;
  double t3151;
  double t3211;
  double t3217;
  double t3237;
  double t3242;
  double t3245;
  double t4196;
  double t3283;
  double t3298;
  double t3303;
  double t3314;
  double t3318;
  double t3323;
  double t4256;
  double t4257;
  double t4260;
  double t3558;
  double t3572;
  double t3573;
  double t3580;
  double t3596;
  double t3600;
  double t3644;
  double t3645;
  double t3657;
  double t3659;
  double t3679;
  double t3684;
  double t3714;
  double t3716;
  double t3721;
  double t4299;
  double t3809;
  double t3829;
  double t3830;
  double t3834;
  double t3837;
  double t3840;
  double t4487;
  double t4489;
  double t4530;
  double t4532;
  double t4577;
  double t4583;
  double t4632;
  double t4635;
  double t4675;
  double t4676;
  double t4733;
  double t4750;
  double t3859;
  double t3866;
  double t3868;
  double t4549;
  double t4553;
  double t4558;
  double t4491;
  double t4514;
  double t4515;
  double t4587;
  double t4596;
  double t4600;
  double t4561;
  double t4572;
  double t4574;
  double t4623;
  double t4628;
  double t4630;
  double t4637;
  double t4645;
  double t4649;
  double t4653;
  double t4664;
  double t4668;
  double t4697;
  double t4699;
  double t4703;
  double t4715;
  double t4719;
  double t4731;
  double t4756;
  double t4758;
  double t4766;
  double t4777;
  double t4784;
  double t4792;
  double t3877;
  double t3880;
  double t3883;
  double t4542;
  double t3890;
  double t3892;
  double t3893;
  double t3899;
  double t3900;
  double t3908;
  double t3918;
  double t3922;
  double t3923;
  double t3934;
  double t3936;
  double t3945;
  double t3947;
  double t3955;
  double t3965;
  double t3981;
  double t5420;
  double t4540;
  double t4544;
  double t3992;
  double t3997;
  double t4014;
  double t4032;
  double t4033;
  double t4034;
  double t4063;
  double t4064;
  double t4072;
  double t4928;
  double t4942;
  double t4960;
  double t4966;
  double t4973;
  double t4093;
  double t4098;
  double t4108;
  double t5089;
  double t5091;
  double t5102;
  double t5104;
  double t5110;
  double t4129;
  double t4130;
  double t4134;
  double t5214;
  double t5216;
  double t5223;
  double t5224;
  double t5229;
  double t4141;
  double t4146;
  double t4149;
  double t5294;
  double t5304;
  double t5307;
  double t5311;
  double t5317;
  double t4162;
  double t4173;
  double t4180;
  double t5376;
  double t5378;
  double t5391;
  double t5396;
  double t5398;
  double t4191;
  double t4198;
  double t5450;
  double t5452;
  double t5460;
  double t5463;
  double t5464;
  double t4215;
  double t4218;
  double t6162;
  double t4236;
  double t4243;
  double t4247;
  double t6292;
  double t6294;
  double t6296;
  double t6297;
  double t6298;
  double t6299;
  double t6302;
  double t6308;
  double t6310;
  double t6324;
  double t6334;
  double t6337;
  double t6338;
  double t6339;
  double t6340;
  double t4274;
  double t4275;
  double t4289;
  double t4291;
  double t4293;
  double t4294;
  double t4298;
  double t4300;
  double t4315;
  double t4324;
  double t6485;
  double t4329;
  double t4333;
  double t4349;
  t455 = Cos(var1[3]);
  t473 = Cos(var1[4]);
  t478 = Cos(var1[5]);
  t606 = Sin(var1[13]);
  t423 = Cos(var1[13]);
  t615 = Sin(var1[5]);
  t747 = Cos(var1[15]);
  t568 = t423*t455*t473*t478;
  t663 = -1.*t455*t473*t606*t615;
  t694 = t568 + t663;
  t385 = Sin(var1[15]);
  t769 = Cos(var1[14]);
  t784 = Sin(var1[4]);
  t793 = -1.*t769*t455*t784;
  t798 = Sin(var1[14]);
  t799 = t455*t473*t478*t606;
  t804 = t423*t455*t473*t615;
  t807 = t799 + t804;
  t823 = t798*t807;
  t836 = t793 + t823;
  t941 = Cos(var1[16]);
  t724 = t385*t694;
  t868 = t747*t836;
  t891 = t724 + t868;
  t374 = Sin(var1[16]);
  t952 = t747*t694;
  t967 = -1.*t385*t836;
  t976 = t952 + t967;
  t1012 = Cos(var1[17]);
  t901 = -1.*t374*t891;
  t1005 = t941*t976;
  t1007 = t901 + t1005;
  t311 = Sin(var1[17]);
  t1016 = t941*t891;
  t1041 = t374*t976;
  t1051 = t1016 + t1041;
  t240 = Sin(var1[18]);
  t1227 = Sin(var1[3]);
  t1203 = t455*t478*t784;
  t1242 = t1227*t615;
  t1245 = t1203 + t1242;
  t1305 = t478*t1227;
  t1317 = -1.*t455*t784*t615;
  t1322 = t1305 + t1317;
  t1284 = -1.*t606*t1245;
  t1328 = t423*t1322;
  t1342 = t1284 + t1328;
  t1385 = t423*t1245;
  t1391 = t606*t1322;
  t1394 = t1385 + t1391;
  t1373 = t385*t1342;
  t1418 = t747*t798*t1394;
  t1419 = t1373 + t1418;
  t1421 = t747*t1342;
  t1434 = -1.*t798*t385*t1394;
  t1447 = t1421 + t1434;
  t1125 = Cos(var1[18]);
  t1420 = -1.*t374*t1419;
  t1449 = t941*t1447;
  t1496 = t1420 + t1449;
  t1504 = t941*t1419;
  t1505 = t374*t1447;
  t1506 = t1504 + t1505;
  t1582 = -1.*t455*t473*t798;
  t1586 = t606*t1245;
  t1592 = -1.*t478*t1227;
  t1594 = t455*t784*t615;
  t1598 = t1592 + t1594;
  t1605 = t423*t1598;
  t1621 = t1586 + t1605;
  t1625 = t769*t1621;
  t1637 = t1582 + t1625;
  t1641 = -1.*t941*t385*t1637;
  t1644 = -1.*t747*t374*t1637;
  t1649 = t1641 + t1644;
  t1659 = t747*t941*t1637;
  t1660 = -1.*t385*t374*t1637;
  t1666 = t1659 + t1660;
  t1753 = -1.*t423*t1598;
  t1759 = t1284 + t1753;
  t1767 = -1.*t606*t1598;
  t1768 = t1385 + t1767;
  t1766 = t385*t1759;
  t1777 = t747*t798*t1768;
  t1801 = t1766 + t1777;
  t1818 = t747*t1759;
  t1821 = -1.*t798*t385*t1768;
  t1829 = t1818 + t1821;
  t1806 = -1.*t374*t1801;
  t1832 = t941*t1829;
  t1835 = t1806 + t1832;
  t1851 = t941*t1801;
  t1855 = t374*t1829;
  t1892 = t1851 + t1855;
  t1955 = t769*t455*t473;
  t1962 = t798*t1621;
  t1963 = t1955 + t1962;
  t1952 = -1.*t385*t1768;
  t1968 = -1.*t747*t1963;
  t1975 = t1952 + t1968;
  t1983 = t747*t1768;
  t1988 = -1.*t385*t1963;
  t1992 = t1983 + t1988;
  t1980 = t374*t1975;
  t2003 = t941*t1992;
  t2011 = t1980 + t2003;
  t2018 = t941*t1975;
  t2021 = -1.*t374*t1992;
  t2031 = t2018 + t2021;
  t2074 = t385*t1768;
  t2086 = t747*t1963;
  t2106 = t2074 + t2086;
  t2113 = -1.*t374*t2106;
  t2120 = t2113 + t2003;
  t2127 = -1.*t941*t2106;
  t2130 = t2127 + t2021;
  t2123 = -1.*t311*t2120;
  t2159 = t1012*t2120;
  t2189 = t941*t2106;
  t2190 = t374*t1992;
  t2192 = t2189 + t2190;
  t2210 = -1.*t311*t2192;
  t2212 = t2159 + t2210;
  t2214 = t240*t2212;
  t2270 = -1.*t478*t1227*t784;
  t2277 = t455*t615;
  t2279 = t2270 + t2277;
  t2303 = -1.*t455*t478;
  t2306 = -1.*t1227*t784*t615;
  t2310 = t2303 + t2306;
  t2294 = t423*t2279;
  t2311 = -1.*t606*t2310;
  t2314 = t2294 + t2311;
  t2327 = -1.*t769*t473*t1227;
  t2331 = t606*t2279;
  t2333 = t423*t2310;
  t2334 = t2331 + t2333;
  t2347 = t798*t2334;
  t2351 = t2327 + t2347;
  t2325 = t385*t2314;
  t2353 = t747*t2351;
  t2356 = t2325 + t2353;
  t2359 = t747*t2314;
  t2360 = -1.*t385*t2351;
  t2361 = t2359 + t2360;
  t2357 = -1.*t374*t2356;
  t2381 = t941*t2361;
  t2386 = t2357 + t2381;
  t2402 = t941*t2356;
  t2407 = t374*t2361;
  t2412 = t2402 + t2407;
  t2462 = t423*t473*t478*t1227;
  t2479 = -1.*t473*t606*t1227*t615;
  t2483 = t2462 + t2479;
  t2498 = -1.*t769*t1227*t784;
  t2499 = t473*t478*t606*t1227;
  t2503 = t423*t473*t1227*t615;
  t2513 = t2499 + t2503;
  t2514 = t798*t2513;
  t2520 = t2498 + t2514;
  t2490 = t385*t2483;
  t2521 = t747*t2520;
  t2527 = t2490 + t2521;
  t2533 = t747*t2483;
  t2537 = -1.*t385*t2520;
  t2563 = t2533 + t2537;
  t2531 = -1.*t374*t2527;
  t2564 = t941*t2563;
  t2569 = t2531 + t2564;
  t2573 = t941*t2527;
  t2575 = t374*t2563;
  t2576 = t2573 + t2575;
  t2607 = t478*t1227*t784;
  t2614 = -1.*t455*t615;
  t2615 = t2607 + t2614;
  t2627 = -1.*t606*t2615;
  t2628 = t2627 + t2333;
  t2636 = t423*t2615;
  t2638 = t606*t2310;
  t2641 = t2636 + t2638;
  t2633 = t385*t2628;
  t2646 = t747*t798*t2641;
  t2647 = t2633 + t2646;
  t2654 = t747*t2628;
  t2667 = -1.*t798*t385*t2641;
  t2669 = t2654 + t2667;
  t2653 = -1.*t374*t2647;
  t2679 = t941*t2669;
  t2681 = t2653 + t2679;
  t2692 = t941*t2647;
  t2700 = t374*t2669;
  t2713 = t2692 + t2700;
  t2762 = -1.*t473*t798*t1227;
  t2774 = t606*t2615;
  t2778 = t455*t478;
  t2782 = t1227*t784*t615;
  t2789 = t2778 + t2782;
  t2791 = t423*t2789;
  t2798 = t2774 + t2791;
  t2799 = t769*t2798;
  t2802 = t2762 + t2799;
  t2803 = -1.*t941*t385*t2802;
  t2807 = -1.*t747*t374*t2802;
  t2812 = t2803 + t2807;
  t2818 = t747*t941*t2802;
  t2827 = -1.*t385*t374*t2802;
  t2829 = t2818 + t2827;
  t2862 = -1.*t423*t2789;
  t2863 = t2627 + t2862;
  t2867 = -1.*t606*t2789;
  t2868 = t2636 + t2867;
  t2864 = t385*t2863;
  t2873 = t747*t798*t2868;
  t2874 = t2864 + t2873;
  t2887 = t747*t2863;
  t2894 = -1.*t798*t385*t2868;
  t2905 = t2887 + t2894;
  t2882 = -1.*t374*t2874;
  t2908 = t941*t2905;
  t2910 = t2882 + t2908;
  t2922 = t941*t2874;
  t2924 = t374*t2905;
  t2935 = t2922 + t2924;
  t2240 = t311*t2120;
  t2241 = t1012*t2192;
  t2253 = t2240 + t2241;
  t2987 = t769*t473*t1227;
  t2990 = t798*t2798;
  t2991 = t2987 + t2990;
  t2985 = -1.*t385*t2868;
  t2998 = -1.*t747*t2991;
  t2999 = t2985 + t2998;
  t3008 = t747*t2868;
  t3018 = -1.*t385*t2991;
  t3022 = t3008 + t3018;
  t3004 = t374*t2999;
  t3024 = t941*t3022;
  t3027 = t3004 + t3024;
  t3038 = t941*t2999;
  t3039 = -1.*t374*t3022;
  t3040 = t3038 + t3039;
  t3095 = t385*t2868;
  t3103 = t747*t2991;
  t3108 = t3095 + t3103;
  t3110 = -1.*t374*t3108;
  t3117 = t3110 + t3024;
  t3125 = -1.*t941*t3108;
  t3127 = t3125 + t3039;
  t3119 = -1.*t311*t3117;
  t3144 = t1012*t3117;
  t3190 = t941*t3108;
  t3196 = t374*t3022;
  t3197 = t3190 + t3196;
  t3224 = -1.*t311*t3197;
  t3226 = t3144 + t3224;
  t3227 = t240*t3226;
  t3256 = t798*t784;
  t3261 = t473*t478*t606;
  t3265 = t423*t473*t615;
  t3268 = t3261 + t3265;
  t3272 = t769*t3268;
  t3273 = t3256 + t3272;
  t3274 = -1.*t941*t385*t3273;
  t3275 = -1.*t747*t374*t3273;
  t3282 = t3274 + t3275;
  t3284 = t747*t941*t3273;
  t3287 = -1.*t385*t374*t3273;
  t3290 = t3284 + t3287;
  t3360 = -1.*t473*t478*t606;
  t3361 = -1.*t423*t473*t615;
  t3365 = t3360 + t3361;
  t3373 = t423*t473*t478;
  t3389 = -1.*t473*t606*t615;
  t3397 = t3373 + t3389;
  t3371 = t385*t3365;
  t3400 = t747*t798*t3397;
  t3401 = t3371 + t3400;
  t3407 = t747*t3365;
  t3414 = -1.*t798*t385*t3397;
  t3435 = t3407 + t3414;
  t3406 = -1.*t374*t3401;
  t3436 = t941*t3435;
  t3444 = t3406 + t3436;
  t3458 = t941*t3401;
  t3469 = t374*t3435;
  t3471 = t3458 + t3469;
  t3453 = t311*t3444;
  t3474 = t1012*t3471;
  t3479 = t3453 + t3474;
  t3482 = t240*t3479;
  t3484 = t1012*t3444;
  t3488 = -1.*t311*t3471;
  t3489 = t3484 + t3488;
  t3492 = -1.*t1125*t3489;
  t3494 = t3482 + t3492;
  t3504 = -1.*t769*t784;
  t3507 = t798*t3268;
  t3509 = t3504 + t3507;
  t3502 = -1.*t385*t3397;
  t3515 = -1.*t747*t3509;
  t3516 = t3502 + t3515;
  t3522 = t747*t3397;
  t3527 = -1.*t385*t3509;
  t3532 = t3522 + t3527;
  t3518 = t374*t3516;
  t3540 = t941*t3532;
  t3545 = t3518 + t3540;
  t3560 = t941*t3516;
  t3563 = -1.*t374*t3532;
  t3571 = t3560 + t3563;
  t3611 = t385*t3397;
  t3613 = t747*t3509;
  t3616 = t3611 + t3613;
  t3620 = -1.*t374*t3616;
  t3623 = t3620 + t3540;
  t3631 = -1.*t941*t3616;
  t3634 = t3631 + t3563;
  t3629 = -1.*t311*t3623;
  t3656 = t1012*t3623;
  t3670 = t941*t3616;
  t3671 = t374*t3532;
  t3672 = t3670 + t3671;
  t3690 = -1.*t311*t3672;
  t3698 = t3656 + t3690;
  t3700 = t240*t3698;
  t3731 = -1.*t423*t478*t784;
  t3742 = t606*t784*t615;
  t3752 = t3731 + t3742;
  t3755 = -1.*t769*t473;
  t3756 = -1.*t478*t606*t784;
  t3758 = -1.*t423*t784*t615;
  t3770 = t3756 + t3758;
  t3772 = t798*t3770;
  t3778 = t3755 + t3772;
  t3754 = t385*t3752;
  t3782 = t747*t3778;
  t3786 = t3754 + t3782;
  t3796 = t747*t3752;
  t3798 = -1.*t385*t3778;
  t3802 = t3796 + t3798;
  t3790 = -1.*t374*t3786;
  t3803 = t941*t3802;
  t3807 = t3790 + t3803;
  t3810 = t941*t3786;
  t3815 = t374*t3802;
  t3820 = t3810 + t3815;
  t1008 = t311*t1007;
  t1091 = t1012*t1051;
  t1110 = t1008 + t1091;
  t1127 = t1012*t1007;
  t1161 = -1.*t311*t1051;
  t1167 = t1127 + t1161;
  t1501 = t311*t1496;
  t1509 = t1012*t1506;
  t1512 = t1501 + t1509;
  t1551 = t1012*t1496;
  t1554 = -1.*t311*t1506;
  t1555 = t1551 + t1554;
  t1653 = t311*t1649;
  t1671 = t1012*t1666;
  t1672 = t1653 + t1671;
  t1713 = t1012*t1649;
  t1715 = -1.*t311*t1666;
  t1730 = t1713 + t1715;
  t1838 = t311*t1835;
  t1893 = t1012*t1892;
  t1894 = t1838 + t1893;
  t1898 = t1012*t1835;
  t1901 = -1.*t311*t1892;
  t1902 = t1898 + t1901;
  t2017 = -1.*t311*t2011;
  t2035 = t1012*t2031;
  t2038 = t2017 + t2035;
  t2044 = t1012*t2011;
  t2052 = t311*t2031;
  t2060 = t2044 + t2052;
  t2135 = t1012*t2130;
  t2144 = t2123 + t2135;
  t2177 = t311*t2130;
  t2178 = t2159 + t2177;
  t2193 = -1.*t1012*t2192;
  t2199 = t2123 + t2193;
  t3950 = t1125*t2212;
  t2393 = t311*t2386;
  t2419 = t1012*t2412;
  t2427 = t2393 + t2419;
  t2430 = t1012*t2386;
  t2432 = -1.*t311*t2412;
  t2433 = t2430 + t2432;
  t2571 = t311*t2569;
  t2577 = t1012*t2576;
  t2578 = t2571 + t2577;
  t2584 = t1012*t2569;
  t2586 = -1.*t311*t2576;
  t2591 = t2584 + t2586;
  t2690 = t311*t2681;
  t2732 = t1012*t2713;
  t2738 = t2690 + t2732;
  t2742 = t1012*t2681;
  t2744 = -1.*t311*t2713;
  t2753 = t2742 + t2744;
  t2815 = t311*t2812;
  t2830 = t1012*t2829;
  t2832 = t2815 + t2830;
  t2840 = t1012*t2812;
  t2843 = -1.*t311*t2829;
  t2845 = t2840 + t2843;
  t2911 = t311*t2910;
  t2936 = t1012*t2935;
  t2940 = t2911 + t2936;
  t2945 = t1012*t2910;
  t2948 = -1.*t311*t2935;
  t2949 = t2945 + t2948;
  t2254 = t1125*t2253;
  t2258 = t2254 + t2214;
  t3033 = -1.*t311*t3027;
  t3041 = t1012*t3040;
  t3044 = t3033 + t3041;
  t3055 = t1012*t3027;
  t3070 = t311*t3040;
  t3083 = t3055 + t3070;
  t3132 = t1012*t3127;
  t3134 = t3119 + t3132;
  t3149 = t311*t3127;
  t3151 = t3144 + t3149;
  t3211 = -1.*t1012*t3197;
  t3217 = t3119 + t3211;
  t3237 = t311*t3117;
  t3242 = t1012*t3197;
  t3245 = t3237 + t3242;
  t4196 = t1125*t3226;
  t3283 = t311*t3282;
  t3298 = t1012*t3290;
  t3303 = t3283 + t3298;
  t3314 = t1012*t3282;
  t3318 = -1.*t311*t3290;
  t3323 = t3314 + t3318;
  t4256 = t1125*t3479;
  t4257 = t240*t3489;
  t4260 = t4256 + t4257;
  t3558 = -1.*t311*t3545;
  t3572 = t1012*t3571;
  t3573 = t3558 + t3572;
  t3580 = t1012*t3545;
  t3596 = t311*t3571;
  t3600 = t3580 + t3596;
  t3644 = t1012*t3634;
  t3645 = t3629 + t3644;
  t3657 = t311*t3634;
  t3659 = t3656 + t3657;
  t3679 = -1.*t1012*t3672;
  t3684 = t3629 + t3679;
  t3714 = t311*t3623;
  t3716 = t1012*t3672;
  t3721 = t3714 + t3716;
  t4299 = t1125*t3698;
  t3809 = t311*t3807;
  t3829 = t1012*t3820;
  t3830 = t3809 + t3829;
  t3834 = t1012*t3807;
  t3837 = -1.*t311*t3820;
  t3840 = t3834 + t3837;
  t4487 = -1.*t423;
  t4489 = 1. + t4487;
  t4530 = -1.*t769;
  t4532 = 1. + t4530;
  t4577 = -1.*t747;
  t4583 = 1. + t4577;
  t4632 = -1.*t941;
  t4635 = 1. + t4632;
  t4675 = -1.*t1012;
  t4676 = 1. + t4675;
  t4733 = -1.*t1125;
  t4750 = 1. + t4733;
  t3859 = t1125*t1110;
  t3866 = t240*t1167;
  t3868 = t3859 + t3866;
  t4549 = -0.135*t4489;
  t4553 = 0.07996*t606;
  t4558 = 0. + t4549 + t4553;
  t4491 = 0.07996*t4489;
  t4514 = 0.135*t606;
  t4515 = 0. + t4491 + t4514;
  t4587 = -0.01004*t4583;
  t4596 = 0.08055*t385;
  t4600 = 0. + t4587 + t4596;
  t4561 = -0.135*t4532;
  t4572 = 0.08055*t798;
  t4574 = 0. + t4561 + t4572;
  t4623 = -0.08055*t4583;
  t4628 = -0.01004*t385;
  t4630 = 0. + t4623 + t4628;
  t4637 = -0.08055*t4635;
  t4645 = -0.13004*t374;
  t4649 = 0. + t4637 + t4645;
  t4653 = -0.13004*t4635;
  t4664 = 0.08055*t374;
  t4668 = 0. + t4653 + t4664;
  t4697 = -0.19074*t4676;
  t4699 = 0.03315*t311;
  t4703 = 0. + t4697 + t4699;
  t4715 = -0.03315*t4676;
  t4719 = -0.19074*t311;
  t4731 = 0. + t4715 + t4719;
  t4756 = -0.01315*t4750;
  t4758 = -0.62554*t240;
  t4766 = 0. + t4756 + t4758;
  t4777 = -0.62554*t4750;
  t4784 = 0.01315*t240;
  t4792 = 0. + t4777 + t4784;
  t3877 = t1125*t1512;
  t3880 = t240*t1555;
  t3883 = t3877 + t3880;
  t4542 = -0.135*t798;
  t3890 = t1125*t1672;
  t3892 = t240*t1730;
  t3893 = t3890 + t3892;
  t3899 = t1125*t1894;
  t3900 = t240*t1902;
  t3908 = t3899 + t3900;
  t3918 = t240*t2038;
  t3922 = t1125*t2060;
  t3923 = t3918 + t3922;
  t3934 = t240*t2144;
  t3936 = t1125*t2178;
  t3945 = t3934 + t3936;
  t3947 = t240*t2199;
  t3955 = t3947 + t3950;
  t3965 = -1.*t240*t2253;
  t3981 = t3965 + t3950;
  t5420 = -1.*t240*t2212;
  t4540 = -0.08055*t4532;
  t4544 = 0. + t4540 + t4542;
  t3992 = t1125*t2427;
  t3997 = t240*t2433;
  t4014 = t3992 + t3997;
  t4032 = t1125*t2578;
  t4033 = t240*t2591;
  t4034 = t4032 + t4033;
  t4063 = t1125*t2738;
  t4064 = t240*t2753;
  t4072 = t4063 + t4064;
  t4928 = -0.135*t769;
  t4942 = -0.08055*t798;
  t4960 = t4928 + t4942;
  t4966 = 0.08055*t769;
  t4973 = t4966 + t4542;
  t4093 = t1125*t2832;
  t4098 = t240*t2845;
  t4108 = t4093 + t4098;
  t5089 = 0.135*t423;
  t5091 = t5089 + t4553;
  t5102 = 0.07996*t423;
  t5104 = -0.135*t606;
  t5110 = t5102 + t5104;
  t4129 = t1125*t2940;
  t4130 = t240*t2949;
  t4134 = t4129 + t4130;
  t5214 = 0.08055*t747;
  t5216 = t5214 + t4628;
  t5223 = -0.01004*t747;
  t5224 = -0.08055*t385;
  t5229 = t5223 + t5224;
  t4141 = t240*t3044;
  t4146 = t1125*t3083;
  t4149 = t4141 + t4146;
  t5294 = -0.13004*t941;
  t5304 = -0.08055*t374;
  t5307 = t5294 + t5304;
  t5311 = 0.08055*t941;
  t5317 = t5311 + t4645;
  t4162 = t240*t3134;
  t4173 = t1125*t3151;
  t4180 = t4162 + t4173;
  t5376 = 0.03315*t1012;
  t5378 = t5376 + t4719;
  t5391 = -0.19074*t1012;
  t5396 = -0.03315*t311;
  t5398 = t5391 + t5396;
  t4191 = t240*t3217;
  t4198 = t4191 + t4196;
  t5450 = -0.62554*t1125;
  t5452 = -0.01315*t240;
  t5460 = t5450 + t5452;
  t5463 = 0.01315*t1125;
  t5464 = t5463 + t4758;
  t4215 = -1.*t240*t3245;
  t4218 = t4215 + t4196;
  t6162 = -1.*t240*t3226;
  t4236 = t1125*t3303;
  t4243 = t240*t3323;
  t4247 = t4236 + t4243;
  t6292 = t4600*t3365;
  t6294 = -0.1305*t769*t3397;
  t6296 = t4574*t3397;
  t6297 = t798*t4630*t3397;
  t6298 = t4649*t3401;
  t6299 = t4668*t3435;
  t6302 = t4703*t3444;
  t6308 = t4731*t3471;
  t6310 = t4766*t3479;
  t6324 = t4792*t3489;
  t6334 = -1.*t240*t3479;
  t6337 = t1125*t3489;
  t6338 = t6334 + t6337;
  t6339 = -0.62554*t6338;
  t6340 = -0.01315*t4260;
  t4274 = t240*t3573;
  t4275 = t1125*t3600;
  t4289 = t4274 + t4275;
  t4291 = t240*t3645;
  t4293 = t1125*t3659;
  t4294 = t4291 + t4293;
  t4298 = t240*t3684;
  t4300 = t4298 + t4299;
  t4315 = -1.*t240*t3721;
  t4324 = t4315 + t4299;
  t6485 = -1.*t240*t3698;
  t4329 = t1125*t3830;
  t4333 = t240*t3840;
  t4349 = t4329 + t4333;
  p_output1[0]=(t240*t2427 - 1.*t1125*t2433)*var2[3] + (-1.*t1125*t1167 + t1110*t240)*var2[4] + (-1.*t1125*t1555 + t1512*t240)*var2[5] + (-1.*t1125*t1902 + t1894*t240)*var2[13] + (-1.*t1125*t1730 + t1672*t240)*var2[14] + (-1.*t1125*t2038 + t2060*t240)*var2[15] + (-1.*t1125*t2144 + t2178*t240)*var2[16] + (-1.*t1125*t2199 + t2214)*var2[17] + t2258*var2[18];
  p_output1[1]=(-1.*t1125*t2212 + t2253*t240)*var2[3] + (t240*t2578 - 1.*t1125*t2591)*var2[4] + (t240*t2738 - 1.*t1125*t2753)*var2[5] + (t240*t2940 - 1.*t1125*t2949)*var2[13] + (t240*t2832 - 1.*t1125*t2845)*var2[14] + (-1.*t1125*t3044 + t240*t3083)*var2[15] + (-1.*t1125*t3134 + t240*t3151)*var2[16] + (-1.*t1125*t3217 + t3227)*var2[17] + (t3227 + t1125*t3245)*var2[18];
  p_output1[2]=(t240*t3830 - 1.*t1125*t3840)*var2[4] + t3494*var2[5] + t3494*var2[13] + (t240*t3303 - 1.*t1125*t3323)*var2[14] + (-1.*t1125*t3573 + t240*t3600)*var2[15] + (-1.*t1125*t3645 + t240*t3659)*var2[16] + (-1.*t1125*t3684 + t3700)*var2[17] + (t3700 + t1125*t3721)*var2[18];
  p_output1[3]=0;
  p_output1[4]=t4014*var2[3] + t3868*var2[4] + t3883*var2[5] + t3908*var2[13] + t3893*var2[14] + t3923*var2[15] + t3945*var2[16] + t3955*var2[17] + t3981*var2[18];
  p_output1[5]=t2258*var2[3] + t4034*var2[4] + t4072*var2[5] + t4134*var2[13] + t4108*var2[14] + t4149*var2[15] + t4180*var2[16] + t4198*var2[17] + t4218*var2[18];
  p_output1[6]=t4349*var2[4] + t4260*var2[5] + t4260*var2[13] + t4247*var2[14] + t4289*var2[15] + t4294*var2[16] + t4300*var2[17] + t4324*var2[18];
  p_output1[7]=0;
  p_output1[8]=(t2762 - 1.*t2334*t769)*var2[3] + (-1.*t455*t784*t798 - 1.*t769*t807)*var2[4] - 1.*t1394*t769*var2[5] - 1.*t1768*t769*var2[13] + t1963*var2[14];
  p_output1[9]=(-1.*t1621*t769 + t455*t473*t798)*var2[3] + (-1.*t2513*t769 - 1.*t1227*t784*t798)*var2[4] - 1.*t2641*t769*var2[5] - 1.*t2868*t769*var2[13] + t2991*var2[14];
  p_output1[10]=(-1.*t3770*t769 - 1.*t473*t798)*var2[4] - 1.*t3397*t769*var2[5] - 1.*t3397*t769*var2[13] + t3509*var2[14];
  p_output1[11]=0;
  p_output1[12]=var2[0] + (-0.62554*(-1.*t240*t2427 + t1125*t2433) - 0.01315*t4014 + t2279*t4515 + t2310*t4558 + t2334*t4574 + t2314*t4600 + t2351*t4630 + t2356*t4649 + t2361*t4668 + t2386*t4703 - 1.*t1227*t4544*t473 + t2412*t4731 + t2427*t4766 + t2433*t4792 - 0.1305*(t2334*t769 + t1227*t473*t798))*var2[3] + (-0.62554*(t1125*t1167 - 1.*t1110*t240) - 0.01315*t3868 + t1007*t4703 + t1051*t4731 + t1110*t4766 + t4515*t455*t473*t478 + t1167*t4792 + t455*t4558*t473*t615 + t4600*t694 - 1.*t4544*t455*t784 + t4574*t807 - 0.1305*(t455*t784*t798 + t769*t807) + t4630*t836 + t4649*t891 + t4668*t976)*var2[4] + (-0.62554*(t1125*t1555 - 1.*t1512*t240) - 0.01315*t3883 + t1322*t4515 + t1245*t4558 + t1394*t4574 + t1342*t4600 + t1419*t4649 + t1447*t4668 + t1496*t4703 + t1506*t4731 + t1512*t4766 + t1555*t4792 - 0.1305*t1394*t769 + t1394*t4630*t798)*var2[5] + (-0.62554*(t1125*t1902 - 1.*t1894*t240) - 0.01315*t3908 + t1768*t4574 + t1759*t4600 + t1801*t4649 + t1829*t4668 + t1835*t4703 + t1892*t4731 + t1894*t4766 + t1902*t4792 + t1245*t5091 + t1598*t5110 - 0.1305*t1768*t769 + t1768*t4630*t798)*var2[13] + (-0.62554*(t1125*t1730 - 1.*t1672*t240) - 0.01315*t3893 + t1637*t4630 - 1.*t1637*t385*t4668 + t1649*t4703 + t1666*t4731 + t1672*t4766 + t1730*t4792 + t455*t473*t4960 + t1621*t4973 + t1637*t4649*t747 - 0.1305*(-1.*t455*t473*t769 - 1.*t1621*t798))*var2[14] + (-0.62554*(t1125*t2038 - 1.*t2060*t240) - 0.01315*t3923 + t1992*t4649 + t1975*t4668 + t2031*t4703 + t2011*t4731 + t2060*t4766 + t2038*t4792 + t1768*t5216 + t1963*t5229)*var2[15] + (-0.62554*(t1125*t2144 - 1.*t2178*t240) - 0.01315*t3945 + t2130*t4703 + t2120*t4731 + t2178*t4766 + t2144*t4792 + t2106*t5307 + t1992*t5317)*var2[16] + (-0.01315*t3955 + t2212*t4766 + t2199*t4792 + t2120*t5378 + t2192*t5398 - 0.62554*(t1125*t2199 + t5420))*var2[17] + (-0.01315*t3981 - 0.62554*(-1.*t1125*t2253 + t5420) + t2253*t5460 + t2212*t5464)*var2[18];
  p_output1[13]=var2[1] + (-0.1305*t1637 - 0.01315*t2258 - 0.62554*t3981 + t1245*t4515 + t1598*t4558 + t1621*t4574 + t1768*t4600 + t1963*t4630 + t2106*t4649 + t1992*t4668 + t2120*t4703 + t4544*t455*t473 + t2192*t4731 + t2253*t4766 + t2212*t4792)*var2[3] + (-0.62554*(-1.*t240*t2578 + t1125*t2591) - 0.01315*t4034 + t2513*t4574 + t2483*t4600 + t2520*t4630 + t2527*t4649 + t2563*t4668 + t2569*t4703 + t2576*t4731 + t2578*t4766 + t1227*t4515*t473*t478 + t2591*t4792 + t1227*t4558*t473*t615 - 1.*t1227*t4544*t784 - 0.1305*(t2513*t769 + t1227*t784*t798))*var2[4] + (-0.62554*(-1.*t240*t2738 + t1125*t2753) - 0.01315*t4072 + t2310*t4515 + t2615*t4558 + t2641*t4574 + t2628*t4600 + t2647*t4649 + t2669*t4668 + t2681*t4703 + t2713*t4731 + t2738*t4766 + t2753*t4792 - 0.1305*t2641*t769 + t2641*t4630*t798)*var2[5] + (-0.62554*(-1.*t240*t2940 + t1125*t2949) - 0.01315*t4134 + t2868*t4574 + t2863*t4600 + t2874*t4649 + t2905*t4668 + t2910*t4703 + t2935*t4731 + t2940*t4766 + t2949*t4792 + t2615*t5091 + t2789*t5110 - 0.1305*t2868*t769 + t2868*t4630*t798)*var2[13] + (-0.62554*(-1.*t240*t2832 + t1125*t2845) - 0.01315*t4108 + t2802*t4630 - 1.*t2802*t385*t4668 + t2812*t4703 + t2829*t4731 + t2832*t4766 + t2845*t4792 + t1227*t473*t4960 + t2798*t4973 + t2802*t4649*t747 - 0.1305*(t2327 - 1.*t2798*t798))*var2[14] + (-0.62554*(t1125*t3044 - 1.*t240*t3083) - 0.01315*t4149 + t3022*t4649 + t2999*t4668 + t3040*t4703 + t3027*t4731 + t3083*t4766 + t3044*t4792 + t2868*t5216 + t2991*t5229)*var2[15] + (-0.62554*(t1125*t3134 - 1.*t240*t3151) - 0.01315*t4180 + t3127*t4703 + t3117*t4731 + t3151*t4766 + t3134*t4792 + t3108*t5307 + t3022*t5317)*var2[16] + (-0.01315*t4198 + t3226*t4766 + t3217*t4792 + t3117*t5378 + t3197*t5398 - 0.62554*(t1125*t3217 + t6162))*var2[17] + (-0.01315*t4218 + t3245*t5460 + t3226*t5464 - 0.62554*(-1.*t1125*t3245 + t6162))*var2[18];
  p_output1[14]=var2[2] + (-0.62554*(-1.*t240*t3830 + t1125*t3840) - 0.01315*t4349 + t3770*t4574 + t3752*t4600 + t3778*t4630 + t3786*t4649 + t3802*t4668 + t3807*t4703 - 1.*t4544*t473 + t3820*t4731 + t3830*t4766 + t3840*t4792 - 1.*t4515*t478*t784 - 1.*t4558*t615*t784 - 0.1305*(t3770*t769 + t473*t798))*var2[4] + (t4558*t473*t478 - 1.*t4515*t473*t615 + t6292 + t6294 + t6296 + t6297 + t6298 + t6299 + t6302 + t6308 + t6310 + t6324 + t6339 + t6340)*var2[5] + (t473*t478*t5091 + t473*t5110*t615 + t6292 + t6294 + t6296 + t6297 + t6298 + t6299 + t6302 + t6308 + t6310 + t6324 + t6339 + t6340)*var2[13] + (-0.62554*(-1.*t240*t3303 + t1125*t3323) - 0.01315*t4247 + t3273*t4630 - 1.*t3273*t385*t4668 + t3282*t4703 + t3290*t4731 + t3303*t4766 + t3323*t4792 + t3268*t4973 + t3273*t4649*t747 - 1.*t4960*t784 - 0.1305*(t769*t784 - 1.*t3268*t798))*var2[14] + (-0.62554*(t1125*t3573 - 1.*t240*t3600) - 0.01315*t4289 + t3532*t4649 + t3516*t4668 + t3571*t4703 + t3545*t4731 + t3600*t4766 + t3573*t4792 + t3397*t5216 + t3509*t5229)*var2[15] + (-0.62554*(t1125*t3645 - 1.*t240*t3659) - 0.01315*t4294 + t3634*t4703 + t3623*t4731 + t3659*t4766 + t3645*t4792 + t3616*t5307 + t3532*t5317)*var2[16] + (-0.01315*t4300 + t3698*t4766 + t3684*t4792 + t3623*t5378 + t3672*t5398 - 0.62554*(t1125*t3684 + t6485))*var2[17] + (-0.01315*t4324 + t3721*t5460 + t3698*t5464 - 0.62554*(-1.*t1125*t3721 + t6485))*var2[18];
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

#include "dT_RightAnkleJoint_mex.hh"

namespace SymExpression
{

void dT_RightAnkleJoint_mex_raw(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}

}

#endif // MATLAB_MEX_FILE