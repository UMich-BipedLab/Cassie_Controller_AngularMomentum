/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:37:42 GMT-04:00
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
static void output1(double *p_output1,const double *var1)
{
  double t3;
  double t96;
  double t130;
  double t177;
  double t178;
  double t182;
  double t196;
  double t235;
  double t185;
  double t197;
  double t201;
  double t214;
  double t219;
  double t224;
  double t225;
  double t253;
  double t349;
  double t353;
  double t354;
  double t372;
  double t259;
  double t263;
  double t273;
  double t288;
  double t289;
  double t290;
  double t369;
  double t384;
  double t385;
  double t389;
  double t390;
  double t396;
  double t401;
  double t402;
  double t404;
  double t416;
  double t424;
  double t450;
  double t455;
  double t467;
  double t474;
  double t427;
  double t433;
  double t436;
  double t472;
  double t481;
  double t483;
  double t484;
  double t489;
  double t498;
  double t506;
  double t510;
  double t512;
  double t516;
  double t524;
  double t556;
  double t559;
  double t562;
  double t570;
  double t530;
  double t538;
  double t547;
  double t565;
  double t571;
  double t572;
  double t573;
  double t574;
  double t585;
  double t588;
  double t591;
  double t592;
  double t596;
  double t599;
  double t616;
  double t619;
  double t622;
  double t630;
  double t605;
  double t606;
  double t607;
  double t623;
  double t640;
  double t645;
  double t647;
  double t649;
  double t652;
  double t653;
  double t659;
  double t660;
  double t661;
  double t664;
  double t703;
  double t704;
  double t709;
  double t717;
  double t676;
  double t681;
  double t684;
  double t716;
  double t720;
  double t725;
  double t726;
  double t729;
  double t731;
  double t745;
  double t747;
  double t749;
  double t753;
  double t754;
  double t783;
  double t785;
  double t791;
  double t793;
  double t758;
  double t760;
  double t766;
  double t175;
  double t837;
  double t843;
  double t847;
  double t858;
  double t851;
  double t861;
  double t862;
  double t866;
  double t869;
  double t871;
  double t877;
  double t885;
  double t897;
  double t898;
  double t900;
  double t887;
  double t890;
  double t894;
  double t910;
  double t912;
  double t916;
  double t923;
  double t918;
  double t924;
  double t931;
  double t934;
  double t935;
  double t939;
  double t940;
  double t941;
  double t944;
  double t946;
  double t952;
  double t955;
  double t956;
  double t957;
  double t969;
  double t971;
  double t972;
  double t983;
  double t978;
  double t986;
  double t987;
  double t990;
  double t991;
  double t992;
  double t998;
  double t999;
  double t1008;
  double t1009;
  double t1012;
  double t1034;
  double t1035;
  double t1036;
  double t1043;
  double t1021;
  double t1022;
  double t1023;
  double t1041;
  double t1046;
  double t1053;
  double t1056;
  double t1057;
  double t1058;
  double t1060;
  double t1061;
  double t1062;
  double t1063;
  double t1064;
  double t1109;
  double t1116;
  double t1119;
  double t1122;
  double t1070;
  double t1084;
  double t1089;
  double t1120;
  double t1123;
  double t1125;
  double t1126;
  double t1130;
  double t1135;
  double t1139;
  double t1140;
  double t1141;
  double t1145;
  double t1147;
  double t1170;
  double t1174;
  double t1179;
  double t1190;
  double t1151;
  double t1155;
  double t1158;
  double t1187;
  double t1193;
  double t1197;
  double t1199;
  double t1202;
  double t1203;
  double t1209;
  double t1210;
  double t1212;
  double t1213;
  double t1214;
  double t1246;
  double t1253;
  double t1256;
  double t1272;
  double t1224;
  double t1226;
  double t1227;
  double t1322;
  double t1325;
  double t1327;
  double t1328;
  double t1331;
  double t1339;
  double t1341;
  double t1342;
  double t1355;
  double t1368;
  double t1385;
  double t1404;
  double t1405;
  double t1406;
  double t1424;
  double t1428;
  double t1430;
  double t1440;
  double t1446;
  double t1447;
  double t1456;
  double t1458;
  double t1461;
  double t1468;
  double t1473;
  double t1480;
  double t1487;
  double t1492;
  double t1494;
  double t1495;
  double t1505;
  double t1508;
  double t1512;
  double t1521;
  double t1523;
  double t792;
  double t795;
  double t796;
  double t1526;
  double t1527;
  double t1530;
  double t801;
  double t803;
  double t805;
  double t1541;
  double t1545;
  double t1546;
  double t1595;
  double t1597;
  double t1598;
  double t1599;
  double t1601;
  double t1621;
  double t1622;
  double t1624;
  double t1643;
  double t1664;
  double t1667;
  double t1671;
  double t1673;
  double t1676;
  double t1683;
  double t1689;
  double t1690;
  double t1702;
  double t1704;
  double t1705;
  double t1706;
  double t1708;
  double t1712;
  double t1715;
  double t1717;
  double t1743;
  double t1753;
  double t1760;
  double t1762;
  double t1763;
  double t1772;
  double t1776;
  double t1782;
  double t1792;
  double t1793;
  double t1271;
  double t1277;
  double t1281;
  double t1795;
  double t1796;
  double t1804;
  double t1288;
  double t1290;
  double t1293;
  double t1810;
  double t1814;
  double t1815;
  double t1612;
  double t1627;
  double t1898;
  double t1899;
  double t1900;
  double t1901;
  double t1903;
  double t1904;
  double t1905;
  double t1636;
  double t1641;
  double t1650;
  double t1661;
  double t1681;
  double t1693;
  double t1701;
  double t1709;
  double t1720;
  double t1735;
  double t1766;
  double t1783;
  double t1791;
  double t1809;
  double t1817;
  double t1832;
  double t1839;
  double t1841;
  double t1846;
  double t1855;
  double t1862;
  double t1864;
  double t1868;
  double t1870;
  double t1875;
  double t1878;
  double t1963;
  double t1967;
  double t1970;
  double t1956;
  double t1957;
  double t1961;
  double t1962;
  double t1975;
  double t1980;
  double t1984;
  double t1987;
  double t1992;
  double t1997;
  double t2007;
  double t2008;
  double t2015;
  double t2038;
  double t2050;
  double t2054;
  double t2060;
  double t2061;
  double t2064;
  double t2068;
  double t2069;
  double t2070;
  double t2084;
  double t2087;
  double t2088;
  double t2091;
  double t2094;
  double t2097;
  double t2099;
  double t2107;
  double t2133;
  double t2134;
  double t2135;
  double t2138;
  double t2140;
  double t2146;
  double t2147;
  double t2148;
  double t2208;
  double t2215;
  double t2217;
  double t2219;
  double t2223;
  double t2224;
  double t2225;
  double t2232;
  double t2240;
  double t2244;
  double t2250;
  double t2252;
  double t2254;
  double t2262;
  double t2263;
  double t2268;
  double t2281;
  double t2283;
  double t2284;
  double t2291;
  double t2292;
  double t2300;
  double t2301;
  double t2303;
  double t2320;
  double t2321;
  double t2323;
  double t2324;
  double t2332;
  double t2337;
  double t2341;
  double t2354;
  double t2360;
  double t2361;
  double t2362;
  double t2365;
  double t2371;
  double t2380;
  double t2383;
  double t2386;
  double t2424;
  double t2427;
  double t2428;
  double t2415;
  double t2417;
  double t2421;
  double t2433;
  double t2434;
  double t2437;
  double t2438;
  double t2446;
  double t2454;
  double t2461;
  double t2466;
  double t2482;
  double t2483;
  double t2484;
  double t2487;
  double t2495;
  double t2499;
  double t2502;
  double t2503;
  double t2521;
  double t2522;
  double t2525;
  double t2528;
  double t2531;
  double t2535;
  double t2537;
  double t2544;
  double t2608;
  double t2610;
  double t2612;
  double t2589;
  double t2593;
  double t2595;
  double t2604;
  double t2605;
  double t2607;
  double t2613;
  double t2616;
  double t2620;
  double t2624;
  double t2625;
  double t2637;
  double t2643;
  double t2645;
  double t2649;
  double t2652;
  double t2659;
  double t2660;
  double t2662;
  double t2704;
  double t2718;
  double t2720;
  double t2698;
  double t2699;
  double t2703;
  double t2722;
  double t2726;
  double t2729;
  double t2733;
  double t2734;
  double t2736;
  double t2740;
  double t2741;
  double t2758;
  double t2798;
  double t2800;
  double t2802;
  double t2768;
  double t1335;
  double t1343;
  double t2839;
  double t2849;
  double t2852;
  double t2855;
  double t2856;
  double t2858;
  double t2859;
  double t1352;
  double t1353;
  double t1358;
  double t1380;
  double t1420;
  double t1435;
  double t1439;
  double t1459;
  double t1474;
  double t1479;
  double t1504;
  double t1513;
  double t1517;
  double t1539;
  double t1549;
  double t1557;
  double t1559;
  double t1564;
  double t1567;
  double t1568;
  double t1569;
  double t1572;
  double t1573;
  double t1580;
  double t1585;
  double t1586;
  double t2910;
  double t2914;
  double t2915;
  double t2920;
  double t2922;
  double t2923;
  double t2888;
  double t2889;
  double t2892;
  double t2895;
  double t2905;
  double t2909;
  double t2919;
  double t2939;
  double t2943;
  double t2944;
  double t2934;
  double t2962;
  double t2964;
  double t2969;
  double t2972;
  double t2978;
  double t2984;
  double t2987;
  double t2988;
  double t3001;
  double t3008;
  double t3009;
  double t3015;
  double t3016;
  double t3023;
  double t3025;
  double t3026;
  double t3033;
  double t3036;
  double t3037;
  double t3038;
  double t3041;
  double t3047;
  double t3049;
  double t3051;
  double t3121;
  double t3122;
  double t3124;
  double t3102;
  double t3103;
  double t3110;
  double t3112;
  double t3118;
  double t3119;
  double t3129;
  double t3130;
  double t3132;
  double t3141;
  double t3149;
  double t3150;
  double t3153;
  double t3160;
  double t3167;
  double t3172;
  double t3174;
  double t3180;
  double t3186;
  double t3189;
  double t3190;
  double t3210;
  double t3212;
  double t3216;
  double t3220;
  double t3226;
  double t3232;
  double t3234;
  double t3235;
  double t3242;
  double t3243;
  double t3246;
  double t3253;
  double t3255;
  double t3261;
  double t3269;
  double t3270;
  double t3342;
  double t3343;
  double t3344;
  double t3329;
  double t3332;
  double t3337;
  double t3351;
  double t3352;
  double t3354;
  double t3361;
  double t3366;
  double t3370;
  double t3376;
  double t3377;
  double t3389;
  double t3391;
  double t3396;
  double t3399;
  double t3401;
  double t3407;
  double t3408;
  double t3409;
  double t3416;
  double t3421;
  double t3427;
  double t3428;
  double t3433;
  double t3439;
  double t3446;
  double t3448;
  double t3505;
  double t3507;
  double t3510;
  double t3490;
  double t3492;
  double t3495;
  double t3498;
  double t3500;
  double t3501;
  double t3511;
  double t3516;
  double t3517;
  double t3522;
  double t3525;
  double t3535;
  double t3537;
  double t3538;
  double t3539;
  double t3540;
  double t3544;
  double t3547;
  double t3548;
  double t3597;
  double t3602;
  double t3605;
  double t3584;
  double t3593;
  double t3595;
  double t3606;
  double t3610;
  double t3613;
  double t3614;
  double t3615;
  double t3619;
  double t3627;
  double t3629;
  double t3639;
  double t3672;
  double t3674;
  double t3676;
  double t3645;
  t3 = Cos(var1[4]);
  t96 = Cos(var1[5]);
  t130 = Sin(var1[4]);
  t177 = Cos(var1[13]);
  t178 = -1.*t177;
  t182 = 1. + t178;
  t196 = Sin(var1[13]);
  t235 = Sin(var1[5]);
  t185 = 0.07996*t182;
  t197 = 0.135*t196;
  t201 = t185 + t197;
  t214 = -1.*t96*t201*t130;
  t219 = -0.135*t182;
  t224 = 0.07996*t196;
  t225 = t219 + t224;
  t253 = -1.*t225*t130*t235;
  t349 = Cos(var1[14]);
  t353 = -1.*t349;
  t354 = 1. + t353;
  t372 = Sin(var1[14]);
  t259 = -1.*t96*t196*t130;
  t263 = -1.*t177*t130*t235;
  t273 = t259 + t263;
  t288 = -1.*t177*t96*t130;
  t289 = t196*t130*t235;
  t290 = t288 + t289;
  t369 = -0.08055*t354;
  t384 = -0.135*t372;
  t385 = t369 + t384;
  t389 = -1.*t3*t385;
  t390 = -0.135*t354;
  t396 = 0.08055*t372;
  t401 = t390 + t396;
  t402 = t401*t273;
  t404 = t3*t372;
  t416 = t349*t273;
  t424 = t404 + t416;
  t450 = Cos(var1[15]);
  t455 = -1.*t450;
  t467 = 1. + t455;
  t474 = Sin(var1[15]);
  t427 = -1.*t349*t3;
  t433 = t372*t273;
  t436 = t427 + t433;
  t472 = -0.01004*t467;
  t481 = 0.08055*t474;
  t483 = t472 + t481;
  t484 = t483*t290;
  t489 = -0.08055*t467;
  t498 = -0.01004*t474;
  t506 = t489 + t498;
  t510 = t506*t436;
  t512 = t474*t290;
  t516 = t450*t436;
  t524 = t512 + t516;
  t556 = Cos(var1[16]);
  t559 = -1.*t556;
  t562 = 1. + t559;
  t570 = Sin(var1[16]);
  t530 = t450*t290;
  t538 = -1.*t474*t436;
  t547 = t530 + t538;
  t565 = -0.08055*t562;
  t571 = -0.13004*t570;
  t572 = t565 + t571;
  t573 = t572*t524;
  t574 = -0.13004*t562;
  t585 = 0.08055*t570;
  t588 = t574 + t585;
  t591 = t588*t547;
  t592 = -1.*t570*t524;
  t596 = t556*t547;
  t599 = t592 + t596;
  t616 = Cos(var1[17]);
  t619 = -1.*t616;
  t622 = 1. + t619;
  t630 = Sin(var1[17]);
  t605 = t556*t524;
  t606 = t570*t547;
  t607 = t605 + t606;
  t623 = -0.19074*t622;
  t640 = 0.03315*t630;
  t645 = t623 + t640;
  t647 = t645*t599;
  t649 = -0.03315*t622;
  t652 = -0.19074*t630;
  t653 = t649 + t652;
  t659 = t653*t607;
  t660 = t630*t599;
  t661 = t616*t607;
  t664 = t660 + t661;
  t703 = Cos(var1[18]);
  t704 = -1.*t703;
  t709 = 1. + t704;
  t717 = Sin(var1[18]);
  t676 = t616*t599;
  t681 = -1.*t630*t607;
  t684 = t676 + t681;
  t716 = -0.01315*t709;
  t720 = -0.62554*t717;
  t725 = t716 + t720;
  t726 = t725*t664;
  t729 = -0.62554*t709;
  t731 = 0.01315*t717;
  t745 = t729 + t731;
  t747 = t745*t684;
  t749 = -1.*t717*t664;
  t753 = t703*t684;
  t754 = t749 + t753;
  t783 = Cos(var1[19]);
  t785 = -1.*t783;
  t791 = 1. + t785;
  t793 = Sin(var1[19]);
  t758 = t703*t664;
  t760 = t717*t684;
  t766 = t758 + t760;
  t175 = 0.05485*t3;
  t837 = Cos(var1[6]);
  t843 = -1.*t837;
  t847 = 1. + t843;
  t858 = Sin(var1[6]);
  t851 = 0.07996*t847;
  t861 = -0.135*t858;
  t862 = t851 + t861;
  t866 = -1.*t96*t130*t862;
  t869 = 0.135*t847;
  t871 = 0.07996*t858;
  t877 = t869 + t871;
  t885 = -1.*t130*t235*t877;
  t897 = -1.*t96*t837*t130;
  t898 = t130*t235*t858;
  t900 = t897 + t898;
  t887 = -1.*t837*t130*t235;
  t890 = -1.*t96*t130*t858;
  t894 = t887 + t890;
  t910 = Cos(var1[7]);
  t912 = -1.*t910;
  t916 = 1. + t912;
  t923 = Sin(var1[7]);
  t918 = 0.135*t916;
  t924 = 0.08055*t923;
  t931 = t918 + t924;
  t934 = t894*t931;
  t935 = -0.08055*t916;
  t939 = 0.135*t923;
  t940 = t935 + t939;
  t941 = -1.*t3*t940;
  t944 = t910*t894;
  t946 = t3*t923;
  t952 = t944 + t946;
  t955 = -1.*t3*t910;
  t956 = t894*t923;
  t957 = t955 + t956;
  t969 = Cos(var1[8]);
  t971 = -1.*t969;
  t972 = 1. + t971;
  t983 = Sin(var1[8]);
  t978 = -0.08055*t972;
  t986 = -0.01004*t983;
  t987 = t978 + t986;
  t990 = t957*t987;
  t991 = -0.01004*t972;
  t992 = 0.08055*t983;
  t998 = t991 + t992;
  t999 = t900*t998;
  t1008 = t969*t957;
  t1009 = t900*t983;
  t1012 = t1008 + t1009;
  t1034 = Cos(var1[9]);
  t1035 = -1.*t1034;
  t1036 = 1. + t1035;
  t1043 = Sin(var1[9]);
  t1021 = t969*t900;
  t1022 = -1.*t957*t983;
  t1023 = t1021 + t1022;
  t1041 = -0.08055*t1036;
  t1046 = -0.13004*t1043;
  t1053 = t1041 + t1046;
  t1056 = t1053*t1012;
  t1057 = -0.13004*t1036;
  t1058 = 0.08055*t1043;
  t1060 = t1057 + t1058;
  t1061 = t1060*t1023;
  t1062 = -1.*t1043*t1012;
  t1063 = t1034*t1023;
  t1064 = t1062 + t1063;
  t1109 = Cos(var1[10]);
  t1116 = -1.*t1109;
  t1119 = 1. + t1116;
  t1122 = Sin(var1[10]);
  t1070 = t1034*t1012;
  t1084 = t1043*t1023;
  t1089 = t1070 + t1084;
  t1120 = -0.19074*t1119;
  t1123 = 0.03315*t1122;
  t1125 = t1120 + t1123;
  t1126 = t1125*t1064;
  t1130 = -0.03315*t1119;
  t1135 = -0.19074*t1122;
  t1139 = t1130 + t1135;
  t1140 = t1139*t1089;
  t1141 = t1122*t1064;
  t1145 = t1109*t1089;
  t1147 = t1141 + t1145;
  t1170 = Cos(var1[11]);
  t1174 = -1.*t1170;
  t1179 = 1. + t1174;
  t1190 = Sin(var1[11]);
  t1151 = t1109*t1064;
  t1155 = -1.*t1122*t1089;
  t1158 = t1151 + t1155;
  t1187 = -0.01315*t1179;
  t1193 = -0.62554*t1190;
  t1197 = t1187 + t1193;
  t1199 = t1197*t1147;
  t1202 = -0.62554*t1179;
  t1203 = 0.01315*t1190;
  t1209 = t1202 + t1203;
  t1210 = t1209*t1158;
  t1212 = -1.*t1190*t1147;
  t1213 = t1170*t1158;
  t1214 = t1212 + t1213;
  t1246 = Cos(var1[12]);
  t1253 = -1.*t1246;
  t1256 = 1. + t1253;
  t1272 = Sin(var1[12]);
  t1224 = t1170*t1147;
  t1226 = t1190*t1158;
  t1227 = t1224 + t1226;
  t1322 = t3*t96*t225;
  t1325 = -1.*t3*t201*t235;
  t1327 = -1.*t3*t96*t196;
  t1328 = -1.*t177*t3*t235;
  t1331 = t1327 + t1328;
  t1339 = t177*t3*t96;
  t1341 = -1.*t3*t196*t235;
  t1342 = t1339 + t1341;
  t1355 = t401*t1342;
  t1368 = t483*t1331;
  t1385 = t372*t506*t1342;
  t1404 = t474*t1331;
  t1405 = t450*t372*t1342;
  t1406 = t1404 + t1405;
  t1424 = t450*t1331;
  t1428 = -1.*t372*t474*t1342;
  t1430 = t1424 + t1428;
  t1440 = t572*t1406;
  t1446 = t588*t1430;
  t1447 = -1.*t570*t1406;
  t1456 = t556*t1430;
  t1458 = t1447 + t1456;
  t1461 = t556*t1406;
  t1468 = t570*t1430;
  t1473 = t1461 + t1468;
  t1480 = t645*t1458;
  t1487 = t653*t1473;
  t1492 = t630*t1458;
  t1494 = t616*t1473;
  t1495 = t1492 + t1494;
  t1505 = t616*t1458;
  t1508 = -1.*t630*t1473;
  t1512 = t1505 + t1508;
  t1521 = t725*t1495;
  t1523 = t745*t1512;
  t792 = -1.03354*t791;
  t795 = 0.05315*t793;
  t796 = t792 + t795;
  t1526 = -1.*t717*t1495;
  t1527 = t703*t1512;
  t1530 = t1526 + t1527;
  t801 = -0.05315*t791;
  t803 = -1.03354*t793;
  t805 = t801 + t803;
  t1541 = t703*t1495;
  t1545 = t717*t1512;
  t1546 = t1541 + t1545;
  t1595 = -1.*t3*t235*t862;
  t1597 = t3*t96*t877;
  t1598 = -1.*t3*t837*t235;
  t1599 = -1.*t3*t96*t858;
  t1601 = t1598 + t1599;
  t1621 = t3*t96*t837;
  t1622 = -1.*t3*t235*t858;
  t1624 = t1621 + t1622;
  t1643 = t1624*t931;
  t1664 = t1624*t923*t987;
  t1667 = t1601*t998;
  t1671 = t969*t1624*t923;
  t1673 = t1601*t983;
  t1676 = t1671 + t1673;
  t1683 = t969*t1601;
  t1689 = -1.*t1624*t923*t983;
  t1690 = t1683 + t1689;
  t1702 = t1053*t1676;
  t1704 = t1060*t1690;
  t1705 = -1.*t1043*t1676;
  t1706 = t1034*t1690;
  t1708 = t1705 + t1706;
  t1712 = t1034*t1676;
  t1715 = t1043*t1690;
  t1717 = t1712 + t1715;
  t1743 = t1125*t1708;
  t1753 = t1139*t1717;
  t1760 = t1122*t1708;
  t1762 = t1109*t1717;
  t1763 = t1760 + t1762;
  t1772 = t1109*t1708;
  t1776 = -1.*t1122*t1717;
  t1782 = t1772 + t1776;
  t1792 = t1197*t1763;
  t1793 = t1209*t1782;
  t1271 = -1.03354*t1256;
  t1277 = 0.05315*t1272;
  t1281 = t1271 + t1277;
  t1795 = -1.*t1190*t1763;
  t1796 = t1170*t1782;
  t1804 = t1795 + t1796;
  t1288 = -0.05315*t1256;
  t1290 = -1.03354*t1272;
  t1293 = t1288 + t1290;
  t1810 = t1170*t1763;
  t1814 = t1190*t1782;
  t1815 = t1810 + t1814;
  t1612 = 0.09786*t1601;
  t1627 = 0.1351*t1624;
  t1898 = -0.135*t837;
  t1899 = t1898 + t871;
  t1900 = t3*t96*t1899;
  t1901 = 0.07996*t837;
  t1903 = 0.135*t858;
  t1904 = t1901 + t1903;
  t1905 = t3*t235*t1904;
  t1636 = 0.04566*t1601;
  t1641 = 0.135*t910*t1624;
  t1650 = -0.08055*t1624*t923;
  t1661 = 0.1708*t910*t1624;
  t1681 = -0.08045*t1676;
  t1693 = -0.06984*t1690;
  t1701 = 0.1327*t910*t1624;
  t1709 = -0.15304*t1708;
  t1720 = -0.04845*t1717;
  t1735 = 0.1303*t910*t1624;
  t1766 = -0.03195*t1763;
  t1783 = -0.37414*t1782;
  t1791 = 0.1318*t910*t1624;
  t1809 = -0.73604*t1804;
  t1817 = -0.04375*t1815;
  t1832 = 0.1306*t910*t1624;
  t1839 = t1281*t1804;
  t1841 = t1293*t1815;
  t1846 = t1272*t1804;
  t1855 = t1246*t1815;
  t1862 = t1846 + t1855;
  t1864 = -0.02565*t1862;
  t1868 = t1246*t1804;
  t1870 = -1.*t1272*t1815;
  t1875 = t1868 + t1870;
  t1878 = -1.03824*t1875;
  t1963 = t3*t837*t235;
  t1967 = t3*t96*t858;
  t1970 = t1963 + t1967;
  t1956 = 0.135*t910;
  t1957 = -0.08055*t923;
  t1961 = t1956 + t1957;
  t1962 = -1.*t130*t1961;
  t1975 = 0.08055*t910;
  t1980 = t1975 + t939;
  t1984 = t1970*t1980;
  t1987 = t910*t1970;
  t1992 = t130*t923;
  t1997 = t1987 + t1992;
  t2007 = t910*t130;
  t2008 = -1.*t1970*t923;
  t2015 = t2007 + t2008;
  t2038 = t1997*t987;
  t2050 = t969*t1053*t1997;
  t2054 = -1.*t1060*t1997*t983;
  t2060 = -1.*t969*t1043*t1997;
  t2061 = -1.*t1034*t1997*t983;
  t2064 = t2060 + t2061;
  t2068 = t1034*t969*t1997;
  t2069 = -1.*t1043*t1997*t983;
  t2070 = t2068 + t2069;
  t2084 = t1125*t2064;
  t2087 = t1139*t2070;
  t2088 = t1122*t2064;
  t2091 = t1109*t2070;
  t2094 = t2088 + t2091;
  t2097 = t1109*t2064;
  t2099 = -1.*t1122*t2070;
  t2107 = t2097 + t2099;
  t2133 = t1197*t2094;
  t2134 = t1209*t2107;
  t2135 = -1.*t1190*t2094;
  t2138 = t1170*t2107;
  t2140 = t2135 + t2138;
  t2146 = t1170*t2094;
  t2147 = t1190*t2107;
  t2148 = t2146 + t2147;
  t2208 = -1.*t910*t130;
  t2215 = t1970*t923;
  t2217 = t2208 + t2215;
  t2219 = -0.01004*t969;
  t2223 = -0.08055*t983;
  t2224 = t2219 + t2223;
  t2225 = t2217*t2224;
  t2232 = 0.08055*t969;
  t2240 = t2232 + t986;
  t2244 = t1624*t2240;
  t2250 = -1.*t969*t2217;
  t2252 = -1.*t1624*t983;
  t2254 = t2250 + t2252;
  t2262 = t969*t1624;
  t2263 = -1.*t2217*t983;
  t2268 = t2262 + t2263;
  t2281 = t1060*t2254;
  t2283 = t1053*t2268;
  t2284 = t1043*t2254;
  t2291 = t1034*t2268;
  t2292 = t2284 + t2291;
  t2300 = t1034*t2254;
  t2301 = -1.*t1043*t2268;
  t2303 = t2300 + t2301;
  t2320 = t1139*t2292;
  t2321 = t1125*t2303;
  t2323 = -1.*t1122*t2292;
  t2324 = t1109*t2303;
  t2332 = t2323 + t2324;
  t2337 = t1109*t2292;
  t2341 = t1122*t2303;
  t2354 = t2337 + t2341;
  t2360 = t1209*t2332;
  t2361 = t1197*t2354;
  t2362 = t1190*t2332;
  t2365 = t1170*t2354;
  t2371 = t2362 + t2365;
  t2380 = t1170*t2332;
  t2383 = -1.*t1190*t2354;
  t2386 = t2380 + t2383;
  t2424 = t969*t2217;
  t2427 = t1624*t983;
  t2428 = t2424 + t2427;
  t2415 = -0.13004*t1034;
  t2417 = -0.08055*t1043;
  t2421 = t2415 + t2417;
  t2433 = t2421*t2428;
  t2434 = 0.08055*t1034;
  t2437 = t2434 + t1046;
  t2438 = t2437*t2268;
  t2446 = -1.*t1043*t2428;
  t2454 = t2446 + t2291;
  t2461 = -1.*t1034*t2428;
  t2466 = t2461 + t2301;
  t2482 = t1139*t2454;
  t2483 = t1125*t2466;
  t2484 = -1.*t1122*t2454;
  t2487 = t1109*t2466;
  t2495 = t2484 + t2487;
  t2499 = t1109*t2454;
  t2502 = t1122*t2466;
  t2503 = t2499 + t2502;
  t2521 = t1209*t2495;
  t2522 = t1197*t2503;
  t2525 = t1190*t2495;
  t2528 = t1170*t2503;
  t2531 = t2525 + t2528;
  t2535 = t1170*t2495;
  t2537 = -1.*t1190*t2503;
  t2544 = t2535 + t2537;
  t2608 = t1034*t2428;
  t2610 = t1043*t2268;
  t2612 = t2608 + t2610;
  t2589 = 0.03315*t1109;
  t2593 = t2589 + t1135;
  t2595 = t2593*t2454;
  t2604 = -0.19074*t1109;
  t2605 = -0.03315*t1122;
  t2607 = t2604 + t2605;
  t2613 = t2607*t2612;
  t2616 = -1.*t1109*t2612;
  t2620 = t2484 + t2616;
  t2624 = -1.*t1122*t2612;
  t2625 = t2499 + t2624;
  t2637 = t1209*t2620;
  t2643 = t1197*t2625;
  t2645 = t1190*t2620;
  t2649 = t1170*t2625;
  t2652 = t2645 + t2649;
  t2659 = t1170*t2620;
  t2660 = -1.*t1190*t2625;
  t2662 = t2659 + t2660;
  t2704 = t1122*t2454;
  t2718 = t1109*t2612;
  t2720 = t2704 + t2718;
  t2698 = -0.62554*t1170;
  t2699 = -0.01315*t1190;
  t2703 = t2698 + t2699;
  t2722 = t2703*t2720;
  t2726 = 0.01315*t1170;
  t2729 = t2726 + t1193;
  t2733 = t2729*t2625;
  t2734 = -1.*t1190*t2720;
  t2736 = t2734 + t2649;
  t2740 = -1.*t1170*t2720;
  t2741 = t2740 + t2660;
  t2758 = -1.*t1272*t2736;
  t2798 = t1170*t2720;
  t2800 = t1190*t2625;
  t2802 = t2798 + t2800;
  t2768 = t1246*t2736;
  t1335 = 0.09786*t1331;
  t1343 = -0.1351*t1342;
  t2839 = 0.135*t177;
  t2849 = t2839 + t224;
  t2852 = t3*t96*t2849;
  t2855 = 0.07996*t177;
  t2856 = -0.135*t196;
  t2858 = t2855 + t2856;
  t2859 = t3*t2858*t235;
  t1352 = 0.04566*t1331;
  t1353 = -0.135*t349*t1342;
  t1358 = -0.08055*t372*t1342;
  t1380 = -0.1708*t349*t1342;
  t1420 = -0.08045*t1406;
  t1435 = -0.06984*t1430;
  t1439 = -0.1327*t349*t1342;
  t1459 = -0.15304*t1458;
  t1474 = -0.04845*t1473;
  t1479 = -0.1303*t349*t1342;
  t1504 = -0.03195*t1495;
  t1513 = -0.37414*t1512;
  t1517 = -0.1318*t349*t1342;
  t1539 = -0.73604*t1530;
  t1549 = -0.04375*t1546;
  t1557 = -0.1306*t349*t1342;
  t1559 = t796*t1530;
  t1564 = t805*t1546;
  t1567 = t793*t1530;
  t1568 = t783*t1546;
  t1569 = t1567 + t1568;
  t1572 = -0.02565*t1569;
  t1573 = t783*t1530;
  t1580 = -1.*t793*t1546;
  t1585 = t1573 + t1580;
  t1586 = -1.03824*t1585;
  t2910 = t3*t96*t196;
  t2914 = t177*t3*t235;
  t2915 = t2910 + t2914;
  t2920 = t372*t130;
  t2922 = t349*t2915;
  t2923 = t2920 + t2922;
  t2888 = -0.135*t349;
  t2889 = -0.08055*t372;
  t2892 = t2888 + t2889;
  t2895 = -1.*t2892*t130;
  t2905 = 0.08055*t349;
  t2909 = t2905 + t384;
  t2919 = t2909*t2915;
  t2939 = t349*t130;
  t2943 = -1.*t372*t2915;
  t2944 = t2939 + t2943;
  t2934 = t506*t2923;
  t2962 = t450*t572*t2923;
  t2964 = -1.*t474*t588*t2923;
  t2969 = -1.*t556*t474*t2923;
  t2972 = -1.*t450*t570*t2923;
  t2978 = t2969 + t2972;
  t2984 = t450*t556*t2923;
  t2987 = -1.*t474*t570*t2923;
  t2988 = t2984 + t2987;
  t3001 = t645*t2978;
  t3008 = t653*t2988;
  t3009 = t630*t2978;
  t3015 = t616*t2988;
  t3016 = t3009 + t3015;
  t3023 = t616*t2978;
  t3025 = -1.*t630*t2988;
  t3026 = t3023 + t3025;
  t3033 = t725*t3016;
  t3036 = t745*t3026;
  t3037 = -1.*t717*t3016;
  t3038 = t703*t3026;
  t3041 = t3037 + t3038;
  t3047 = t703*t3016;
  t3049 = t717*t3026;
  t3051 = t3047 + t3049;
  t3121 = -1.*t349*t130;
  t3122 = t372*t2915;
  t3124 = t3121 + t3122;
  t3102 = 0.08055*t450;
  t3103 = t3102 + t498;
  t3110 = t3103*t1342;
  t3112 = -0.01004*t450;
  t3118 = -0.08055*t474;
  t3119 = t3112 + t3118;
  t3129 = t3119*t3124;
  t3130 = -1.*t474*t1342;
  t3132 = -1.*t450*t3124;
  t3141 = t3130 + t3132;
  t3149 = t450*t1342;
  t3150 = -1.*t474*t3124;
  t3153 = t3149 + t3150;
  t3160 = t588*t3141;
  t3167 = t572*t3153;
  t3172 = t570*t3141;
  t3174 = t556*t3153;
  t3180 = t3172 + t3174;
  t3186 = t556*t3141;
  t3189 = -1.*t570*t3153;
  t3190 = t3186 + t3189;
  t3210 = t653*t3180;
  t3212 = t645*t3190;
  t3216 = -1.*t630*t3180;
  t3220 = t616*t3190;
  t3226 = t3216 + t3220;
  t3232 = t616*t3180;
  t3234 = t630*t3190;
  t3235 = t3232 + t3234;
  t3242 = t745*t3226;
  t3243 = t725*t3235;
  t3246 = t717*t3226;
  t3253 = t703*t3235;
  t3255 = t3246 + t3253;
  t3261 = t703*t3226;
  t3269 = -1.*t717*t3235;
  t3270 = t3261 + t3269;
  t3342 = t474*t1342;
  t3343 = t450*t3124;
  t3344 = t3342 + t3343;
  t3329 = -0.13004*t556;
  t3332 = -0.08055*t570;
  t3337 = t3329 + t3332;
  t3351 = t3337*t3344;
  t3352 = 0.08055*t556;
  t3354 = t3352 + t571;
  t3361 = t3354*t3153;
  t3366 = -1.*t570*t3344;
  t3370 = t3366 + t3174;
  t3376 = -1.*t556*t3344;
  t3377 = t3376 + t3189;
  t3389 = t653*t3370;
  t3391 = t645*t3377;
  t3396 = -1.*t630*t3370;
  t3399 = t616*t3377;
  t3401 = t3396 + t3399;
  t3407 = t616*t3370;
  t3408 = t630*t3377;
  t3409 = t3407 + t3408;
  t3416 = t745*t3401;
  t3421 = t725*t3409;
  t3427 = t717*t3401;
  t3428 = t703*t3409;
  t3433 = t3427 + t3428;
  t3439 = t703*t3401;
  t3446 = -1.*t717*t3409;
  t3448 = t3439 + t3446;
  t3505 = t556*t3344;
  t3507 = t570*t3153;
  t3510 = t3505 + t3507;
  t3490 = 0.03315*t616;
  t3492 = t3490 + t652;
  t3495 = t3492*t3370;
  t3498 = -0.19074*t616;
  t3500 = -0.03315*t630;
  t3501 = t3498 + t3500;
  t3511 = t3501*t3510;
  t3516 = -1.*t616*t3510;
  t3517 = t3396 + t3516;
  t3522 = -1.*t630*t3510;
  t3525 = t3407 + t3522;
  t3535 = t745*t3517;
  t3537 = t725*t3525;
  t3538 = t717*t3517;
  t3539 = t703*t3525;
  t3540 = t3538 + t3539;
  t3544 = t703*t3517;
  t3547 = -1.*t717*t3525;
  t3548 = t3544 + t3547;
  t3597 = t630*t3370;
  t3602 = t616*t3510;
  t3605 = t3597 + t3602;
  t3584 = -0.62554*t703;
  t3593 = -0.01315*t717;
  t3595 = t3584 + t3593;
  t3606 = t3595*t3605;
  t3610 = 0.01315*t703;
  t3613 = t3610 + t720;
  t3614 = t3613*t3525;
  t3615 = -1.*t717*t3605;
  t3619 = t3615 + t3539;
  t3627 = -1.*t703*t3605;
  t3629 = t3627 + t3547;
  t3639 = -1.*t793*t3619;
  t3672 = t703*t3605;
  t3674 = t717*t3525;
  t3676 = t3672 + t3674;
  t3645 = t783*t3619;
  p_output1[0]=0;
  p_output1[1]=0;
  p_output1[2]=312.78204;
  p_output1[3]=0;
  p_output1[4]=17.854200000000002*(t175 + t214 + t253 - 0.1351*t273 + 0.09786*t290) + 11.4777*(t214 + t253 + 0.04566*t290 + t389 + t402 - 0.135*t424 - 0.08055*t436) + 54.151199999999996*(t214 + t253 + t389 + t402 - 0.1708*t424 + t484 + t510 - 0.08045*t524 - 0.06984*t547) + 7.435980000000001*(t214 + t253 + t389 + t402 - 0.1327*t424 + t484 + t510 + t573 + t591 - 0.15304*t599 - 0.04845*t607) + 5.6603699999999995*(t214 + t253 + t389 + t402 - 0.1303*t424 + t484 + t510 + t573 + t591 + t647 + t659 - 0.03195*t664 - 0.37414*t684) + 7.67142*(t214 + t253 + t389 + t402 - 0.1318*t424 + t484 + t510 + t573 + t591 + t647 + t659 + t726 + t747 - 0.73604*t754 - 0.04375*t766) + 1.4715*(t214 + t253 + t389 + t402 - 0.1306*t424 + t484 + t510 + t573 + t591 + t647 + t659 + t726 + t747 - 0.02565*(t766*t783 + t754*t793) - 1.03824*(t754*t783 - 1.*t766*t793) + t754*t796 + t766*t805) + 17.854200000000002*(t175 + t866 + t885 + 0.1351*t894 + 0.09786*t900) + 11.4777*(t866 + t885 + 0.04566*t900 + t934 + t941 + 0.135*t952 - 0.08055*t957) + 101.3373*(-0.01915*t3 - 0.10836*t130*t96) + 5.6603699999999995*(t1056 + t1061 + t1126 + t1140 - 0.03195*t1147 - 0.37414*t1158 + t866 + t885 + t934 + t941 + 0.1303*t952 + t990 + t999) + 1.4715*(t1056 + t1061 + t1126 + t1140 + t1199 + t1210 - 0.02565*(t1227*t1246 + t1214*t1272) - 1.03824*(t1214*t1246 - 1.*t1227*t1272) + t1214*t1281 + t1227*t1293 + t866 + t885 + t934 + t941 + 0.1306*t952 + t990 + t999) + 7.67142*(t1056 + t1061 + t1126 + t1140 + t1199 + t1210 - 0.73604*t1214 - 0.04375*t1227 + t866 + t885 + t934 + t941 + 0.1318*t952 + t990 + t999) + 7.435980000000001*(t1056 + t1061 - 0.15304*t1064 - 0.04845*t1089 + t866 + t885 + t934 + t941 + 0.1327*t952 + t990 + t999) + 54.151199999999996*(-0.08045*t1012 - 0.06984*t1023 + t866 + t885 + t934 + t941 + 0.1708*t952 + t990 + t999);
  p_output1[5]=17.854200000000002*(t1322 + t1325 + t1335 + t1343) + 11.4777*(t1322 + t1325 + t1352 + t1353 + t1355 + t1358) + 54.151199999999996*(t1322 + t1325 + t1355 + t1368 + t1380 + t1385 + t1420 + t1435) + 7.435980000000001*(t1322 + t1325 + t1355 + t1368 + t1385 + t1439 + t1440 + t1446 + t1459 + t1474) + 5.6603699999999995*(t1322 + t1325 + t1355 + t1368 + t1385 + t1440 + t1446 + t1479 + t1480 + t1487 + t1504 + t1513) + 7.67142*(t1322 + t1325 + t1355 + t1368 + t1385 + t1440 + t1446 + t1480 + t1487 + t1517 + t1521 + t1523 + t1539 + t1549) + 1.4715*(t1322 + t1325 + t1355 + t1368 + t1385 + t1440 + t1446 + t1480 + t1487 + t1521 + t1523 + t1557 + t1559 + t1564 + t1572 + t1586) + 17.854200000000002*(t1595 + t1597 + t1612 + t1627) + 11.4777*(t1595 + t1597 + t1636 + t1641 + t1643 + t1650) + 54.151199999999996*(t1595 + t1597 + t1643 + t1661 + t1664 + t1667 + t1681 + t1693) + 7.435980000000001*(t1595 + t1597 + t1643 + t1664 + t1667 + t1701 + t1702 + t1704 + t1709 + t1720) + 5.6603699999999995*(t1595 + t1597 + t1643 + t1664 + t1667 + t1702 + t1704 + t1735 + t1743 + t1753 + t1766 + t1783) + 7.67142*(t1595 + t1597 + t1643 + t1664 + t1667 + t1702 + t1704 + t1743 + t1753 + t1791 + t1792 + t1793 + t1809 + t1817) + 1.4715*(t1595 + t1597 + t1643 + t1664 + t1667 + t1702 + t1704 + t1743 + t1753 + t1792 + t1793 + t1832 + t1839 + t1841 + t1864 + t1878) - 10.980909828*t235*t3;
  p_output1[6]=17.854200000000002*(t1612 + t1627 + t1900 + t1905) + 11.4777*(t1636 + t1641 + t1643 + t1650 + t1900 + t1905) + 54.151199999999996*(t1643 + t1661 + t1664 + t1667 + t1681 + t1693 + t1900 + t1905) + 7.435980000000001*(t1643 + t1664 + t1667 + t1701 + t1702 + t1704 + t1709 + t1720 + t1900 + t1905) + 5.6603699999999995*(t1643 + t1664 + t1667 + t1702 + t1704 + t1735 + t1743 + t1753 + t1766 + t1783 + t1900 + t1905) + 7.67142*(t1643 + t1664 + t1667 + t1702 + t1704 + t1743 + t1753 + t1791 + t1792 + t1793 + t1809 + t1817 + t1900 + t1905) + 1.4715*(t1643 + t1664 + t1667 + t1702 + t1704 + t1743 + t1753 + t1792 + t1793 + t1832 + t1839 + t1841 + t1864 + t1878 + t1900 + t1905);
  p_output1[7]=11.4777*(t1962 + t1984 - 0.08055*t1997 + 0.135*t2015) + 7.435980000000001*(t1962 + t1984 + 0.1327*t2015 + t2038 + t2050 + t2054 - 0.15304*t2064 - 0.04845*t2070) + 5.6603699999999995*(t1962 + t1984 + 0.1303*t2015 + t2038 + t2050 + t2054 + t2084 + t2087 - 0.03195*t2094 - 0.37414*t2107) + 7.67142*(t1962 + t1984 + 0.1318*t2015 + t2038 + t2050 + t2054 + t2084 + t2087 + t2133 + t2134 - 0.73604*t2140 - 0.04375*t2148) + 1.4715*(t1962 + t1984 + 0.1306*t2015 + t2038 + t2050 + t2054 + t2084 + t2087 + t2133 + t2134 + t1281*t2140 + t1293*t2148 - 0.02565*(t1272*t2140 + t1246*t2148) - 1.03824*(t1246*t2140 - 1.*t1272*t2148)) + 54.151199999999996*(t1962 + t1984 + 0.1708*t2015 + t2038 - 0.08045*t1997*t969 + 0.06984*t1997*t983);
  p_output1[8]=54.151199999999996*(t2225 + t2244 - 0.06984*t2254 - 0.08045*t2268) + 7.435980000000001*(t2225 + t2244 + t2281 + t2283 - 0.04845*t2292 - 0.15304*t2303) + 5.6603699999999995*(t2225 + t2244 + t2281 + t2283 + t2320 + t2321 - 0.37414*t2332 - 0.03195*t2354) + 7.67142*(t2225 + t2244 + t2281 + t2283 + t2320 + t2321 + t2360 + t2361 - 0.04375*t2371 - 0.73604*t2386) + 1.4715*(t2225 + t2244 + t2281 + t2283 + t2320 + t2321 + t2360 + t2361 + t1293*t2371 + t1281*t2386 - 1.03824*(-1.*t1272*t2371 + t1246*t2386) - 0.02565*(t1246*t2371 + t1272*t2386));
  p_output1[9]=7.435980000000001*(t2433 + t2438 - 0.04845*t2454 - 0.15304*t2466) + 5.6603699999999995*(t2433 + t2438 + t2482 + t2483 - 0.37414*t2495 - 0.03195*t2503) + 7.67142*(t2433 + t2438 + t2482 + t2483 + t2521 + t2522 - 0.04375*t2531 - 0.73604*t2544) + 1.4715*(t2433 + t2438 + t2482 + t2483 + t2521 + t2522 + t1293*t2531 + t1281*t2544 - 1.03824*(-1.*t1272*t2531 + t1246*t2544) - 0.02565*(t1246*t2531 + t1272*t2544));
  p_output1[10]=5.6603699999999995*(t2595 + t2613 - 0.37414*t2620 - 0.03195*t2625) + 7.67142*(t2595 + t2613 + t2637 + t2643 - 0.04375*t2652 - 0.73604*t2662) + 1.4715*(t2595 + t2613 + t2637 + t2643 + t1293*t2652 + t1281*t2662 - 1.03824*(-1.*t1272*t2652 + t1246*t2662) - 0.02565*(t1246*t2652 + t1272*t2662));
  p_output1[11]=7.67142*(t2722 + t2733 - 0.04375*t2736 - 0.73604*t2741) + 1.4715*(t2722 + t2733 + t1293*t2736 + t1281*t2741 - 1.03824*(t1246*t2741 + t2758) - 0.02565*(t1272*t2741 + t2768));
  p_output1[12]=1.4715*((0.05315*t1246 + t1290)*t2736 + (-1.03354*t1246 - 0.05315*t1272)*t2802 - 1.03824*(t2758 - 1.*t1246*t2802) - 0.02565*(t2768 - 1.*t1272*t2802));
  p_output1[13]=17.854200000000002*(t1335 + t1343 + t2852 + t2859) + 11.4777*(t1352 + t1353 + t1355 + t1358 + t2852 + t2859) + 54.151199999999996*(t1355 + t1368 + t1380 + t1385 + t1420 + t1435 + t2852 + t2859) + 7.435980000000001*(t1355 + t1368 + t1385 + t1439 + t1440 + t1446 + t1459 + t1474 + t2852 + t2859) + 5.6603699999999995*(t1355 + t1368 + t1385 + t1440 + t1446 + t1479 + t1480 + t1487 + t1504 + t1513 + t2852 + t2859) + 7.67142*(t1355 + t1368 + t1385 + t1440 + t1446 + t1480 + t1487 + t1517 + t1521 + t1523 + t1539 + t1549 + t2852 + t2859) + 1.4715*(t1355 + t1368 + t1385 + t1440 + t1446 + t1480 + t1487 + t1521 + t1523 + t1557 + t1559 + t1564 + t1572 + t1586 + t2852 + t2859);
  p_output1[14]=11.4777*(t2895 + t2919 - 0.08055*t2923 - 0.135*t2944) + 7.435980000000001*(t2895 + t2919 + t2934 - 0.1327*t2944 + t2962 + t2964 - 0.15304*t2978 - 0.04845*t2988) + 5.6603699999999995*(t2895 + t2919 + t2934 - 0.1303*t2944 + t2962 + t2964 + t3001 + t3008 - 0.03195*t3016 - 0.37414*t3026) + 7.67142*(t2895 + t2919 + t2934 - 0.1318*t2944 + t2962 + t2964 + t3001 + t3008 + t3033 + t3036 - 0.73604*t3041 - 0.04375*t3051) + 54.151199999999996*(t2895 + t2919 + t2934 - 0.1708*t2944 - 0.08045*t2923*t450 + 0.06984*t2923*t474) + 1.4715*(t2895 + t2919 + t2934 - 0.1306*t2944 + t2962 + t2964 + t3001 + t3008 + t3033 + t3036 - 0.02565*(t3051*t783 + t3041*t793) - 1.03824*(t3041*t783 - 1.*t3051*t793) + t3041*t796 + t3051*t805);
  p_output1[15]=54.151199999999996*(t3110 + t3129 - 0.06984*t3141 - 0.08045*t3153) + 7.435980000000001*(t3110 + t3129 + t3160 + t3167 - 0.04845*t3180 - 0.15304*t3190) + 5.6603699999999995*(t3110 + t3129 + t3160 + t3167 + t3210 + t3212 - 0.37414*t3226 - 0.03195*t3235) + 7.67142*(t3110 + t3129 + t3160 + t3167 + t3210 + t3212 + t3242 + t3243 - 0.04375*t3255 - 0.73604*t3270) + 1.4715*(t3110 + t3129 + t3160 + t3167 + t3210 + t3212 + t3242 + t3243 - 1.03824*(t3270*t783 - 1.*t3255*t793) - 0.02565*(t3255*t783 + t3270*t793) + t3270*t796 + t3255*t805);
  p_output1[16]=7.435980000000001*(t3351 + t3361 - 0.04845*t3370 - 0.15304*t3377) + 5.6603699999999995*(t3351 + t3361 + t3389 + t3391 - 0.37414*t3401 - 0.03195*t3409) + 7.67142*(t3351 + t3361 + t3389 + t3391 + t3416 + t3421 - 0.04375*t3433 - 0.73604*t3448) + 1.4715*(t3351 + t3361 + t3389 + t3391 + t3416 + t3421 - 1.03824*(t3448*t783 - 1.*t3433*t793) - 0.02565*(t3433*t783 + t3448*t793) + t3448*t796 + t3433*t805);
  p_output1[17]=5.6603699999999995*(t3495 + t3511 - 0.37414*t3517 - 0.03195*t3525) + 7.67142*(t3495 + t3511 + t3535 + t3537 - 0.04375*t3540 - 0.73604*t3548) + 1.4715*(t3495 + t3511 + t3535 + t3537 - 1.03824*(t3548*t783 - 1.*t3540*t793) - 0.02565*(t3540*t783 + t3548*t793) + t3548*t796 + t3540*t805);
  p_output1[18]=7.67142*(t3606 + t3614 - 0.04375*t3619 - 0.73604*t3629) + 1.4715*(t3606 + t3614 - 1.03824*(t3639 + t3629*t783) - 0.02565*(t3645 + t3629*t793) + t3629*t796 + t3619*t805);
  p_output1[19]=1.4715*(-1.03824*(t3639 - 1.*t3676*t783) + t3676*(-1.03354*t783 - 0.05315*t793) - 0.02565*(t3645 - 1.*t3676*t793) + t3619*(0.05315*t783 + t803));
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

  double *var1;
  double *p_output1;

  /*  Check for proper number of arguments.  */ 
  if( nrhs != 1)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "One input(s) required (var1).");
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

  /*  Assign pointers to each input.  */
  var1 = mxGetPr(prhs[0]);
   


   
  /*  Create matrices for return arguments.  */
  plhs[0] = mxCreateDoubleMatrix((mwSize) 20, (mwSize) 1, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,var1);


}

#else // MATLAB_MEX_FILE

#include "GravityVector_mex.hh"

namespace SymExpression
{

void GravityVector_mex_raw(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

}

#endif // MATLAB_MEX_FILE
