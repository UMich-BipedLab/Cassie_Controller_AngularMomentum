/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:25:28 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jq_AMWorld_LeftKnee_src.h"

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
  double t124;
  double t171;
  double t193;
  double t89;
  double t197;
  double t65;
  double t29;
  double t646;
  double t695;
  double t474;
  double t571;
  double t888;
  double t568;
  double t746;
  double t767;
  double t790;
  double t796;
  double t1292;
  double t1346;
  double t1138;
  double t1354;
  double t1503;
  double t1512;
  double t1518;
  double t1577;
  double t1626;
  double t1307;
  double t1370;
  double t1385;
  double t1757;
  double t1786;
  double t1661;
  double t1789;
  double t1814;
  double t1927;
  double t2002;
  double t2007;
  double t2457;
  double t2483;
  double t2484;
  double t2502;
  double t2508;
  double t2678;
  double t2696;
  double t2617;
  double t2708;
  double t2709;
  double t2754;
  double t2791;
  double t2824;
  double t167;
  double t226;
  double t232;
  double t265;
  double t361;
  double t400;
  double t425;
  double t528;
  double t550;
  double t3254;
  double t3255;
  double t3266;
  double t3183;
  double t3210;
  double t3212;
  double t777;
  double t824;
  double t851;
  double t858;
  double t943;
  double t1034;
  double t1039;
  double t1042;
  double t1045;
  double t1048;
  double t1066;
  double t1086;
  double t1092;
  double t1094;
  double t1461;
  double t1655;
  double t1656;
  double t1918;
  double t2028;
  double t2029;
  double t2074;
  double t2100;
  double t2205;
  double t2245;
  double t2281;
  double t2302;
  double t2333;
  double t2334;
  double t2359;
  double t2449;
  double t2510;
  double t2605;
  double t2722;
  double t2858;
  double t2863;
  double t2959;
  double t2973;
  double t3020;
  double t3033;
  double t3101;
  double t3103;
  double t3129;
  double t3131;
  double t3136;
  double t3243;
  double t3286;
  double t3287;
  double t3295;
  double t3309;
  double t3331;
  double t1099;
  double t2394;
  double t3148;
  double t3169;
  double t3622;
  double t3635;
  double t3640;
  double t3654;
  double t3667;
  double t3681;
  double t3685;
  double t3753;
  double t3769;
  double t3883;
  double t3890;
  double t3893;
  double t3828;
  double t3836;
  double t3858;
  double t3389;
  double t3403;
  double t3417;
  double t3426;
  double t3861;
  double t3910;
  double t3911;
  double t3971;
  double t3993;
  double t4009;
  double t3513;
  double t3567;
  double t3592;
  double t3595;
  double t3773;
  double t3788;
  double t3804;
  double t3959;
  double t4013;
  double t4015;
  double t4083;
  double t4087;
  double t4136;
  double t4195;
  double t4209;
  double t4233;
  double t4247;
  double t4273;
  double t4287;
  double t4023;
  double t4059;
  double t4069;
  double t4489;
  double t4492;
  double t4494;
  double t4554;
  double t4574;
  double t4581;
  double t4529;
  double t4539;
  double t4542;
  double t4543;
  double t4590;
  double t4612;
  double t4618;
  double t4643;
  double t4652;
  double t4142;
  double t4234;
  double t4311;
  double t4314;
  double t4765;
  double t4769;
  double t4770;
  double t4772;
  double t4777;
  double t4834;
  double t4866;
  double t4881;
  double t4885;
  double t5040;
  double t5071;
  double t5111;
  double t4945;
  double t4980;
  double t5036;
  double t4318;
  double t4329;
  double t4335;
  double t4339;
  double t5039;
  double t5115;
  double t5117;
  double t5131;
  double t5154;
  double t5167;
  double t4425;
  double t4427;
  double t4441;
  double t4474;
  double t5329;
  double t5339;
  double t5357;
  double t5731;
  double t5762;
  double t5765;
  double t5604;
  double t5610;
  double t5692;
  double t5703;
  double t5769;
  double t5797;
  double t5804;
  double t5817;
  double t5818;
  double t5952;
  double t5958;
  double t5962;
  double t6058;
  double t6069;
  double t6074;
  double t6012;
  double t6013;
  double t6014;
  double t6037;
  double t6093;
  double t6145;
  double t6169;
  double t6170;
  double t6229;
  double t6339;
  double t6345;
  double t6356;
  double t6482;
  double t6492;
  double t6504;
  double t6429;
  double t6455;
  double t6460;
  double t6461;
  double t6515;
  double t6534;
  double t6543;
  double t6545;
  double t6554;
  double t6771;
  double t6778;
  double t6785;
  double t6791;
  double t6795;
  double t6801;
  double t6832;
  double t6842;
  double t6846;
  double t6861;
  double t6884;
  double t6902;
  double t6909;
  double t6920;
  double t6930;
  double t6934;
  double t6938;
  double t6940;
  double t6942;
  double t6953;
  double t6954;
  double t7098;
  double t7160;
  double t7180;
  double t7181;
  double t7182;
  double t7226;
  double t7228;
  double t7229;
  double t7217;
  double t7235;
  double t7238;
  double t7255;
  double t7261;
  double t7265;
  double t4890;
  double t4926;
  double t4927;
  double t6841;
  double t6922;
  double t6968;
  double t6977;
  double t5118;
  double t5181;
  double t5182;
  double t7057;
  double t7059;
  double t7061;
  double t7064;
  double t5300;
  double t5307;
  double t5314;
  double t7073;
  double t7075;
  double t7082;
  double t7088;
  double t7365;
  double t7370;
  double t7378;
  double t7400;
  double t7373;
  double t7406;
  double t7423;
  double t7464;
  double t7471;
  double t7476;
  double t5976;
  double t5982;
  double t5993;
  double t6158;
  double t6237;
  double t6254;
  double t6300;
  double t6320;
  double t6321;
  double t7639;
  double t7655;
  double t7677;
  double t7638;
  double t7693;
  double t7699;
  double t7727;
  double t7730;
  double t7737;
  double t7839;
  double t7841;
  double t7843;
  double t7844;
  double t7864;
  double t7868;
  double t7869;
  double t7870;
  double t7809;
  double t7812;
  double t7818;
  double t7819;
  double t7820;
  double t7821;
  double t7824;
  double t7833;
  double t7835;
  double t7840;
  double t7847;
  double t7848;
  double t7857;
  double t7858;
  double t7859;
  double t7860;
  double t7861;
  double t7862;
  double t7865;
  double t7875;
  double t7878;
  double t7880;
  double t7881;
  double t7882;
  double t7883;
  double t7886;
  double t7888;
  double t7929;
  double t7933;
  double t7926;
  double t7934;
  double t7935;
  double t7938;
  double t7939;
  double t7942;
  double t7838;
  double t7863;
  double t7889;
  double t7891;
  double t7893;
  double t7897;
  double t7902;
  double t7907;
  double t7911;
  double t7915;
  double t7916;
  double t7919;
  double t7990;
  double t7993;
  double t7988;
  double t8004;
  double t8006;
  double t8008;
  double t8009;
  double t8013;
  double t7637;
  double t7705;
  double t7740;
  double t7741;
  double t7745;
  double t7759;
  double t7771;
  double t7793;
  double t7797;
  double t8033;
  double t8034;
  double t8036;
  double t8038;
  double t8039;
  double t8040;
  double t8041;
  double t8043;
  double t8045;
  double t8046;
  double t8048;
  double t8049;
  double t8051;
  double t8053;
  double t8054;
  double t8057;
  double t8058;
  double t8060;
  double t8061;
  double t8062;
  double t8064;
  double t8065;
  double t8066;
  double t8070;
  double t8071;
  double t8073;
  double t8074;
  double t8076;
  double t8077;
  double t8078;
  double t8079;
  double t8080;
  double t8081;
  double t8082;
  double t8084;
  double t8085;
  double t8087;
  double t8088;
  double t8089;
  double t8090;
  double t8092;
  double t8116;
  double t8117;
  double t8118;
  double t8052;
  double t8075;
  double t8093;
  double t8094;
  double t8096;
  double t8099;
  double t8100;
  double t8101;
  double t8108;
  double t8109;
  double t8110;
  double t8113;
  double t606;
  double t8133;
  double t8134;
  double t8161;
  double t8162;
  double t8163;
  double t8174;
  double t8175;
  double t8185;
  double t8201;
  double t8205;
  double t8206;
  double t8207;
  double t8215;
  double t8217;
  double t8218;
  double t8239;
  double t8249;
  double t8250;
  double t8255;
  double t8257;
  double t8258;
  double t8259;
  double t8195;
  double t8198;
  double t8203;
  double t8214;
  double t8220;
  double t8221;
  double t8222;
  double t8227;
  double t8228;
  double t8229;
  double t8230;
  double t8233;
  double t8235;
  double t8236;
  double t8237;
  double t8241;
  double t8245;
  double t8247;
  double t8256;
  double t8262;
  double t8263;
  double t8264;
  double t8266;
  double t8267;
  double t8269;
  double t8270;
  double t8271;
  double t8275;
  double t8276;
  double t8278;
  double t8296;
  double t8301;
  double t8302;
  double t8238;
  double t8280;
  double t8283;
  double t8285;
  double t8286;
  double t8287;
  double t8292;
  double t8293;
  double t8294;
  double t8322;
  double t8324;
  double t8325;
  double t8342;
  double t8343;
  double t8344;
  double t8284;
  double t8288;
  double t8295;
  double t8308;
  double t8317;
  double t8318;
  double t8321;
  double t8332;
  double t8337;
  double t8339;
  double t8340;
  double t8350;
  t124 = Cos(var1[5]);
  t171 = Sin(var1[3]);
  t193 = Sin(var1[4]);
  t89 = Cos(var1[3]);
  t197 = Sin(var1[5]);
  t65 = Cos(var1[6]);
  t29 = Cos(var1[7]);
  t646 = -1.*t29;
  t695 = 0. + t646;
  t474 = Sin(var1[6]);
  t571 = Sin(var1[7]);
  t888 = 0. + t571;
  t568 = Cos(var1[4]);
  t746 = t65*t695;
  t767 = 0. + t746;
  t790 = t695*t474;
  t796 = 0. + t790;
  t1292 = Cos(var1[8]);
  t1346 = Sin(var1[9]);
  t1138 = Cos(var1[9]);
  t1354 = Sin(var1[8]);
  t1503 = t1292*t1346;
  t1512 = t1138*t1354;
  t1518 = 0. + t1503 + t1512;
  t1577 = t29*t1518;
  t1626 = 0. + t1577;
  t1307 = -1.*t1138*t1292;
  t1370 = t1346*t1354;
  t1385 = 0. + t1307 + t1370;
  t1757 = t571*t1518;
  t1786 = 0. + t1757;
  t1661 = -1.*t474*t1385;
  t1789 = t65*t1786;
  t1814 = 0. + t1661 + t1789;
  t1927 = t65*t1385;
  t2002 = t474*t1786;
  t2007 = 0. + t1927 + t2002;
  t2457 = t1138*t1292;
  t2483 = -1.*t1346*t1354;
  t2484 = 0. + t2457 + t2483;
  t2502 = t29*t2484;
  t2508 = 0. + t2502;
  t2678 = t571*t2484;
  t2696 = 0. + t2678;
  t2617 = -1.*t474*t1518;
  t2708 = t65*t2696;
  t2709 = 0. + t2617 + t2708;
  t2754 = t65*t1518;
  t2791 = t474*t2696;
  t2824 = 0. + t2754 + t2791;
  t167 = -1.*t89*t124;
  t226 = -1.*t171*t193*t197;
  t232 = t167 + t226;
  t265 = t65*t232;
  t361 = -1.*t124*t171*t193;
  t400 = t89*t197;
  t425 = t361 + t400;
  t528 = t425*t474;
  t550 = t265 + t528;
  t3254 = t65*t425;
  t3255 = -1.*t232*t474;
  t3266 = t3254 + t3255;
  t3183 = -1.*t568*t29*t171;
  t3210 = t550*t571;
  t3212 = t3183 + t3210;
  t777 = t124*t767;
  t824 = -1.*t197*t796;
  t851 = 0. + t777 + t824;
  t858 = var2[4]*t851;
  t943 = var2[5]*t888;
  t1034 = var2[6]*t888;
  t1039 = t767*t197;
  t1042 = t124*t796;
  t1045 = 0. + t1039 + t1042;
  t1048 = t568*t1045;
  t1066 = -1.*t193*t888;
  t1086 = 0. + t1048 + t1066;
  t1092 = var2[3]*t1086;
  t1094 = 0. + var2[9] + var2[8] + t858 + t943 + t1034 + t1092;
  t1461 = var2[7]*t1385;
  t1655 = var2[5]*t1626;
  t1656 = var2[6]*t1626;
  t1918 = t124*t1814;
  t2028 = -1.*t197*t2007;
  t2029 = 0. + t1918 + t2028;
  t2074 = var2[4]*t2029;
  t2100 = -1.*t193*t1626;
  t2205 = t197*t1814;
  t2245 = t124*t2007;
  t2281 = 0. + t2205 + t2245;
  t2302 = t568*t2281;
  t2333 = 0. + t2100 + t2302;
  t2334 = var2[3]*t2333;
  t2359 = 0. + t1461 + t1655 + t1656 + t2074 + t2334;
  t2449 = var2[7]*t1518;
  t2510 = var2[5]*t2508;
  t2605 = var2[6]*t2508;
  t2722 = t124*t2709;
  t2858 = -1.*t197*t2824;
  t2863 = 0. + t2722 + t2858;
  t2959 = var2[4]*t2863;
  t2973 = -1.*t193*t2508;
  t3020 = t197*t2709;
  t3033 = t124*t2824;
  t3101 = 0. + t3020 + t3033;
  t3103 = t568*t3101;
  t3129 = 0. + t2973 + t3103;
  t3131 = var2[3]*t3129;
  t3136 = 0. + t2449 + t2510 + t2605 + t2959 + t3131;
  t3243 = t1292*t3212;
  t3286 = t3266*t1354;
  t3287 = t3243 + t3286;
  t3295 = t1292*t3266;
  t3309 = -1.*t3212*t1354;
  t3331 = t3295 + t3309;
  t1099 = 0.00334*t1094;
  t2394 = 3.e-6*t2359;
  t3148 = 1.e-6*t3136;
  t3169 = t1099 + t2394 + t3148;
  t3622 = -1.*t124*t171;
  t3635 = t89*t193*t197;
  t3640 = t3622 + t3635;
  t3654 = t65*t3640;
  t3667 = t89*t124*t193;
  t3681 = t171*t197;
  t3685 = t3667 + t3681;
  t3753 = t3685*t474;
  t3769 = t3654 + t3753;
  t3883 = t65*t3685;
  t3890 = -1.*t3640*t474;
  t3893 = t3883 + t3890;
  t3828 = t89*t568*t29;
  t3836 = t3769*t571;
  t3858 = t3828 + t3836;
  t3389 = 3.e-6*t1094;
  t3403 = 0.00216*t2359;
  t3417 = 0.000956*t3136;
  t3426 = t3389 + t3403 + t3417;
  t3861 = t1292*t3858;
  t3910 = t3893*t1354;
  t3911 = t3861 + t3910;
  t3971 = t1292*t3893;
  t3993 = -1.*t3858*t1354;
  t4009 = t3971 + t3993;
  t3513 = 1.e-6*t1094;
  t3567 = 0.000956*t2359;
  t3592 = 0.00144*t3136;
  t3595 = t3513 + t3567 + t3592;
  t3773 = -1.*t29*t3769;
  t3788 = t89*t568*t571;
  t3804 = t3773 + t3788;
  t3959 = t1346*t3911;
  t4013 = -1.*t1138*t4009;
  t4015 = t3959 + t4013;
  t4083 = -1.*t193*t1045;
  t4087 = -1.*t568*t888;
  t4136 = t4083 + t4087;
  t4195 = -1.*t568*t1626;
  t4209 = -1.*t193*t2281;
  t4233 = t4195 + t4209;
  t4247 = -1.*t568*t2508;
  t4273 = -1.*t193*t3101;
  t4287 = t4247 + t4273;
  t4023 = t1138*t3911;
  t4059 = t1346*t4009;
  t4069 = t4023 + t4059;
  t4489 = t89*t568*t65*t197;
  t4492 = t89*t568*t124*t474;
  t4494 = t4489 + t4492;
  t4554 = t89*t568*t124*t65;
  t4574 = -1.*t89*t568*t197*t474;
  t4581 = t4554 + t4574;
  t4529 = -1.*t89*t29*t193;
  t4539 = t4494*t571;
  t4542 = t4529 + t4539;
  t4543 = t1292*t4542;
  t4590 = t4581*t1354;
  t4612 = t4543 + t4590;
  t4618 = t1292*t4581;
  t4643 = -1.*t4542*t1354;
  t4652 = t4618 + t4643;
  t4142 = 0.00334*var2[3]*t4136;
  t4234 = 3.e-6*var2[3]*t4233;
  t4311 = 1.e-6*var2[3]*t4287;
  t4314 = t4142 + t4234 + t4311;
  t4765 = t89*t124;
  t4769 = t171*t193*t197;
  t4770 = t4765 + t4769;
  t4772 = t65*t4770;
  t4777 = t124*t171*t193;
  t4834 = -1.*t89*t197;
  t4866 = t4777 + t4834;
  t4881 = t4866*t474;
  t4885 = t4772 + t4881;
  t5040 = t65*t4866;
  t5071 = -1.*t4770*t474;
  t5111 = t5040 + t5071;
  t4945 = t568*t29*t171;
  t4980 = t4885*t571;
  t5036 = t4945 + t4980;
  t4318 = 3.e-6*var2[3]*t4136;
  t4329 = 0.00216*var2[3]*t4233;
  t4335 = 0.000956*var2[3]*t4287;
  t4339 = t4318 + t4329 + t4335;
  t5039 = t1292*t5036;
  t5115 = t5111*t1354;
  t5117 = t5039 + t5115;
  t5131 = t1292*t5111;
  t5154 = -1.*t5036*t1354;
  t5167 = t5131 + t5154;
  t4425 = 1.e-6*var2[3]*t4136;
  t4427 = 0.000956*var2[3]*t4233;
  t4441 = 0.00144*var2[3]*t4287;
  t4474 = t4425 + t4427 + t4441;
  t5329 = t568*t65*t171*t197;
  t5339 = t568*t124*t171*t474;
  t5357 = t5329 + t5339;
  t5731 = t568*t124*t65*t171;
  t5762 = -1.*t568*t171*t197*t474;
  t5765 = t5731 + t5762;
  t5604 = -1.*t29*t171*t193;
  t5610 = t5357*t571;
  t5692 = t5604 + t5610;
  t5703 = t1292*t5692;
  t5769 = t5765*t1354;
  t5797 = t5703 + t5769;
  t5804 = t1292*t5765;
  t5817 = -1.*t5692*t1354;
  t5818 = t5804 + t5817;
  t5952 = t568*t65*t197;
  t5958 = t568*t124*t474;
  t5962 = t5952 + t5958;
  t6058 = t568*t124*t65;
  t6069 = -1.*t568*t197*t474;
  t6074 = t6058 + t6069;
  t6012 = -1.*t29*t193;
  t6013 = t5962*t571;
  t6014 = t6012 + t6013;
  t6037 = t1292*t6014;
  t6093 = t6074*t1354;
  t6145 = t6037 + t6093;
  t6169 = t1292*t6074;
  t6170 = -1.*t6014*t1354;
  t6229 = t6169 + t6170;
  t6339 = -1.*t65*t193*t197;
  t6345 = -1.*t124*t193*t474;
  t6356 = t6339 + t6345;
  t6482 = -1.*t124*t65*t193;
  t6492 = t193*t197*t474;
  t6504 = t6482 + t6492;
  t6429 = -1.*t568*t29;
  t6455 = t6356*t571;
  t6460 = t6429 + t6455;
  t6461 = t1292*t6460;
  t6515 = t6504*t1354;
  t6534 = t6461 + t6515;
  t6543 = t1292*t6504;
  t6545 = -1.*t6460*t1354;
  t6554 = t6543 + t6545;
  t6771 = -1.*t767*t197;
  t6778 = -1.*t124*t796;
  t6785 = t6771 + t6778;
  t6791 = var2[4]*t6785;
  t6795 = t777 + t824;
  t6801 = var2[3]*t568*t6795;
  t6832 = t6791 + t6801;
  t6842 = -1.*t197*t1814;
  t6846 = -1.*t124*t2007;
  t6861 = t6842 + t6846;
  t6884 = var2[4]*t6861;
  t6902 = t1918 + t2028;
  t6909 = var2[3]*t568*t6902;
  t6920 = t6884 + t6909;
  t6930 = -1.*t197*t2709;
  t6934 = -1.*t124*t2824;
  t6938 = t6930 + t6934;
  t6940 = var2[4]*t6938;
  t6942 = t2722 + t2858;
  t6953 = var2[3]*t568*t6942;
  t6954 = t6940 + t6953;
  t7098 = t124*t171;
  t7160 = -1.*t89*t193*t197;
  t7180 = t7098 + t7160;
  t7181 = t7180*t474;
  t7182 = t3883 + t7181;
  t7226 = t65*t7180;
  t7228 = -1.*t3685*t474;
  t7229 = t7226 + t7228;
  t7217 = t1292*t7182*t571;
  t7235 = t7229*t1354;
  t7238 = t7217 + t7235;
  t7255 = t1292*t7229;
  t7261 = -1.*t7182*t571*t1354;
  t7265 = t7255 + t7261;
  t4890 = -1.*t29*t4885;
  t4926 = t568*t171*t571;
  t4927 = t4890 + t4926;
  t6841 = 0.00334*t6832;
  t6922 = 3.e-6*t6920;
  t6968 = 1.e-6*t6954;
  t6977 = t6841 + t6922 + t6968;
  t5118 = t1346*t5117;
  t5181 = -1.*t1138*t5167;
  t5182 = t5118 + t5181;
  t7057 = 3.e-6*t6832;
  t7059 = 0.00216*t6920;
  t7061 = 0.000956*t6954;
  t7064 = t7057 + t7059 + t7061;
  t5300 = t1138*t5117;
  t5307 = t1346*t5167;
  t5314 = t5300 + t5307;
  t7073 = 1.e-6*t6832;
  t7075 = 0.000956*t6920;
  t7082 = 0.00144*t6954;
  t7088 = t7073 + t7075 + t7082;
  t7365 = t232*t474;
  t7370 = t5040 + t7365;
  t7378 = -1.*t4866*t474;
  t7400 = t265 + t7378;
  t7373 = t1292*t7370*t571;
  t7406 = t7400*t1354;
  t7423 = t7373 + t7406;
  t7464 = t1292*t7400;
  t7471 = -1.*t7370*t571*t1354;
  t7476 = t7464 + t7471;
  t5976 = -1.*t29*t5962;
  t5982 = -1.*t193*t571;
  t5993 = t5976 + t5982;
  t6158 = t1346*t6145;
  t6237 = -1.*t1138*t6229;
  t6254 = t6158 + t6237;
  t6300 = t1138*t6145;
  t6320 = t1346*t6229;
  t6321 = t6300 + t6320;
  t7639 = -1.*t568*t65*t197;
  t7655 = -1.*t568*t124*t474;
  t7677 = t7639 + t7655;
  t7638 = t1292*t6074*t571;
  t7693 = t7677*t1354;
  t7699 = t7638 + t7693;
  t7727 = t1292*t7677;
  t7730 = -1.*t6074*t571*t1354;
  t7737 = t7727 + t7730;
  t7839 = t1661 + t1789;
  t7841 = -1.*t65*t1385;
  t7843 = -1.*t474*t1786;
  t7844 = t7841 + t7843;
  t7864 = t2617 + t2708;
  t7868 = -1.*t65*t1518;
  t7869 = -1.*t474*t2696;
  t7870 = t7868 + t7869;
  t7809 = -1.*t65*t695*t197;
  t7812 = -1.*t124*t695*t474;
  t7818 = t7809 + t7812;
  t7819 = var2[4]*t7818;
  t7820 = t124*t65*t695;
  t7821 = -1.*t695*t197*t474;
  t7824 = t7820 + t7821;
  t7833 = var2[3]*t568*t7824;
  t7835 = t7819 + t7833;
  t7840 = -1.*t197*t7839;
  t7847 = t124*t7844;
  t7848 = t7840 + t7847;
  t7857 = var2[4]*t7848;
  t7858 = t124*t7839;
  t7859 = t197*t7844;
  t7860 = t7858 + t7859;
  t7861 = var2[3]*t568*t7860;
  t7862 = t7857 + t7861;
  t7865 = -1.*t197*t7864;
  t7875 = t124*t7870;
  t7878 = t7865 + t7875;
  t7880 = var2[4]*t7878;
  t7881 = t124*t7864;
  t7882 = t197*t7870;
  t7883 = t7881 + t7882;
  t7886 = var2[3]*t568*t7883;
  t7888 = t7880 + t7886;
  t7929 = -1.*t65*t3640;
  t7933 = t7929 + t7228;
  t7926 = t1292*t3893*t571;
  t7934 = t7933*t1354;
  t7935 = t7926 + t7934;
  t7938 = t1292*t7933;
  t7939 = -1.*t3893*t571*t1354;
  t7942 = t7938 + t7939;
  t7838 = 0.00334*t7835;
  t7863 = 3.e-6*t7862;
  t7889 = 1.e-6*t7888;
  t7891 = t7838 + t7863 + t7889;
  t7893 = 3.e-6*t7835;
  t7897 = 0.00216*t7862;
  t7902 = 0.000956*t7888;
  t7907 = t7893 + t7897 + t7902;
  t7911 = 1.e-6*t7835;
  t7915 = 0.000956*t7862;
  t7916 = 0.00144*t7888;
  t7919 = t7911 + t7915 + t7916;
  t7990 = -1.*t65*t4770;
  t7993 = t7990 + t7378;
  t7988 = t1292*t5111*t571;
  t8004 = t7993*t1354;
  t8006 = t7988 + t8004;
  t8008 = t1292*t7993;
  t8009 = -1.*t5111*t571*t1354;
  t8013 = t8008 + t8009;
  t7637 = -1.*t29*t6074*t3169;
  t7705 = t1346*t7699;
  t7740 = -1.*t1138*t7737;
  t7741 = t7705 + t7740;
  t7745 = t7741*t3426;
  t7759 = t1138*t7699;
  t7771 = t1346*t7737;
  t7793 = t7759 + t7771;
  t7797 = t7793*t3595;
  t8033 = var2[5]*t29;
  t8034 = var2[6]*t29;
  t8036 = t124*t65*t571;
  t8038 = -1.*t197*t474*t571;
  t8039 = t8036 + t8038;
  t8040 = var2[4]*t8039;
  t8041 = t65*t197*t571;
  t8043 = t124*t474*t571;
  t8045 = t8041 + t8043;
  t8046 = t568*t8045;
  t8048 = t6012 + t8046;
  t8049 = var2[3]*t8048;
  t8051 = t8033 + t8034 + t8040 + t8049;
  t8053 = -1.*var2[5]*t571*t1518;
  t8054 = -1.*var2[6]*t571*t1518;
  t8057 = t124*t65*t29*t1518;
  t8058 = -1.*t29*t197*t474*t1518;
  t8060 = t8057 + t8058;
  t8061 = var2[4]*t8060;
  t8062 = t193*t571*t1518;
  t8064 = t65*t29*t197*t1518;
  t8065 = t124*t29*t474*t1518;
  t8066 = t8064 + t8065;
  t8070 = t568*t8066;
  t8071 = t8062 + t8070;
  t8073 = var2[3]*t8071;
  t8074 = t8053 + t8054 + t8061 + t8073;
  t8076 = -1.*var2[5]*t571*t2484;
  t8077 = -1.*var2[6]*t571*t2484;
  t8078 = t124*t65*t29*t2484;
  t8079 = -1.*t29*t197*t474*t2484;
  t8080 = t8078 + t8079;
  t8081 = var2[4]*t8080;
  t8082 = t193*t571*t2484;
  t8084 = t65*t29*t197*t2484;
  t8085 = t124*t29*t474*t2484;
  t8087 = t8084 + t8085;
  t8088 = t568*t8087;
  t8089 = t8082 + t8088;
  t8090 = var2[3]*t8089;
  t8092 = t8076 + t8077 + t8081 + t8090;
  t8116 = t29*t3769;
  t8117 = -1.*t89*t568*t571;
  t8118 = t8116 + t8117;
  t8052 = 0.00334*t8051;
  t8075 = 3.e-6*t8074;
  t8093 = 1.e-6*t8092;
  t8094 = t8052 + t8075 + t8093;
  t8096 = 3.e-6*t8051;
  t8099 = 0.00216*t8074;
  t8100 = 0.000956*t8092;
  t8101 = t8096 + t8099 + t8100;
  t8108 = 1.e-6*t8051;
  t8109 = 0.000956*t8074;
  t8110 = 0.00144*t8092;
  t8113 = t8108 + t8109 + t8110;
  t606 = -1.*t568*t171*t571;
  t8133 = t29*t4885;
  t8134 = t8133 + t606;
  t8161 = t29*t5962;
  t8162 = t193*t571;
  t8163 = t8161 + t8162;
  t8174 = -1.*t1292*t1346;
  t8175 = -1.*t1138*t1354;
  t8185 = t8174 + t8175;
  t8201 = t2457 + t2483;
  t8205 = t474*t571*t8185;
  t8206 = t65*t8201;
  t8207 = t8205 + t8206;
  t8215 = t65*t571*t8185;
  t8217 = -1.*t474*t8201;
  t8218 = t8215 + t8217;
  t8239 = t1503 + t1512;
  t8249 = -1.*t474*t8239;
  t8250 = t65*t571*t8201;
  t8255 = t8249 + t8250;
  t8257 = t65*t8239;
  t8258 = t474*t571*t8201;
  t8259 = t8257 + t8258;
  t8195 = var2[5]*t29*t8185;
  t8198 = var2[6]*t29*t8185;
  t8203 = var2[7]*t8201;
  t8214 = -1.*t197*t8207;
  t8220 = t124*t8218;
  t8221 = t8214 + t8220;
  t8222 = var2[4]*t8221;
  t8227 = -1.*t29*t193*t8185;
  t8228 = t124*t8207;
  t8229 = t197*t8218;
  t8230 = t8228 + t8229;
  t8233 = t568*t8230;
  t8235 = t8227 + t8233;
  t8236 = var2[3]*t8235;
  t8237 = t8195 + t8198 + t8203 + t8222 + t8236;
  t8241 = var2[7]*t8239;
  t8245 = var2[5]*t29*t8201;
  t8247 = var2[6]*t29*t8201;
  t8256 = t124*t8255;
  t8262 = -1.*t197*t8259;
  t8263 = t8256 + t8262;
  t8264 = var2[4]*t8263;
  t8266 = -1.*t29*t193*t8201;
  t8267 = t197*t8255;
  t8269 = t124*t8259;
  t8270 = t8267 + t8269;
  t8271 = t568*t8270;
  t8275 = t8266 + t8271;
  t8276 = var2[3]*t8275;
  t8278 = t8241 + t8245 + t8247 + t8264 + t8276;
  t8296 = -1.*t1292*t3858;
  t8301 = -1.*t3893*t1354;
  t8302 = t8296 + t8301;
  t8238 = 1.e-6*t8237;
  t8280 = 3.e-6*t8278;
  t8283 = t8238 + t8280;
  t8285 = 0.00144*t8237;
  t8286 = 0.000956*t8278;
  t8287 = t8285 + t8286;
  t8292 = 0.000956*t8237;
  t8293 = 0.00216*t8278;
  t8294 = t8292 + t8293;
  t8322 = -1.*t1292*t5036;
  t8324 = -1.*t5111*t1354;
  t8325 = t8322 + t8324;
  t8342 = -1.*t1292*t6014;
  t8343 = -1.*t6074*t1354;
  t8344 = t8342 + t8343;
  t8284 = t3804*t8283;
  t8288 = t4069*t8287;
  t8295 = t4015*t8294;
  t8308 = t1138*t4009;
  t8317 = t4927*t8283;
  t8318 = t5314*t8287;
  t8321 = t5182*t8294;
  t8332 = t1138*t5167;
  t8337 = t5993*t8283;
  t8339 = t6321*t8287;
  t8340 = t6254*t8294;
  t8350 = t1138*t6229;
  p_output1[0]=0;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=0;
  p_output1[9]=(t1346*t3287 - 1.*t1138*t3331)*t3426 + (t1138*t3287 + t1346*t3331)*t3595 + t3169*(-1.*t29*t550 + t606);
  p_output1[10]=t3169*t3804 + t3426*t4015 + t3595*t4069;
  p_output1[11]=0;
  p_output1[12]=t3804*t4314 + t4015*t4339 + t4069*t4474 + t3426*(t1346*t4612 - 1.*t1138*t4652) + t3595*(t1138*t4612 + t1346*t4652) + t3169*(-1.*t29*t4494 - 1.*t193*t571*t89);
  p_output1[13]=t4314*t4927 + t4339*t5182 + t4474*t5314 + t3169*(-1.*t29*t5357 - 1.*t171*t193*t571) + t3426*(t1346*t5797 - 1.*t1138*t5818) + t3595*(t1138*t5797 + t1346*t5818);
  p_output1[14]=t4314*t5993 + t4339*t6254 + t4474*t6321 + t3169*(-1.*t568*t571 - 1.*t29*t6356) + t3426*(t1346*t6534 - 1.*t1138*t6554) + t3595*(t1138*t6534 + t1346*t6554);
  p_output1[15]=t3804*t6977 + t4015*t7064 + t4069*t7088 - 1.*t29*t3169*t7182 + t3426*(t1346*t7238 - 1.*t1138*t7265) + t3595*(t1138*t7238 + t1346*t7265);
  p_output1[16]=t4927*t6977 + t5182*t7064 + t5314*t7088 - 1.*t29*t3169*t7370 + t3426*(t1346*t7423 - 1.*t1138*t7476) + t3595*(t1138*t7423 + t1346*t7476);
  p_output1[17]=t5993*t6977 + t6254*t7064 + t6321*t7088 + t7637 + t7745 + t7797;
  p_output1[18]=-1.*t29*t3169*t3893 + t3804*t7891 + t4015*t7907 + t4069*t7919 + t3426*(t1346*t7935 - 1.*t1138*t7942) + t3595*(t1138*t7935 + t1346*t7942);
  p_output1[19]=-1.*t29*t3169*t5111 + t4927*t7891 + t5182*t7907 + t5314*t7919 + t3426*(t1346*t8006 - 1.*t1138*t8013) + t3595*(t1138*t8006 + t1346*t8013);
  p_output1[20]=t7637 + t7745 + t7797 + t5993*t7891 + t6254*t7907 + t6321*t7919;
  p_output1[21]=t3169*t3858 + t3804*t8094 + t4015*t8101 + t4069*t8113 + t3426*(t1292*t1346*t8118 + t1138*t1354*t8118) + t3595*(t1138*t1292*t8118 - 1.*t1346*t1354*t8118);
  p_output1[22]=t3169*t5036 + t4927*t8094 + t5182*t8101 + t5314*t8113 + t3426*(t1292*t1346*t8134 + t1138*t1354*t8134) + t3595*(t1138*t1292*t8134 - 1.*t1346*t1354*t8134);
  p_output1[23]=t3169*t6014 + t5993*t8094 + t6254*t8101 + t6321*t8113 + t3426*(t1292*t1346*t8163 + t1138*t1354*t8163) + t3595*(t1138*t1292*t8163 - 1.*t1346*t1354*t8163);
  p_output1[24]=t8284 + t8288 + t8295 + t3426*(t4059 - 1.*t1138*t8302) + t3595*(t1346*t8302 + t8308);
  p_output1[25]=t8317 + t8318 + t8321 + t3426*(t5307 - 1.*t1138*t8325) + t3595*(t1346*t8325 + t8332);
  p_output1[26]=t8337 + t8339 + t8340 + t3426*(t6320 - 1.*t1138*t8344) + t3595*(t1346*t8344 + t8350);
  p_output1[27]=t3426*t4069 + t8284 + t8288 + t8295 + t3595*(-1.*t1346*t3911 + t8308);
  p_output1[28]=t3426*t5314 + t8317 + t8318 + t8321 + t3595*(-1.*t1346*t5117 + t8332);
  p_output1[29]=t3426*t6321 + t8337 + t8339 + t8340 + t3595*(-1.*t1346*t6145 + t8350);
  p_output1[30]=0;
  p_output1[31]=0;
  p_output1[32]=0;
  p_output1[33]=0;
  p_output1[34]=0;
  p_output1[35]=0;
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
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



void Jq_AMWorld_LeftKnee_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
