/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:27:05 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "dR_LeftToe_src.h"

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
  double t486;
  double t227;
  double t501;
  double t454;
  double t522;
  double t132;
  double t226;
  double t470;
  double t576;
  double t613;
  double t624;
  double t741;
  double t760;
  double t801;
  double t807;
  double t810;
  double t821;
  double t843;
  double t850;
  double t854;
  double t869;
  double t879;
  double t1009;
  double t102;
  double t122;
  double t1016;
  double t1143;
  double t920;
  double t1088;
  double t1121;
  double t90;
  double t1216;
  double t1241;
  double t1259;
  double t1363;
  double t1125;
  double t1299;
  double t1303;
  double t62;
  double t1385;
  double t1387;
  double t1424;
  double t3;
  double t1799;
  double t1808;
  double t1883;
  double t2027;
  double t2032;
  double t2065;
  double t1753;
  double t1944;
  double t1951;
  double t1967;
  double t2123;
  double t2142;
  double t2167;
  double t2244;
  double t2246;
  double t2147;
  double t2295;
  double t2309;
  double t2413;
  double t2506;
  double t2729;
  double t1541;
  double t2325;
  double t2882;
  double t2905;
  double t3111;
  double t3272;
  double t3423;
  double t4038;
  double t4073;
  double t3973;
  double t3984;
  double t4006;
  double t4087;
  double t4114;
  double t4216;
  double t4326;
  double t4498;
  double t4126;
  double t4515;
  double t4518;
  double t4648;
  double t4757;
  double t4849;
  double t4529;
  double t4864;
  double t4867;
  double t5029;
  double t5050;
  double t5087;
  double t5678;
  double t5777;
  double t5823;
  double t5505;
  double t5507;
  double t5525;
  double t5526;
  double t5542;
  double t5555;
  double t5654;
  double t5831;
  double t5836;
  double t5978;
  double t5993;
  double t6018;
  double t5977;
  double t6030;
  double t6032;
  double t6126;
  double t6166;
  double t6169;
  double t6039;
  double t6170;
  double t6231;
  double t6250;
  double t6251;
  double t6286;
  double t6649;
  double t6732;
  double t6761;
  double t6567;
  double t6631;
  double t6645;
  double t6866;
  double t6882;
  double t6884;
  double t6508;
  double t6647;
  double t6770;
  double t6773;
  double t6777;
  double t6778;
  double t6824;
  double t6885;
  double t6912;
  double t6977;
  double t6985;
  double t6989;
  double t6957;
  double t6992;
  double t6996;
  double t7044;
  double t7055;
  double t7071;
  double t7012;
  double t7073;
  double t7080;
  double t7161;
  double t7162;
  double t7167;
  double t7227;
  double t7232;
  double t7235;
  double t7236;
  double t7311;
  double t7325;
  double t7328;
  double t7441;
  double t7442;
  double t7326;
  double t7471;
  double t7529;
  double t7548;
  double t7549;
  double t7550;
  double t7544;
  double t7559;
  double t7565;
  double t7614;
  double t7617;
  double t7619;
  double t7702;
  double t7718;
  double t7722;
  double t7724;
  double t7732;
  double t7737;
  double t7739;
  double t7736;
  double t7746;
  double t7767;
  double t7871;
  double t7878;
  double t7879;
  double t7909;
  double t7910;
  double t7912;
  double t7913;
  double t7916;
  double t7921;
  double t7923;
  double t7925;
  double t7941;
  double t7944;
  double t7945;
  double t7933;
  double t7949;
  double t7950;
  double t7951;
  double t7973;
  double t7975;
  double t7979;
  double t7980;
  double t7981;
  double t7982;
  double t7984;
  double t7985;
  double t7988;
  double t7991;
  double t7993;
  double t7994;
  double t7995;
  double t7996;
  double t7997;
  double t8002;
  double t8004;
  double t8006;
  double t8001;
  double t8010;
  double t8014;
  double t8018;
  double t8019;
  double t8020;
  double t8043;
  double t8045;
  double t8039;
  double t8040;
  double t8041;
  double t8042;
  double t8050;
  double t8051;
  double t8055;
  double t8057;
  double t8058;
  double t8052;
  double t8059;
  double t8061;
  double t8065;
  double t8066;
  double t8069;
  double t8062;
  double t8070;
  double t8073;
  double t8078;
  double t8087;
  double t8092;
  double t8171;
  double t8173;
  double t8162;
  double t8167;
  double t8170;
  double t8176;
  double t8177;
  double t8184;
  double t8185;
  double t8186;
  double t8183;
  double t8188;
  double t8190;
  double t8192;
  double t8195;
  double t8196;
  double t8191;
  double t8197;
  double t8198;
  double t8200;
  double t8202;
  double t8204;
  double t8232;
  double t8234;
  double t8236;
  double t8219;
  double t8221;
  double t8222;
  double t8223;
  double t8226;
  double t8227;
  double t8231;
  double t8237;
  double t8238;
  double t8241;
  double t8243;
  double t8244;
  double t8240;
  double t8249;
  double t8250;
  double t8254;
  double t8257;
  double t8259;
  double t8251;
  double t8260;
  double t8261;
  double t8265;
  double t8266;
  double t8267;
  double t8288;
  double t8291;
  double t8292;
  double t8293;
  double t8295;
  double t8297;
  double t8299;
  double t8305;
  double t8306;
  double t8298;
  double t8307;
  double t8311;
  double t8328;
  double t8339;
  double t8342;
  double t8314;
  double t8344;
  double t8345;
  double t8353;
  double t8354;
  double t8356;
  double t8371;
  double t8372;
  double t8373;
  double t8374;
  double t8375;
  double t8377;
  double t8378;
  double t8376;
  double t8379;
  double t8380;
  double t8382;
  double t8383;
  double t8384;
  double t8397;
  double t8398;
  double t8401;
  double t8402;
  double t8403;
  double t8405;
  double t8406;
  double t8407;
  double t8419;
  double t8420;
  double t8421;
  double t8412;
  double t8422;
  double t8423;
  double t8424;
  double t7961;
  double t7962;
  double t7966;
  double t8443;
  double t8444;
  double t8445;
  double t8446;
  double t8447;
  double t8448;
  double t8449;
  double t8450;
  double t8451;
  double t8453;
  double t8455;
  double t8456;
  double t8452;
  double t8457;
  double t8460;
  double t8462;
  double t8463;
  double t8464;
  double t8479;
  double t8481;
  double t8482;
  double t8475;
  double t8476;
  double t8477;
  double t8478;
  double t8483;
  double t8484;
  double t8486;
  double t8487;
  double t8488;
  double t8485;
  double t8490;
  double t8491;
  double t8494;
  double t8496;
  double t8498;
  double t8492;
  double t8499;
  double t8501;
  double t8503;
  double t8504;
  double t8508;
  double t8502;
  double t8509;
  double t8510;
  double t8511;
  double t8512;
  double t8513;
  double t8514;
  double t8515;
  double t8516;
  double t8519;
  double t8520;
  double t8521;
  double t8522;
  double t8523;
  double t8524;
  double t8526;
  double t8529;
  double t8531;
  double t8525;
  double t8532;
  double t8533;
  double t8535;
  double t8536;
  double t8537;
  double t8534;
  double t8539;
  double t8540;
  double t8544;
  double t8548;
  double t8549;
  double t8564;
  double t8567;
  double t8569;
  double t8570;
  double t8572;
  double t8575;
  double t8576;
  double t8573;
  double t8578;
  double t8579;
  double t8582;
  double t8583;
  double t8586;
  double t8601;
  double t8602;
  double t8604;
  double t8605;
  double t8606;
  double t8608;
  double t8609;
  double t8611;
  double t8623;
  double t8624;
  double t8625;
  double t8616;
  double t8626;
  double t8627;
  double t8628;
  double t8651;
  double t8652;
  double t8653;
  double t8641;
  double t8643;
  double t8645;
  double t8646;
  double t8647;
  double t8649;
  double t8650;
  double t8654;
  double t8656;
  double t8658;
  double t8659;
  double t8660;
  double t8657;
  double t8661;
  double t8662;
  double t8664;
  double t8665;
  double t8666;
  double t8663;
  double t8667;
  double t8668;
  double t8670;
  double t8671;
  double t8672;
  double t1323;
  double t1428;
  double t1461;
  double t1568;
  double t1616;
  double t1619;
  double t3000;
  double t3469;
  double t3627;
  double t3702;
  double t3733;
  double t3739;
  double t4911;
  double t5107;
  double t5157;
  double t5163;
  double t5173;
  double t5209;
  double t6246;
  double t6290;
  double t6301;
  double t6346;
  double t6352;
  double t6391;
  double t7143;
  double t7171;
  double t7194;
  double t7201;
  double t7211;
  double t7218;
  double t7569;
  double t7635;
  double t7641;
  double t7646;
  double t7657;
  double t7659;
  double t7804;
  double t7880;
  double t7887;
  double t7894;
  double t7897;
  double t7901;
  double t7920;
  double t7930;
  double t7932;
  double t7934;
  double t7952;
  double t7953;
  double t8730;
  double t8016;
  double t8021;
  double t8024;
  double t8029;
  double t8030;
  double t8031;
  double t8077;
  double t8100;
  double t8109;
  double t8137;
  double t8139;
  double t8140;
  double t8199;
  double t8205;
  double t8206;
  double t8211;
  double t8212;
  double t8215;
  double t8263;
  double t8268;
  double t8269;
  double t8272;
  double t8274;
  double t8279;
  double t8351;
  double t8358;
  double t8359;
  double t8364;
  double t8365;
  double t8366;
  double t8381;
  double t8387;
  double t8388;
  double t8391;
  double t8392;
  double t8393;
  double t8404;
  double t8408;
  double t8410;
  double t8413;
  double t8426;
  double t8427;
  double t8772;
  double t8431;
  double t8432;
  double t8433;
  double t7968;
  double t7970;
  double t8461;
  double t8466;
  double t8467;
  double t8469;
  double t8470;
  double t8471;
  double t8786;
  double t8787;
  double t8788;
  double t8543;
  double t8550;
  double t8553;
  double t8555;
  double t8556;
  double t8557;
  double t8580;
  double t8588;
  double t8590;
  double t8593;
  double t8594;
  double t8595;
  double t8607;
  double t8612;
  double t8614;
  double t8619;
  double t8629;
  double t8630;
  double t8805;
  double t8634;
  double t8635;
  double t8636;
  double t8669;
  double t8673;
  double t8674;
  double t8676;
  double t8677;
  double t8680;
  t486 = Cos(var1[3]);
  t227 = Cos(var1[5]);
  t501 = Sin(var1[4]);
  t454 = Sin(var1[3]);
  t522 = Sin(var1[5]);
  t132 = Cos(var1[7]);
  t226 = Cos(var1[6]);
  t470 = -1.*t227*t454;
  t576 = t486*t501*t522;
  t613 = t470 + t576;
  t624 = t226*t613;
  t741 = t486*t227*t501;
  t760 = t454*t522;
  t801 = t741 + t760;
  t807 = Sin(var1[6]);
  t810 = t801*t807;
  t821 = t624 + t810;
  t843 = t132*t821;
  t850 = Cos(var1[4]);
  t854 = Sin(var1[7]);
  t869 = -1.*t486*t850*t854;
  t879 = t843 + t869;
  t1009 = Cos(var1[9]);
  t102 = Cos(var1[8]);
  t122 = Sin(var1[9]);
  t1016 = Sin(var1[8]);
  t1143 = Cos(var1[10]);
  t920 = -1.*t102*t122*t879;
  t1088 = -1.*t1009*t879*t1016;
  t1121 = t920 + t1088;
  t90 = Sin(var1[10]);
  t1216 = t1009*t102*t879;
  t1241 = -1.*t122*t879*t1016;
  t1259 = t1216 + t1241;
  t1363 = Cos(var1[11]);
  t1125 = t90*t1121;
  t1299 = t1143*t1259;
  t1303 = t1125 + t1299;
  t62 = Sin(var1[11]);
  t1385 = t1143*t1121;
  t1387 = -1.*t90*t1259;
  t1424 = t1385 + t1387;
  t3 = Cos(var1[12]);
  t1799 = t227*t454;
  t1808 = -1.*t486*t501*t522;
  t1883 = t1799 + t1808;
  t2027 = t226*t1883;
  t2032 = -1.*t801*t807;
  t2065 = t2027 + t2032;
  t1753 = t226*t801;
  t1944 = t1883*t807;
  t1951 = t1753 + t1944;
  t1967 = t102*t1951*t854;
  t2123 = t2065*t1016;
  t2142 = t1967 + t2123;
  t2167 = t102*t2065;
  t2244 = -1.*t1951*t854*t1016;
  t2246 = t2167 + t2244;
  t2147 = -1.*t122*t2142;
  t2295 = t1009*t2246;
  t2309 = t2147 + t2295;
  t2413 = t1009*t2142;
  t2506 = t122*t2246;
  t2729 = t2413 + t2506;
  t1541 = Sin(var1[12]);
  t2325 = t90*t2309;
  t2882 = t1143*t2729;
  t2905 = t2325 + t2882;
  t3111 = t1143*t2309;
  t3272 = -1.*t90*t2729;
  t3423 = t3111 + t3272;
  t4038 = -1.*t226*t613;
  t4073 = t4038 + t2032;
  t3973 = -1.*t613*t807;
  t3984 = t1753 + t3973;
  t4006 = t102*t3984*t854;
  t4087 = t4073*t1016;
  t4114 = t4006 + t4087;
  t4216 = t102*t4073;
  t4326 = -1.*t3984*t854*t1016;
  t4498 = t4216 + t4326;
  t4126 = -1.*t122*t4114;
  t4515 = t1009*t4498;
  t4518 = t4126 + t4515;
  t4648 = t1009*t4114;
  t4757 = t122*t4498;
  t4849 = t4648 + t4757;
  t4529 = t90*t4518;
  t4864 = t1143*t4849;
  t4867 = t4529 + t4864;
  t5029 = t1143*t4518;
  t5050 = -1.*t90*t4849;
  t5087 = t5029 + t5050;
  t5678 = t486*t850*t227*t226;
  t5777 = -1.*t486*t850*t522*t807;
  t5823 = t5678 + t5777;
  t5505 = -1.*t486*t132*t501;
  t5507 = t486*t850*t226*t522;
  t5525 = t486*t850*t227*t807;
  t5526 = t5507 + t5525;
  t5542 = t5526*t854;
  t5555 = t5505 + t5542;
  t5654 = t102*t5555;
  t5831 = t5823*t1016;
  t5836 = t5654 + t5831;
  t5978 = t102*t5823;
  t5993 = -1.*t5555*t1016;
  t6018 = t5978 + t5993;
  t5977 = -1.*t122*t5836;
  t6030 = t1009*t6018;
  t6032 = t5977 + t6030;
  t6126 = t1009*t5836;
  t6166 = t122*t6018;
  t6169 = t6126 + t6166;
  t6039 = t90*t6032;
  t6170 = t1143*t6169;
  t6231 = t6039 + t6170;
  t6250 = t1143*t6032;
  t6251 = -1.*t90*t6169;
  t6286 = t6250 + t6251;
  t6649 = -1.*t227*t454*t501;
  t6732 = t486*t522;
  t6761 = t6649 + t6732;
  t6567 = -1.*t486*t227;
  t6631 = -1.*t454*t501*t522;
  t6645 = t6567 + t6631;
  t6866 = t226*t6761;
  t6882 = -1.*t6645*t807;
  t6884 = t6866 + t6882;
  t6508 = -1.*t850*t132*t454;
  t6647 = t226*t6645;
  t6770 = t6761*t807;
  t6773 = t6647 + t6770;
  t6777 = t6773*t854;
  t6778 = t6508 + t6777;
  t6824 = t102*t6778;
  t6885 = t6884*t1016;
  t6912 = t6824 + t6885;
  t6977 = t102*t6884;
  t6985 = -1.*t6778*t1016;
  t6989 = t6977 + t6985;
  t6957 = -1.*t122*t6912;
  t6992 = t1009*t6989;
  t6996 = t6957 + t6992;
  t7044 = t1009*t6912;
  t7055 = t122*t6989;
  t7071 = t7044 + t7055;
  t7012 = t90*t6996;
  t7073 = t1143*t7071;
  t7080 = t7012 + t7073;
  t7161 = t1143*t6996;
  t7162 = -1.*t90*t7071;
  t7167 = t7161 + t7162;
  t7227 = t486*t850*t132;
  t7232 = t821*t854;
  t7235 = t7227 + t7232;
  t7236 = -1.*t102*t7235;
  t7311 = -1.*t3984*t1016;
  t7325 = t7236 + t7311;
  t7328 = t102*t3984;
  t7441 = -1.*t7235*t1016;
  t7442 = t7328 + t7441;
  t7326 = t122*t7325;
  t7471 = t1009*t7442;
  t7529 = t7326 + t7471;
  t7548 = t1009*t7325;
  t7549 = -1.*t122*t7442;
  t7550 = t7548 + t7549;
  t7544 = -1.*t90*t7529;
  t7559 = t1143*t7550;
  t7565 = t7544 + t7559;
  t7614 = t1143*t7529;
  t7617 = t90*t7550;
  t7619 = t7614 + t7617;
  t7702 = t102*t7235;
  t7718 = t3984*t1016;
  t7722 = t7702 + t7718;
  t7724 = -1.*t122*t7722;
  t7732 = t7724 + t7471;
  t7737 = -1.*t1009*t7722;
  t7739 = t7737 + t7549;
  t7736 = -1.*t90*t7732;
  t7746 = t1143*t7739;
  t7767 = t7736 + t7746;
  t7871 = t1143*t7732;
  t7878 = t90*t7739;
  t7879 = t7871 + t7878;
  t7909 = t1009*t7722;
  t7910 = t122*t7442;
  t7912 = t7909 + t7910;
  t7913 = -1.*t1143*t7912;
  t7916 = t7736 + t7913;
  t7921 = -1.*t90*t7912;
  t7923 = t7871 + t7921;
  t7925 = t1363*t7923;
  t7941 = t90*t7732;
  t7944 = t1143*t7912;
  t7945 = t7941 + t7944;
  t7933 = -1.*t62*t7923;
  t7949 = -1.*t62*t7945;
  t7950 = t7949 + t7925;
  t7951 = t1541*t7950;
  t7973 = t486*t227;
  t7975 = t454*t501*t522;
  t7979 = t7973 + t7975;
  t7980 = t226*t7979;
  t7981 = t227*t454*t501;
  t7982 = -1.*t486*t522;
  t7984 = t7981 + t7982;
  t7985 = t7984*t807;
  t7988 = t7980 + t7985;
  t7991 = t132*t7988;
  t7993 = -1.*t850*t454*t854;
  t7994 = t7991 + t7993;
  t7995 = -1.*t102*t122*t7994;
  t7996 = -1.*t1009*t7994*t1016;
  t7997 = t7995 + t7996;
  t8002 = t1009*t102*t7994;
  t8004 = -1.*t122*t7994*t1016;
  t8006 = t8002 + t8004;
  t8001 = t90*t7997;
  t8010 = t1143*t8006;
  t8014 = t8001 + t8010;
  t8018 = t1143*t7997;
  t8019 = -1.*t90*t8006;
  t8020 = t8018 + t8019;
  t8043 = -1.*t7984*t807;
  t8045 = t6647 + t8043;
  t8039 = t226*t7984;
  t8040 = t6645*t807;
  t8041 = t8039 + t8040;
  t8042 = t102*t8041*t854;
  t8050 = t8045*t1016;
  t8051 = t8042 + t8050;
  t8055 = t102*t8045;
  t8057 = -1.*t8041*t854*t1016;
  t8058 = t8055 + t8057;
  t8052 = -1.*t122*t8051;
  t8059 = t1009*t8058;
  t8061 = t8052 + t8059;
  t8065 = t1009*t8051;
  t8066 = t122*t8058;
  t8069 = t8065 + t8066;
  t8062 = t90*t8061;
  t8070 = t1143*t8069;
  t8073 = t8062 + t8070;
  t8078 = t1143*t8061;
  t8087 = -1.*t90*t8069;
  t8092 = t8078 + t8087;
  t8171 = -1.*t226*t7979;
  t8173 = t8171 + t8043;
  t8162 = -1.*t7979*t807;
  t8167 = t8039 + t8162;
  t8170 = t102*t8167*t854;
  t8176 = t8173*t1016;
  t8177 = t8170 + t8176;
  t8184 = t102*t8173;
  t8185 = -1.*t8167*t854*t1016;
  t8186 = t8184 + t8185;
  t8183 = -1.*t122*t8177;
  t8188 = t1009*t8186;
  t8190 = t8183 + t8188;
  t8192 = t1009*t8177;
  t8195 = t122*t8186;
  t8196 = t8192 + t8195;
  t8191 = t90*t8190;
  t8197 = t1143*t8196;
  t8198 = t8191 + t8197;
  t8200 = t1143*t8190;
  t8202 = -1.*t90*t8196;
  t8204 = t8200 + t8202;
  t8232 = t850*t227*t226*t454;
  t8234 = -1.*t850*t454*t522*t807;
  t8236 = t8232 + t8234;
  t8219 = -1.*t132*t454*t501;
  t8221 = t850*t226*t454*t522;
  t8222 = t850*t227*t454*t807;
  t8223 = t8221 + t8222;
  t8226 = t8223*t854;
  t8227 = t8219 + t8226;
  t8231 = t102*t8227;
  t8237 = t8236*t1016;
  t8238 = t8231 + t8237;
  t8241 = t102*t8236;
  t8243 = -1.*t8227*t1016;
  t8244 = t8241 + t8243;
  t8240 = -1.*t122*t8238;
  t8249 = t1009*t8244;
  t8250 = t8240 + t8249;
  t8254 = t1009*t8238;
  t8257 = t122*t8244;
  t8259 = t8254 + t8257;
  t8251 = t90*t8250;
  t8260 = t1143*t8259;
  t8261 = t8251 + t8260;
  t8265 = t1143*t8250;
  t8266 = -1.*t90*t8259;
  t8267 = t8265 + t8266;
  t8288 = t850*t132*t454;
  t8291 = t7988*t854;
  t8292 = t8288 + t8291;
  t8293 = -1.*t102*t8292;
  t8295 = -1.*t8167*t1016;
  t8297 = t8293 + t8295;
  t8299 = t102*t8167;
  t8305 = -1.*t8292*t1016;
  t8306 = t8299 + t8305;
  t8298 = t122*t8297;
  t8307 = t1009*t8306;
  t8311 = t8298 + t8307;
  t8328 = t1009*t8297;
  t8339 = -1.*t122*t8306;
  t8342 = t8328 + t8339;
  t8314 = -1.*t90*t8311;
  t8344 = t1143*t8342;
  t8345 = t8314 + t8344;
  t8353 = t1143*t8311;
  t8354 = t90*t8342;
  t8356 = t8353 + t8354;
  t8371 = t102*t8292;
  t8372 = t8167*t1016;
  t8373 = t8371 + t8372;
  t8374 = -1.*t122*t8373;
  t8375 = t8374 + t8307;
  t8377 = -1.*t1009*t8373;
  t8378 = t8377 + t8339;
  t8376 = -1.*t90*t8375;
  t8379 = t1143*t8378;
  t8380 = t8376 + t8379;
  t8382 = t1143*t8375;
  t8383 = t90*t8378;
  t8384 = t8382 + t8383;
  t8397 = t1009*t8373;
  t8398 = t122*t8306;
  t8401 = t8397 + t8398;
  t8402 = -1.*t1143*t8401;
  t8403 = t8376 + t8402;
  t8405 = -1.*t90*t8401;
  t8406 = t8382 + t8405;
  t8407 = t1363*t8406;
  t8419 = t90*t8375;
  t8420 = t1143*t8401;
  t8421 = t8419 + t8420;
  t8412 = -1.*t62*t8406;
  t8422 = -1.*t62*t8421;
  t8423 = t8422 + t8407;
  t8424 = t1541*t8423;
  t7961 = t1363*t7945;
  t7962 = t62*t7923;
  t7966 = t7961 + t7962;
  t8443 = t850*t226*t522;
  t8444 = t850*t227*t807;
  t8445 = t8443 + t8444;
  t8446 = t132*t8445;
  t8447 = t501*t854;
  t8448 = t8446 + t8447;
  t8449 = -1.*t102*t122*t8448;
  t8450 = -1.*t1009*t8448*t1016;
  t8451 = t8449 + t8450;
  t8453 = t1009*t102*t8448;
  t8455 = -1.*t122*t8448*t1016;
  t8456 = t8453 + t8455;
  t8452 = t90*t8451;
  t8457 = t1143*t8456;
  t8460 = t8452 + t8457;
  t8462 = t1143*t8451;
  t8463 = -1.*t90*t8456;
  t8464 = t8462 + t8463;
  t8479 = -1.*t850*t226*t522;
  t8481 = -1.*t850*t227*t807;
  t8482 = t8479 + t8481;
  t8475 = t850*t227*t226;
  t8476 = -1.*t850*t522*t807;
  t8477 = t8475 + t8476;
  t8478 = t102*t8477*t854;
  t8483 = t8482*t1016;
  t8484 = t8478 + t8483;
  t8486 = t102*t8482;
  t8487 = -1.*t8477*t854*t1016;
  t8488 = t8486 + t8487;
  t8485 = -1.*t122*t8484;
  t8490 = t1009*t8488;
  t8491 = t8485 + t8490;
  t8494 = t1009*t8484;
  t8496 = t122*t8488;
  t8498 = t8494 + t8496;
  t8492 = t90*t8491;
  t8499 = t1143*t8498;
  t8501 = t8492 + t8499;
  t8503 = t1143*t8491;
  t8504 = -1.*t90*t8498;
  t8508 = t8503 + t8504;
  t8502 = -1.*t62*t8501;
  t8509 = t1363*t8508;
  t8510 = t8502 + t8509;
  t8511 = -1.*t3*t8510;
  t8512 = t1363*t8501;
  t8513 = t62*t8508;
  t8514 = t8512 + t8513;
  t8515 = t1541*t8514;
  t8516 = t8511 + t8515;
  t8519 = -1.*t132*t501;
  t8520 = t8445*t854;
  t8521 = t8519 + t8520;
  t8522 = -1.*t102*t8521;
  t8523 = -1.*t8477*t1016;
  t8524 = t8522 + t8523;
  t8526 = t102*t8477;
  t8529 = -1.*t8521*t1016;
  t8531 = t8526 + t8529;
  t8525 = t122*t8524;
  t8532 = t1009*t8531;
  t8533 = t8525 + t8532;
  t8535 = t1009*t8524;
  t8536 = -1.*t122*t8531;
  t8537 = t8535 + t8536;
  t8534 = -1.*t90*t8533;
  t8539 = t1143*t8537;
  t8540 = t8534 + t8539;
  t8544 = t1143*t8533;
  t8548 = t90*t8537;
  t8549 = t8544 + t8548;
  t8564 = t102*t8521;
  t8567 = t8477*t1016;
  t8569 = t8564 + t8567;
  t8570 = -1.*t122*t8569;
  t8572 = t8570 + t8532;
  t8575 = -1.*t1009*t8569;
  t8576 = t8575 + t8536;
  t8573 = -1.*t90*t8572;
  t8578 = t1143*t8576;
  t8579 = t8573 + t8578;
  t8582 = t1143*t8572;
  t8583 = t90*t8576;
  t8586 = t8582 + t8583;
  t8601 = t1009*t8569;
  t8602 = t122*t8531;
  t8604 = t8601 + t8602;
  t8605 = -1.*t1143*t8604;
  t8606 = t8573 + t8605;
  t8608 = -1.*t90*t8604;
  t8609 = t8582 + t8608;
  t8611 = t1363*t8609;
  t8623 = t90*t8572;
  t8624 = t1143*t8604;
  t8625 = t8623 + t8624;
  t8616 = -1.*t62*t8609;
  t8626 = -1.*t62*t8625;
  t8627 = t8626 + t8611;
  t8628 = t1541*t8627;
  t8651 = -1.*t227*t226*t501;
  t8652 = t501*t522*t807;
  t8653 = t8651 + t8652;
  t8641 = -1.*t850*t132;
  t8643 = -1.*t226*t501*t522;
  t8645 = -1.*t227*t501*t807;
  t8646 = t8643 + t8645;
  t8647 = t8646*t854;
  t8649 = t8641 + t8647;
  t8650 = t102*t8649;
  t8654 = t8653*t1016;
  t8656 = t8650 + t8654;
  t8658 = t102*t8653;
  t8659 = -1.*t8649*t1016;
  t8660 = t8658 + t8659;
  t8657 = -1.*t122*t8656;
  t8661 = t1009*t8660;
  t8662 = t8657 + t8661;
  t8664 = t1009*t8656;
  t8665 = t122*t8660;
  t8666 = t8664 + t8665;
  t8663 = t90*t8662;
  t8667 = t1143*t8666;
  t8668 = t8663 + t8667;
  t8670 = t1143*t8662;
  t8671 = -1.*t90*t8666;
  t8672 = t8670 + t8671;
  t1323 = -1.*t62*t1303;
  t1428 = t1363*t1424;
  t1461 = t1323 + t1428;
  t1568 = t1363*t1303;
  t1616 = t62*t1424;
  t1619 = t1568 + t1616;
  t3000 = -1.*t62*t2905;
  t3469 = t1363*t3423;
  t3627 = t3000 + t3469;
  t3702 = t1363*t2905;
  t3733 = t62*t3423;
  t3739 = t3702 + t3733;
  t4911 = -1.*t62*t4867;
  t5107 = t1363*t5087;
  t5157 = t4911 + t5107;
  t5163 = t1363*t4867;
  t5173 = t62*t5087;
  t5209 = t5163 + t5173;
  t6246 = -1.*t62*t6231;
  t6290 = t1363*t6286;
  t6301 = t6246 + t6290;
  t6346 = t1363*t6231;
  t6352 = t62*t6286;
  t6391 = t6346 + t6352;
  t7143 = -1.*t62*t7080;
  t7171 = t1363*t7167;
  t7194 = t7143 + t7171;
  t7201 = t1363*t7080;
  t7211 = t62*t7167;
  t7218 = t7201 + t7211;
  t7569 = t62*t7565;
  t7635 = t1363*t7619;
  t7641 = t7569 + t7635;
  t7646 = t1363*t7565;
  t7657 = -1.*t62*t7619;
  t7659 = t7646 + t7657;
  t7804 = t62*t7767;
  t7880 = t1363*t7879;
  t7887 = t7804 + t7880;
  t7894 = t1363*t7767;
  t7897 = -1.*t62*t7879;
  t7901 = t7894 + t7897;
  t7920 = t62*t7916;
  t7930 = t7920 + t7925;
  t7932 = t1363*t7916;
  t7934 = t7932 + t7933;
  t7952 = -1.*t1363*t7945;
  t7953 = t7952 + t7933;
  t8730 = t3*t7950;
  t8016 = -1.*t62*t8014;
  t8021 = t1363*t8020;
  t8024 = t8016 + t8021;
  t8029 = t1363*t8014;
  t8030 = t62*t8020;
  t8031 = t8029 + t8030;
  t8077 = -1.*t62*t8073;
  t8100 = t1363*t8092;
  t8109 = t8077 + t8100;
  t8137 = t1363*t8073;
  t8139 = t62*t8092;
  t8140 = t8137 + t8139;
  t8199 = -1.*t62*t8198;
  t8205 = t1363*t8204;
  t8206 = t8199 + t8205;
  t8211 = t1363*t8198;
  t8212 = t62*t8204;
  t8215 = t8211 + t8212;
  t8263 = -1.*t62*t8261;
  t8268 = t1363*t8267;
  t8269 = t8263 + t8268;
  t8272 = t1363*t8261;
  t8274 = t62*t8267;
  t8279 = t8272 + t8274;
  t8351 = t62*t8345;
  t8358 = t1363*t8356;
  t8359 = t8351 + t8358;
  t8364 = t1363*t8345;
  t8365 = -1.*t62*t8356;
  t8366 = t8364 + t8365;
  t8381 = t62*t8380;
  t8387 = t1363*t8384;
  t8388 = t8381 + t8387;
  t8391 = t1363*t8380;
  t8392 = -1.*t62*t8384;
  t8393 = t8391 + t8392;
  t8404 = t62*t8403;
  t8408 = t8404 + t8407;
  t8410 = t1363*t8403;
  t8413 = t8410 + t8412;
  t8426 = -1.*t1363*t8421;
  t8427 = t8426 + t8412;
  t8772 = t3*t8423;
  t8431 = t1363*t8421;
  t8432 = t62*t8406;
  t8433 = t8431 + t8432;
  t7968 = t3*t7966;
  t7970 = t7951 + t7968;
  t8461 = -1.*t62*t8460;
  t8466 = t1363*t8464;
  t8467 = t8461 + t8466;
  t8469 = t1363*t8460;
  t8470 = t62*t8464;
  t8471 = t8469 + t8470;
  t8786 = t1541*t8510;
  t8787 = t3*t8514;
  t8788 = t8786 + t8787;
  t8543 = t62*t8540;
  t8550 = t1363*t8549;
  t8553 = t8543 + t8550;
  t8555 = t1363*t8540;
  t8556 = -1.*t62*t8549;
  t8557 = t8555 + t8556;
  t8580 = t62*t8579;
  t8588 = t1363*t8586;
  t8590 = t8580 + t8588;
  t8593 = t1363*t8579;
  t8594 = -1.*t62*t8586;
  t8595 = t8593 + t8594;
  t8607 = t62*t8606;
  t8612 = t8607 + t8611;
  t8614 = t1363*t8606;
  t8619 = t8614 + t8616;
  t8629 = -1.*t1363*t8625;
  t8630 = t8629 + t8616;
  t8805 = t3*t8627;
  t8634 = t1363*t8625;
  t8635 = t62*t8609;
  t8636 = t8634 + t8635;
  t8669 = -1.*t62*t8668;
  t8673 = t1363*t8672;
  t8674 = t8669 + t8673;
  t8676 = t1363*t8668;
  t8677 = t62*t8672;
  t8680 = t8676 + t8677;
  p_output1[0]=(-1.*t3*t7194 + t1541*t7218)*var2[3] + (-1.*t3*t6301 + t1541*t6391)*var2[4] + (-1.*t3*t3627 + t1541*t3739)*var2[5] + (-1.*t3*t5157 + t1541*t5209)*var2[6] + (t1541*t1619 - 1.*t1461*t3)*var2[7] + (t1541*t7641 - 1.*t3*t7659)*var2[8] + (t1541*t7887 - 1.*t3*t7901)*var2[9] + (t1541*t7930 - 1.*t3*t7934)*var2[10] + (t7951 - 1.*t3*t7953)*var2[11] + t7970*var2[12];
  p_output1[1]=(-1.*t3*t7950 + t1541*t7966)*var2[3] + (-1.*t3*t8269 + t1541*t8279)*var2[4] + (-1.*t3*t8109 + t1541*t8140)*var2[5] + (-1.*t3*t8206 + t1541*t8215)*var2[6] + (-1.*t3*t8024 + t1541*t8031)*var2[7] + (t1541*t8359 - 1.*t3*t8366)*var2[8] + (t1541*t8388 - 1.*t3*t8393)*var2[9] + (t1541*t8408 - 1.*t3*t8413)*var2[10] + (t8424 - 1.*t3*t8427)*var2[11] + (t8424 + t3*t8433)*var2[12];
  p_output1[2]=(-1.*t3*t8674 + t1541*t8680)*var2[4] + t8516*var2[5] + t8516*var2[6] + (-1.*t3*t8467 + t1541*t8471)*var2[7] + (t1541*t8553 - 1.*t3*t8557)*var2[8] + (t1541*t8590 - 1.*t3*t8595)*var2[9] + (t1541*t8612 - 1.*t3*t8619)*var2[10] + (t8628 - 1.*t3*t8630)*var2[11] + (t8628 + t3*t8636)*var2[12];
  p_output1[3]=(t1541*t7194 + t3*t7218)*var2[3] + (t1541*t6301 + t3*t6391)*var2[4] + (t1541*t3627 + t3*t3739)*var2[5] + (t1541*t5157 + t3*t5209)*var2[6] + (t1461*t1541 + t1619*t3)*var2[7] + (t3*t7641 + t1541*t7659)*var2[8] + (t3*t7887 + t1541*t7901)*var2[9] + (t3*t7930 + t1541*t7934)*var2[10] + (t1541*t7953 + t8730)*var2[11] + (-1.*t1541*t7966 + t8730)*var2[12];
  p_output1[4]=t7970*var2[3] + (t1541*t8269 + t3*t8279)*var2[4] + (t1541*t8109 + t3*t8140)*var2[5] + (t1541*t8206 + t3*t8215)*var2[6] + (t1541*t8024 + t3*t8031)*var2[7] + (t3*t8359 + t1541*t8366)*var2[8] + (t3*t8388 + t1541*t8393)*var2[9] + (t3*t8408 + t1541*t8413)*var2[10] + (t1541*t8427 + t8772)*var2[11] + (-1.*t1541*t8433 + t8772)*var2[12];
  p_output1[5]=(t1541*t8674 + t3*t8680)*var2[4] + t8788*var2[5] + t8788*var2[6] + (t1541*t8467 + t3*t8471)*var2[7] + (t3*t8553 + t1541*t8557)*var2[8] + (t3*t8590 + t1541*t8595)*var2[9] + (t3*t8612 + t1541*t8619)*var2[10] + (t1541*t8630 + t8805)*var2[11] + (-1.*t1541*t8636 + t8805)*var2[12];
  p_output1[6]=(-1.*t132*t6773 + t7993)*var2[3] + (-1.*t132*t5526 - 1.*t486*t501*t854)*var2[4] - 1.*t132*t1951*var2[5] - 1.*t132*t3984*var2[6] + t7235*var2[7];
  p_output1[7]=(-1.*t132*t821 + t486*t850*t854)*var2[3] + (-1.*t132*t8223 - 1.*t454*t501*t854)*var2[4] - 1.*t132*t8041*var2[5] - 1.*t132*t8167*var2[6] + t8292*var2[7];
  p_output1[8]=(-1.*t850*t854 - 1.*t132*t8646)*var2[4] - 1.*t132*t8477*var2[5] - 1.*t132*t8477*var2[6] + t8521*var2[7];
}



void dR_LeftToe_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}