/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:31:46 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "dR_LeftToeBottom_src.h"

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
  double t319;
  double t251;
  double t471;
  double t299;
  double t479;
  double t233;
  double t235;
  double t312;
  double t491;
  double t577;
  double t730;
  double t748;
  double t759;
  double t788;
  double t850;
  double t914;
  double t926;
  double t972;
  double t996;
  double t1027;
  double t1097;
  double t1151;
  double t1225;
  double t194;
  double t220;
  double t1512;
  double t1569;
  double t1219;
  double t1515;
  double t1517;
  double t174;
  double t1635;
  double t1637;
  double t1690;
  double t1786;
  double t1558;
  double t1709;
  double t1724;
  double t140;
  double t1800;
  double t1835;
  double t1837;
  double t1869;
  double t1769;
  double t1842;
  double t1843;
  double t20;
  double t1911;
  double t1941;
  double t1973;
  double t2330;
  double t2369;
  double t2385;
  double t2495;
  double t2497;
  double t2501;
  double t2303;
  double t2408;
  double t2473;
  double t2486;
  double t2519;
  double t2556;
  double t2593;
  double t2594;
  double t2605;
  double t2579;
  double t2615;
  double t2632;
  double t2652;
  double t2726;
  double t2740;
  double t2650;
  double t2749;
  double t2760;
  double t2835;
  double t2841;
  double t2842;
  double t2773;
  double t2849;
  double t2852;
  double t2880;
  double t2897;
  double t2902;
  double t3022;
  double t3024;
  double t2990;
  double t3003;
  double t3017;
  double t3027;
  double t3048;
  double t3067;
  double t3070;
  double t3100;
  double t3057;
  double t3112;
  double t3131;
  double t3183;
  double t3195;
  double t3197;
  double t3157;
  double t3207;
  double t3255;
  double t3270;
  double t3273;
  double t3294;
  double t3260;
  double t3316;
  double t3317;
  double t3331;
  double t3359;
  double t3365;
  double t3556;
  double t3558;
  double t3561;
  double t3496;
  double t3501;
  double t3511;
  double t3512;
  double t3520;
  double t3526;
  double t3548;
  double t3562;
  double t3576;
  double t3583;
  double t3586;
  double t3594;
  double t3580;
  double t3596;
  double t3614;
  double t3639;
  double t3641;
  double t3662;
  double t3636;
  double t3666;
  double t3669;
  double t3682;
  double t3686;
  double t3710;
  double t3675;
  double t3725;
  double t3743;
  double t3787;
  double t3791;
  double t3793;
  double t3915;
  double t3918;
  double t3921;
  double t3893;
  double t3894;
  double t3901;
  double t4044;
  double t4051;
  double t4059;
  double t3892;
  double t3904;
  double t3923;
  double t3930;
  double t3998;
  double t4012;
  double t4041;
  double t4065;
  double t4076;
  double t4103;
  double t4110;
  double t4118;
  double t4099;
  double t4119;
  double t4120;
  double t4134;
  double t4137;
  double t4140;
  double t4125;
  double t4147;
  double t4168;
  double t4207;
  double t4209;
  double t4216;
  double t4188;
  double t4228;
  double t4260;
  double t4268;
  double t4276;
  double t4283;
  double t4383;
  double t4422;
  double t4429;
  double t4437;
  double t4440;
  double t4481;
  double t4502;
  double t4528;
  double t4531;
  double t4501;
  double t4535;
  double t4536;
  double t4538;
  double t4540;
  double t4543;
  double t4537;
  double t4546;
  double t4564;
  double t4608;
  double t4613;
  double t4615;
  double t4595;
  double t4627;
  double t4645;
  double t4678;
  double t4679;
  double t4687;
  double t4829;
  double t4830;
  double t4853;
  double t4891;
  double t4896;
  double t4931;
  double t4935;
  double t4922;
  double t4941;
  double t4956;
  double t4964;
  double t4968;
  double t4970;
  double t4962;
  double t4978;
  double t4981;
  double t4988;
  double t4990;
  double t5002;
  double t5163;
  double t5219;
  double t5243;
  double t5245;
  double t5247;
  double t5256;
  double t5265;
  double t5248;
  double t5272;
  double t5317;
  double t5325;
  double t5326;
  double t5332;
  double t5461;
  double t5479;
  double t5485;
  double t5489;
  double t5507;
  double t5509;
  double t5532;
  double t5508;
  double t5577;
  double t5680;
  double t5688;
  double t5690;
  double t5812;
  double t5816;
  double t5851;
  double t5860;
  double t5886;
  double t5887;
  double t5898;
  double t5904;
  double t5926;
  double t5929;
  double t5948;
  double t5954;
  double t5957;
  double t5980;
  double t6015;
  double t6021;
  double t6027;
  double t6031;
  double t6017;
  double t6033;
  double t6042;
  double t6084;
  double t6088;
  double t6102;
  double t6062;
  double t6114;
  double t6167;
  double t6199;
  double t6210;
  double t6218;
  double t6360;
  double t6362;
  double t6316;
  double t6327;
  double t6329;
  double t6354;
  double t6364;
  double t6389;
  double t6398;
  double t6410;
  double t6417;
  double t6394;
  double t6429;
  double t6439;
  double t6467;
  double t6470;
  double t6476;
  double t6457;
  double t6483;
  double t6487;
  double t6512;
  double t6526;
  double t6541;
  double t6507;
  double t6551;
  double t6562;
  double t6566;
  double t6581;
  double t6593;
  double t6742;
  double t6744;
  double t6709;
  double t6730;
  double t6731;
  double t6748;
  double t6757;
  double t6784;
  double t6789;
  double t6790;
  double t6761;
  double t6806;
  double t6818;
  double t6839;
  double t6843;
  double t6851;
  double t6833;
  double t6856;
  double t6874;
  double t6881;
  double t6884;
  double t6889;
  double t6878;
  double t6893;
  double t6900;
  double t6914;
  double t6915;
  double t6932;
  double t7059;
  double t7060;
  double t7069;
  double t7048;
  double t7051;
  double t7054;
  double t7055;
  double t7056;
  double t7057;
  double t7058;
  double t7082;
  double t7083;
  double t7107;
  double t7108;
  double t7115;
  double t7087;
  double t7121;
  double t7122;
  double t7132;
  double t7140;
  double t7150;
  double t7125;
  double t7151;
  double t7157;
  double t7164;
  double t7168;
  double t7171;
  double t7160;
  double t7172;
  double t7173;
  double t7181;
  double t7187;
  double t7189;
  double t7240;
  double t7252;
  double t7265;
  double t7281;
  double t7282;
  double t7283;
  double t7285;
  double t7308;
  double t7315;
  double t7284;
  double t7320;
  double t7321;
  double t7324;
  double t7327;
  double t7332;
  double t7322;
  double t7336;
  double t7341;
  double t7349;
  double t7359;
  double t7367;
  double t7347;
  double t7370;
  double t7371;
  double t7375;
  double t7381;
  double t7395;
  double t7447;
  double t7461;
  double t7466;
  double t7475;
  double t7480;
  double t7501;
  double t7502;
  double t7492;
  double t7506;
  double t7512;
  double t7515;
  double t7525;
  double t7532;
  double t7514;
  double t7533;
  double t7543;
  double t7555;
  double t7572;
  double t7580;
  double t7617;
  double t7619;
  double t7620;
  double t7622;
  double t7630;
  double t7642;
  double t7644;
  double t7639;
  double t7645;
  double t7648;
  double t7654;
  double t7664;
  double t7668;
  double t7711;
  double t7712;
  double t7714;
  double t7729;
  double t7736;
  double t7747;
  double t7749;
  double t7743;
  double t7763;
  double t7780;
  double t7781;
  double t7782;
  double t5782;
  double t5786;
  double t7858;
  double t7859;
  double t7860;
  double t7866;
  double t7869;
  double t7870;
  double t7871;
  double t7873;
  double t7875;
  double t7886;
  double t7889;
  double t7894;
  double t7884;
  double t7904;
  double t7908;
  double t7911;
  double t7917;
  double t7919;
  double t7910;
  double t7920;
  double t7921;
  double t7924;
  double t7926;
  double t7929;
  double t7965;
  double t7969;
  double t7970;
  double t7957;
  double t7958;
  double t7961;
  double t7962;
  double t7976;
  double t7977;
  double t7981;
  double t7989;
  double t8029;
  double t7980;
  double t8031;
  double t8034;
  double t8041;
  double t8045;
  double t8046;
  double t8035;
  double t8048;
  double t8049;
  double t8053;
  double t8057;
  double t8060;
  double t8050;
  double t8063;
  double t8064;
  double t8068;
  double t8069;
  double t8070;
  double t8067;
  double t8074;
  double t8077;
  double t8079;
  double t8081;
  double t8083;
  double t8084;
  double t8086;
  double t8087;
  double t8092;
  double t8093;
  double t8094;
  double t8095;
  double t8096;
  double t8097;
  double t8107;
  double t8110;
  double t8114;
  double t8106;
  double t8115;
  double t8116;
  double t8120;
  double t8121;
  double t8122;
  double t8117;
  double t8124;
  double t8125;
  double t8127;
  double t8128;
  double t8132;
  double t8126;
  double t8133;
  double t8134;
  double t8138;
  double t8140;
  double t8141;
  double t8154;
  double t8157;
  double t8158;
  double t8159;
  double t8160;
  double t8162;
  double t8163;
  double t8161;
  double t8165;
  double t8169;
  double t8171;
  double t8173;
  double t8174;
  double t8170;
  double t8177;
  double t8178;
  double t8180;
  double t8181;
  double t8182;
  double t8194;
  double t8195;
  double t8198;
  double t8199;
  double t8200;
  double t8202;
  double t8203;
  double t8201;
  double t8204;
  double t8205;
  double t8207;
  double t8208;
  double t8209;
  double t8221;
  double t8222;
  double t8223;
  double t8224;
  double t8225;
  double t8228;
  double t8229;
  double t8227;
  double t8234;
  double t8241;
  double t8259;
  double t8260;
  double t8281;
  double t8282;
  double t8283;
  double t8271;
  double t8272;
  double t8275;
  double t8276;
  double t8277;
  double t8278;
  double t8280;
  double t8284;
  double t8285;
  double t8287;
  double t8288;
  double t8289;
  double t8286;
  double t8291;
  double t8293;
  double t8295;
  double t8296;
  double t8297;
  double t8294;
  double t8298;
  double t8299;
  double t8304;
  double t8306;
  double t8307;
  double t8302;
  double t8308;
  double t8313;
  double t8316;
  double t8317;
  double t8318;
  double t1853;
  double t1974;
  double t1994;
  double t2095;
  double t2109;
  double t2142;
  double t2854;
  double t2903;
  double t2914;
  double t2942;
  double t2947;
  double t2952;
  double t3322;
  double t3374;
  double t3381;
  double t3399;
  double t3407;
  double t3424;
  double t3785;
  double t3804;
  double t3806;
  double t3825;
  double t3831;
  double t3832;
  double t4264;
  double t4289;
  double t4302;
  double t4327;
  double t4334;
  double t4335;
  double t4665;
  double t4692;
  double t4701;
  double t4704;
  double t4787;
  double t4788;
  double t4985;
  double t5012;
  double t5065;
  double t5101;
  double t5116;
  double t5122;
  double t5324;
  double t5337;
  double t5346;
  double t5369;
  double t5371;
  double t5373;
  double t5535;
  double t5554;
  double t5588;
  double t5650;
  double t5729;
  double t5737;
  double t6172;
  double t6235;
  double t6242;
  double t6259;
  double t6269;
  double t6279;
  double t6564;
  double t6612;
  double t6632;
  double t6660;
  double t6684;
  double t6687;
  double t6901;
  double t6980;
  double t6999;
  double t7012;
  double t7013;
  double t7014;
  double t7177;
  double t7203;
  double t7210;
  double t7229;
  double t7233;
  double t7234;
  double t7373;
  double t7401;
  double t7404;
  double t7412;
  double t7413;
  double t7415;
  double t7553;
  double t7582;
  double t7583;
  double t7594;
  double t7604;
  double t7605;
  double t7653;
  double t7670;
  double t7677;
  double t7685;
  double t7687;
  double t7694;
  double t7752;
  double t7755;
  double t7766;
  double t7769;
  double t7786;
  double t7791;
  double t7793;
  double t7799;
  double t7834;
  double t7835;
  double t7836;
  double t5787;
  double t7922;
  double t7930;
  double t7932;
  double t7944;
  double t7947;
  double t7948;
  double t8500;
  double t8504;
  double t8505;
  double t8136;
  double t8145;
  double t8146;
  double t8148;
  double t8149;
  double t8150;
  double t8179;
  double t8183;
  double t8184;
  double t8186;
  double t8188;
  double t8189;
  double t8206;
  double t8210;
  double t8211;
  double t8215;
  double t8216;
  double t8217;
  double t8230;
  double t8231;
  double t8235;
  double t8236;
  double t8261;
  double t8263;
  double t8265;
  double t8266;
  double t8315;
  double t8323;
  double t8327;
  double t8331;
  double t8332;
  double t8333;
  t319 = Cos(var1[3]);
  t251 = Cos(var1[5]);
  t471 = Sin(var1[4]);
  t299 = Sin(var1[3]);
  t479 = Sin(var1[5]);
  t233 = Cos(var1[7]);
  t235 = Cos(var1[6]);
  t312 = -1.*t251*t299;
  t491 = t319*t471*t479;
  t577 = t312 + t491;
  t730 = t235*t577;
  t748 = t319*t251*t471;
  t759 = t299*t479;
  t788 = t748 + t759;
  t850 = Sin(var1[6]);
  t914 = t788*t850;
  t926 = t730 + t914;
  t972 = t233*t926;
  t996 = Cos(var1[4]);
  t1027 = Sin(var1[7]);
  t1097 = -1.*t319*t996*t1027;
  t1151 = t972 + t1097;
  t1225 = Cos(var1[9]);
  t194 = Cos(var1[8]);
  t220 = Sin(var1[9]);
  t1512 = Sin(var1[8]);
  t1569 = Cos(var1[10]);
  t1219 = -1.*t194*t220*t1151;
  t1515 = -1.*t1225*t1151*t1512;
  t1517 = t1219 + t1515;
  t174 = Sin(var1[10]);
  t1635 = t1225*t194*t1151;
  t1637 = -1.*t220*t1151*t1512;
  t1690 = t1635 + t1637;
  t1786 = Cos(var1[11]);
  t1558 = t174*t1517;
  t1709 = t1569*t1690;
  t1724 = t1558 + t1709;
  t140 = Sin(var1[11]);
  t1800 = t1569*t1517;
  t1835 = -1.*t174*t1690;
  t1837 = t1800 + t1835;
  t1869 = Cos(var1[12]);
  t1769 = -1.*t140*t1724;
  t1842 = t1786*t1837;
  t1843 = t1769 + t1842;
  t20 = Sin(var1[12]);
  t1911 = t1786*t1724;
  t1941 = t140*t1837;
  t1973 = t1911 + t1941;
  t2330 = t251*t299;
  t2369 = -1.*t319*t471*t479;
  t2385 = t2330 + t2369;
  t2495 = t235*t2385;
  t2497 = -1.*t788*t850;
  t2501 = t2495 + t2497;
  t2303 = t235*t788;
  t2408 = t2385*t850;
  t2473 = t2303 + t2408;
  t2486 = t194*t2473*t1027;
  t2519 = t2501*t1512;
  t2556 = t2486 + t2519;
  t2593 = t194*t2501;
  t2594 = -1.*t2473*t1027*t1512;
  t2605 = t2593 + t2594;
  t2579 = -1.*t220*t2556;
  t2615 = t1225*t2605;
  t2632 = t2579 + t2615;
  t2652 = t1225*t2556;
  t2726 = t220*t2605;
  t2740 = t2652 + t2726;
  t2650 = t174*t2632;
  t2749 = t1569*t2740;
  t2760 = t2650 + t2749;
  t2835 = t1569*t2632;
  t2841 = -1.*t174*t2740;
  t2842 = t2835 + t2841;
  t2773 = -1.*t140*t2760;
  t2849 = t1786*t2842;
  t2852 = t2773 + t2849;
  t2880 = t1786*t2760;
  t2897 = t140*t2842;
  t2902 = t2880 + t2897;
  t3022 = -1.*t235*t577;
  t3024 = t3022 + t2497;
  t2990 = -1.*t577*t850;
  t3003 = t2303 + t2990;
  t3017 = t194*t3003*t1027;
  t3027 = t3024*t1512;
  t3048 = t3017 + t3027;
  t3067 = t194*t3024;
  t3070 = -1.*t3003*t1027*t1512;
  t3100 = t3067 + t3070;
  t3057 = -1.*t220*t3048;
  t3112 = t1225*t3100;
  t3131 = t3057 + t3112;
  t3183 = t1225*t3048;
  t3195 = t220*t3100;
  t3197 = t3183 + t3195;
  t3157 = t174*t3131;
  t3207 = t1569*t3197;
  t3255 = t3157 + t3207;
  t3270 = t1569*t3131;
  t3273 = -1.*t174*t3197;
  t3294 = t3270 + t3273;
  t3260 = -1.*t140*t3255;
  t3316 = t1786*t3294;
  t3317 = t3260 + t3316;
  t3331 = t1786*t3255;
  t3359 = t140*t3294;
  t3365 = t3331 + t3359;
  t3556 = t319*t996*t251*t235;
  t3558 = -1.*t319*t996*t479*t850;
  t3561 = t3556 + t3558;
  t3496 = -1.*t319*t233*t471;
  t3501 = t319*t996*t235*t479;
  t3511 = t319*t996*t251*t850;
  t3512 = t3501 + t3511;
  t3520 = t3512*t1027;
  t3526 = t3496 + t3520;
  t3548 = t194*t3526;
  t3562 = t3561*t1512;
  t3576 = t3548 + t3562;
  t3583 = t194*t3561;
  t3586 = -1.*t3526*t1512;
  t3594 = t3583 + t3586;
  t3580 = -1.*t220*t3576;
  t3596 = t1225*t3594;
  t3614 = t3580 + t3596;
  t3639 = t1225*t3576;
  t3641 = t220*t3594;
  t3662 = t3639 + t3641;
  t3636 = t174*t3614;
  t3666 = t1569*t3662;
  t3669 = t3636 + t3666;
  t3682 = t1569*t3614;
  t3686 = -1.*t174*t3662;
  t3710 = t3682 + t3686;
  t3675 = -1.*t140*t3669;
  t3725 = t1786*t3710;
  t3743 = t3675 + t3725;
  t3787 = t1786*t3669;
  t3791 = t140*t3710;
  t3793 = t3787 + t3791;
  t3915 = -1.*t251*t299*t471;
  t3918 = t319*t479;
  t3921 = t3915 + t3918;
  t3893 = -1.*t319*t251;
  t3894 = -1.*t299*t471*t479;
  t3901 = t3893 + t3894;
  t4044 = t235*t3921;
  t4051 = -1.*t3901*t850;
  t4059 = t4044 + t4051;
  t3892 = -1.*t996*t233*t299;
  t3904 = t235*t3901;
  t3923 = t3921*t850;
  t3930 = t3904 + t3923;
  t3998 = t3930*t1027;
  t4012 = t3892 + t3998;
  t4041 = t194*t4012;
  t4065 = t4059*t1512;
  t4076 = t4041 + t4065;
  t4103 = t194*t4059;
  t4110 = -1.*t4012*t1512;
  t4118 = t4103 + t4110;
  t4099 = -1.*t220*t4076;
  t4119 = t1225*t4118;
  t4120 = t4099 + t4119;
  t4134 = t1225*t4076;
  t4137 = t220*t4118;
  t4140 = t4134 + t4137;
  t4125 = t174*t4120;
  t4147 = t1569*t4140;
  t4168 = t4125 + t4147;
  t4207 = t1569*t4120;
  t4209 = -1.*t174*t4140;
  t4216 = t4207 + t4209;
  t4188 = -1.*t140*t4168;
  t4228 = t1786*t4216;
  t4260 = t4188 + t4228;
  t4268 = t1786*t4168;
  t4276 = t140*t4216;
  t4283 = t4268 + t4276;
  t4383 = t319*t996*t233;
  t4422 = t926*t1027;
  t4429 = t4383 + t4422;
  t4437 = -1.*t194*t4429;
  t4440 = -1.*t3003*t1512;
  t4481 = t4437 + t4440;
  t4502 = t194*t3003;
  t4528 = -1.*t4429*t1512;
  t4531 = t4502 + t4528;
  t4501 = t220*t4481;
  t4535 = t1225*t4531;
  t4536 = t4501 + t4535;
  t4538 = t1225*t4481;
  t4540 = -1.*t220*t4531;
  t4543 = t4538 + t4540;
  t4537 = -1.*t174*t4536;
  t4546 = t1569*t4543;
  t4564 = t4537 + t4546;
  t4608 = t1569*t4536;
  t4613 = t174*t4543;
  t4615 = t4608 + t4613;
  t4595 = t140*t4564;
  t4627 = t1786*t4615;
  t4645 = t4595 + t4627;
  t4678 = t1786*t4564;
  t4679 = -1.*t140*t4615;
  t4687 = t4678 + t4679;
  t4829 = t194*t4429;
  t4830 = t3003*t1512;
  t4853 = t4829 + t4830;
  t4891 = -1.*t220*t4853;
  t4896 = t4891 + t4535;
  t4931 = -1.*t1225*t4853;
  t4935 = t4931 + t4540;
  t4922 = -1.*t174*t4896;
  t4941 = t1569*t4935;
  t4956 = t4922 + t4941;
  t4964 = t1569*t4896;
  t4968 = t174*t4935;
  t4970 = t4964 + t4968;
  t4962 = t140*t4956;
  t4978 = t1786*t4970;
  t4981 = t4962 + t4978;
  t4988 = t1786*t4956;
  t4990 = -1.*t140*t4970;
  t5002 = t4988 + t4990;
  t5163 = t1225*t4853;
  t5219 = t220*t4531;
  t5243 = t5163 + t5219;
  t5245 = -1.*t1569*t5243;
  t5247 = t4922 + t5245;
  t5256 = -1.*t174*t5243;
  t5265 = t4964 + t5256;
  t5248 = t140*t5247;
  t5272 = t1786*t5265;
  t5317 = t5248 + t5272;
  t5325 = t1786*t5247;
  t5326 = -1.*t140*t5265;
  t5332 = t5325 + t5326;
  t5461 = t174*t4896;
  t5479 = t1569*t5243;
  t5485 = t5461 + t5479;
  t5489 = -1.*t140*t5485;
  t5507 = t5489 + t5272;
  t5509 = -1.*t1786*t5485;
  t5532 = t5509 + t5326;
  t5508 = -1.*t20*t5507;
  t5577 = t1869*t5507;
  t5680 = t1786*t5485;
  t5688 = t140*t5265;
  t5690 = t5680 + t5688;
  t5812 = t319*t251;
  t5816 = t299*t471*t479;
  t5851 = t5812 + t5816;
  t5860 = t235*t5851;
  t5886 = t251*t299*t471;
  t5887 = -1.*t319*t479;
  t5898 = t5886 + t5887;
  t5904 = t5898*t850;
  t5926 = t5860 + t5904;
  t5929 = t233*t5926;
  t5948 = -1.*t996*t299*t1027;
  t5954 = t5929 + t5948;
  t5957 = -1.*t194*t220*t5954;
  t5980 = -1.*t1225*t5954*t1512;
  t6015 = t5957 + t5980;
  t6021 = t1225*t194*t5954;
  t6027 = -1.*t220*t5954*t1512;
  t6031 = t6021 + t6027;
  t6017 = t174*t6015;
  t6033 = t1569*t6031;
  t6042 = t6017 + t6033;
  t6084 = t1569*t6015;
  t6088 = -1.*t174*t6031;
  t6102 = t6084 + t6088;
  t6062 = -1.*t140*t6042;
  t6114 = t1786*t6102;
  t6167 = t6062 + t6114;
  t6199 = t1786*t6042;
  t6210 = t140*t6102;
  t6218 = t6199 + t6210;
  t6360 = -1.*t5898*t850;
  t6362 = t3904 + t6360;
  t6316 = t235*t5898;
  t6327 = t3901*t850;
  t6329 = t6316 + t6327;
  t6354 = t194*t6329*t1027;
  t6364 = t6362*t1512;
  t6389 = t6354 + t6364;
  t6398 = t194*t6362;
  t6410 = -1.*t6329*t1027*t1512;
  t6417 = t6398 + t6410;
  t6394 = -1.*t220*t6389;
  t6429 = t1225*t6417;
  t6439 = t6394 + t6429;
  t6467 = t1225*t6389;
  t6470 = t220*t6417;
  t6476 = t6467 + t6470;
  t6457 = t174*t6439;
  t6483 = t1569*t6476;
  t6487 = t6457 + t6483;
  t6512 = t1569*t6439;
  t6526 = -1.*t174*t6476;
  t6541 = t6512 + t6526;
  t6507 = -1.*t140*t6487;
  t6551 = t1786*t6541;
  t6562 = t6507 + t6551;
  t6566 = t1786*t6487;
  t6581 = t140*t6541;
  t6593 = t6566 + t6581;
  t6742 = -1.*t235*t5851;
  t6744 = t6742 + t6360;
  t6709 = -1.*t5851*t850;
  t6730 = t6316 + t6709;
  t6731 = t194*t6730*t1027;
  t6748 = t6744*t1512;
  t6757 = t6731 + t6748;
  t6784 = t194*t6744;
  t6789 = -1.*t6730*t1027*t1512;
  t6790 = t6784 + t6789;
  t6761 = -1.*t220*t6757;
  t6806 = t1225*t6790;
  t6818 = t6761 + t6806;
  t6839 = t1225*t6757;
  t6843 = t220*t6790;
  t6851 = t6839 + t6843;
  t6833 = t174*t6818;
  t6856 = t1569*t6851;
  t6874 = t6833 + t6856;
  t6881 = t1569*t6818;
  t6884 = -1.*t174*t6851;
  t6889 = t6881 + t6884;
  t6878 = -1.*t140*t6874;
  t6893 = t1786*t6889;
  t6900 = t6878 + t6893;
  t6914 = t1786*t6874;
  t6915 = t140*t6889;
  t6932 = t6914 + t6915;
  t7059 = t996*t251*t235*t299;
  t7060 = -1.*t996*t299*t479*t850;
  t7069 = t7059 + t7060;
  t7048 = -1.*t233*t299*t471;
  t7051 = t996*t235*t299*t479;
  t7054 = t996*t251*t299*t850;
  t7055 = t7051 + t7054;
  t7056 = t7055*t1027;
  t7057 = t7048 + t7056;
  t7058 = t194*t7057;
  t7082 = t7069*t1512;
  t7083 = t7058 + t7082;
  t7107 = t194*t7069;
  t7108 = -1.*t7057*t1512;
  t7115 = t7107 + t7108;
  t7087 = -1.*t220*t7083;
  t7121 = t1225*t7115;
  t7122 = t7087 + t7121;
  t7132 = t1225*t7083;
  t7140 = t220*t7115;
  t7150 = t7132 + t7140;
  t7125 = t174*t7122;
  t7151 = t1569*t7150;
  t7157 = t7125 + t7151;
  t7164 = t1569*t7122;
  t7168 = -1.*t174*t7150;
  t7171 = t7164 + t7168;
  t7160 = -1.*t140*t7157;
  t7172 = t1786*t7171;
  t7173 = t7160 + t7172;
  t7181 = t1786*t7157;
  t7187 = t140*t7171;
  t7189 = t7181 + t7187;
  t7240 = t996*t233*t299;
  t7252 = t5926*t1027;
  t7265 = t7240 + t7252;
  t7281 = -1.*t194*t7265;
  t7282 = -1.*t6730*t1512;
  t7283 = t7281 + t7282;
  t7285 = t194*t6730;
  t7308 = -1.*t7265*t1512;
  t7315 = t7285 + t7308;
  t7284 = t220*t7283;
  t7320 = t1225*t7315;
  t7321 = t7284 + t7320;
  t7324 = t1225*t7283;
  t7327 = -1.*t220*t7315;
  t7332 = t7324 + t7327;
  t7322 = -1.*t174*t7321;
  t7336 = t1569*t7332;
  t7341 = t7322 + t7336;
  t7349 = t1569*t7321;
  t7359 = t174*t7332;
  t7367 = t7349 + t7359;
  t7347 = t140*t7341;
  t7370 = t1786*t7367;
  t7371 = t7347 + t7370;
  t7375 = t1786*t7341;
  t7381 = -1.*t140*t7367;
  t7395 = t7375 + t7381;
  t7447 = t194*t7265;
  t7461 = t6730*t1512;
  t7466 = t7447 + t7461;
  t7475 = -1.*t220*t7466;
  t7480 = t7475 + t7320;
  t7501 = -1.*t1225*t7466;
  t7502 = t7501 + t7327;
  t7492 = -1.*t174*t7480;
  t7506 = t1569*t7502;
  t7512 = t7492 + t7506;
  t7515 = t1569*t7480;
  t7525 = t174*t7502;
  t7532 = t7515 + t7525;
  t7514 = t140*t7512;
  t7533 = t1786*t7532;
  t7543 = t7514 + t7533;
  t7555 = t1786*t7512;
  t7572 = -1.*t140*t7532;
  t7580 = t7555 + t7572;
  t7617 = t1225*t7466;
  t7619 = t220*t7315;
  t7620 = t7617 + t7619;
  t7622 = -1.*t1569*t7620;
  t7630 = t7492 + t7622;
  t7642 = -1.*t174*t7620;
  t7644 = t7515 + t7642;
  t7639 = t140*t7630;
  t7645 = t1786*t7644;
  t7648 = t7639 + t7645;
  t7654 = t1786*t7630;
  t7664 = -1.*t140*t7644;
  t7668 = t7654 + t7664;
  t7711 = t174*t7480;
  t7712 = t1569*t7620;
  t7714 = t7711 + t7712;
  t7729 = -1.*t140*t7714;
  t7736 = t7729 + t7645;
  t7747 = -1.*t1786*t7714;
  t7749 = t7747 + t7664;
  t7743 = -1.*t20*t7736;
  t7763 = t1869*t7736;
  t7780 = t1786*t7714;
  t7781 = t140*t7644;
  t7782 = t7780 + t7781;
  t5782 = -1.*t20*t5690;
  t5786 = t5577 + t5782;
  t7858 = t996*t235*t479;
  t7859 = t996*t251*t850;
  t7860 = t7858 + t7859;
  t7866 = t233*t7860;
  t7869 = t471*t1027;
  t7870 = t7866 + t7869;
  t7871 = -1.*t194*t220*t7870;
  t7873 = -1.*t1225*t7870*t1512;
  t7875 = t7871 + t7873;
  t7886 = t1225*t194*t7870;
  t7889 = -1.*t220*t7870*t1512;
  t7894 = t7886 + t7889;
  t7884 = t174*t7875;
  t7904 = t1569*t7894;
  t7908 = t7884 + t7904;
  t7911 = t1569*t7875;
  t7917 = -1.*t174*t7894;
  t7919 = t7911 + t7917;
  t7910 = -1.*t140*t7908;
  t7920 = t1786*t7919;
  t7921 = t7910 + t7920;
  t7924 = t1786*t7908;
  t7926 = t140*t7919;
  t7929 = t7924 + t7926;
  t7965 = -1.*t996*t235*t479;
  t7969 = -1.*t996*t251*t850;
  t7970 = t7965 + t7969;
  t7957 = t996*t251*t235;
  t7958 = -1.*t996*t479*t850;
  t7961 = t7957 + t7958;
  t7962 = t194*t7961*t1027;
  t7976 = t7970*t1512;
  t7977 = t7962 + t7976;
  t7981 = t194*t7970;
  t7989 = -1.*t7961*t1027*t1512;
  t8029 = t7981 + t7989;
  t7980 = -1.*t220*t7977;
  t8031 = t1225*t8029;
  t8034 = t7980 + t8031;
  t8041 = t1225*t7977;
  t8045 = t220*t8029;
  t8046 = t8041 + t8045;
  t8035 = t174*t8034;
  t8048 = t1569*t8046;
  t8049 = t8035 + t8048;
  t8053 = t1569*t8034;
  t8057 = -1.*t174*t8046;
  t8060 = t8053 + t8057;
  t8050 = -1.*t140*t8049;
  t8063 = t1786*t8060;
  t8064 = t8050 + t8063;
  t8068 = t1786*t8049;
  t8069 = t140*t8060;
  t8070 = t8068 + t8069;
  t8067 = t20*t8064;
  t8074 = t1869*t8070;
  t8077 = t8067 + t8074;
  t8079 = 0.642788*t8077;
  t8081 = t1869*t8064;
  t8083 = -1.*t20*t8070;
  t8084 = t8081 + t8083;
  t8086 = 0.766044*t8084;
  t8087 = t8079 + t8086;
  t8092 = -1.*t233*t471;
  t8093 = t7860*t1027;
  t8094 = t8092 + t8093;
  t8095 = -1.*t194*t8094;
  t8096 = -1.*t7961*t1512;
  t8097 = t8095 + t8096;
  t8107 = t194*t7961;
  t8110 = -1.*t8094*t1512;
  t8114 = t8107 + t8110;
  t8106 = t220*t8097;
  t8115 = t1225*t8114;
  t8116 = t8106 + t8115;
  t8120 = t1225*t8097;
  t8121 = -1.*t220*t8114;
  t8122 = t8120 + t8121;
  t8117 = -1.*t174*t8116;
  t8124 = t1569*t8122;
  t8125 = t8117 + t8124;
  t8127 = t1569*t8116;
  t8128 = t174*t8122;
  t8132 = t8127 + t8128;
  t8126 = t140*t8125;
  t8133 = t1786*t8132;
  t8134 = t8126 + t8133;
  t8138 = t1786*t8125;
  t8140 = -1.*t140*t8132;
  t8141 = t8138 + t8140;
  t8154 = t194*t8094;
  t8157 = t7961*t1512;
  t8158 = t8154 + t8157;
  t8159 = -1.*t220*t8158;
  t8160 = t8159 + t8115;
  t8162 = -1.*t1225*t8158;
  t8163 = t8162 + t8121;
  t8161 = -1.*t174*t8160;
  t8165 = t1569*t8163;
  t8169 = t8161 + t8165;
  t8171 = t1569*t8160;
  t8173 = t174*t8163;
  t8174 = t8171 + t8173;
  t8170 = t140*t8169;
  t8177 = t1786*t8174;
  t8178 = t8170 + t8177;
  t8180 = t1786*t8169;
  t8181 = -1.*t140*t8174;
  t8182 = t8180 + t8181;
  t8194 = t1225*t8158;
  t8195 = t220*t8114;
  t8198 = t8194 + t8195;
  t8199 = -1.*t1569*t8198;
  t8200 = t8161 + t8199;
  t8202 = -1.*t174*t8198;
  t8203 = t8171 + t8202;
  t8201 = t140*t8200;
  t8204 = t1786*t8203;
  t8205 = t8201 + t8204;
  t8207 = t1786*t8200;
  t8208 = -1.*t140*t8203;
  t8209 = t8207 + t8208;
  t8221 = t174*t8160;
  t8222 = t1569*t8198;
  t8223 = t8221 + t8222;
  t8224 = -1.*t140*t8223;
  t8225 = t8224 + t8204;
  t8228 = -1.*t1786*t8223;
  t8229 = t8228 + t8208;
  t8227 = -1.*t20*t8225;
  t8234 = t1869*t8225;
  t8241 = t1786*t8223;
  t8259 = t140*t8203;
  t8260 = t8241 + t8259;
  t8281 = -1.*t251*t235*t471;
  t8282 = t471*t479*t850;
  t8283 = t8281 + t8282;
  t8271 = -1.*t996*t233;
  t8272 = -1.*t235*t471*t479;
  t8275 = -1.*t251*t471*t850;
  t8276 = t8272 + t8275;
  t8277 = t8276*t1027;
  t8278 = t8271 + t8277;
  t8280 = t194*t8278;
  t8284 = t8283*t1512;
  t8285 = t8280 + t8284;
  t8287 = t194*t8283;
  t8288 = -1.*t8278*t1512;
  t8289 = t8287 + t8288;
  t8286 = -1.*t220*t8285;
  t8291 = t1225*t8289;
  t8293 = t8286 + t8291;
  t8295 = t1225*t8285;
  t8296 = t220*t8289;
  t8297 = t8295 + t8296;
  t8294 = t174*t8293;
  t8298 = t1569*t8297;
  t8299 = t8294 + t8298;
  t8304 = t1569*t8293;
  t8306 = -1.*t174*t8297;
  t8307 = t8304 + t8306;
  t8302 = -1.*t140*t8299;
  t8308 = t1786*t8307;
  t8313 = t8302 + t8308;
  t8316 = t1786*t8299;
  t8317 = t140*t8307;
  t8318 = t8316 + t8317;
  t1853 = t20*t1843;
  t1974 = t1869*t1973;
  t1994 = t1853 + t1974;
  t2095 = t1869*t1843;
  t2109 = -1.*t20*t1973;
  t2142 = t2095 + t2109;
  t2854 = t20*t2852;
  t2903 = t1869*t2902;
  t2914 = t2854 + t2903;
  t2942 = t1869*t2852;
  t2947 = -1.*t20*t2902;
  t2952 = t2942 + t2947;
  t3322 = t20*t3317;
  t3374 = t1869*t3365;
  t3381 = t3322 + t3374;
  t3399 = t1869*t3317;
  t3407 = -1.*t20*t3365;
  t3424 = t3399 + t3407;
  t3785 = t20*t3743;
  t3804 = t1869*t3793;
  t3806 = t3785 + t3804;
  t3825 = t1869*t3743;
  t3831 = -1.*t20*t3793;
  t3832 = t3825 + t3831;
  t4264 = t20*t4260;
  t4289 = t1869*t4283;
  t4302 = t4264 + t4289;
  t4327 = t1869*t4260;
  t4334 = -1.*t20*t4283;
  t4335 = t4327 + t4334;
  t4665 = -1.*t20*t4645;
  t4692 = t1869*t4687;
  t4701 = t4665 + t4692;
  t4704 = t1869*t4645;
  t4787 = t20*t4687;
  t4788 = t4704 + t4787;
  t4985 = -1.*t20*t4981;
  t5012 = t1869*t5002;
  t5065 = t4985 + t5012;
  t5101 = t1869*t4981;
  t5116 = t20*t5002;
  t5122 = t5101 + t5116;
  t5324 = -1.*t20*t5317;
  t5337 = t1869*t5332;
  t5346 = t5324 + t5337;
  t5369 = t1869*t5317;
  t5371 = t20*t5332;
  t5373 = t5369 + t5371;
  t5535 = t1869*t5532;
  t5554 = t5508 + t5535;
  t5588 = t20*t5532;
  t5650 = t5577 + t5588;
  t5729 = -1.*t1869*t5690;
  t5737 = t5508 + t5729;
  t6172 = t20*t6167;
  t6235 = t1869*t6218;
  t6242 = t6172 + t6235;
  t6259 = t1869*t6167;
  t6269 = -1.*t20*t6218;
  t6279 = t6259 + t6269;
  t6564 = t20*t6562;
  t6612 = t1869*t6593;
  t6632 = t6564 + t6612;
  t6660 = t1869*t6562;
  t6684 = -1.*t20*t6593;
  t6687 = t6660 + t6684;
  t6901 = t20*t6900;
  t6980 = t1869*t6932;
  t6999 = t6901 + t6980;
  t7012 = t1869*t6900;
  t7013 = -1.*t20*t6932;
  t7014 = t7012 + t7013;
  t7177 = t20*t7173;
  t7203 = t1869*t7189;
  t7210 = t7177 + t7203;
  t7229 = t1869*t7173;
  t7233 = -1.*t20*t7189;
  t7234 = t7229 + t7233;
  t7373 = -1.*t20*t7371;
  t7401 = t1869*t7395;
  t7404 = t7373 + t7401;
  t7412 = t1869*t7371;
  t7413 = t20*t7395;
  t7415 = t7412 + t7413;
  t7553 = -1.*t20*t7543;
  t7582 = t1869*t7580;
  t7583 = t7553 + t7582;
  t7594 = t1869*t7543;
  t7604 = t20*t7580;
  t7605 = t7594 + t7604;
  t7653 = -1.*t20*t7648;
  t7670 = t1869*t7668;
  t7677 = t7653 + t7670;
  t7685 = t1869*t7648;
  t7687 = t20*t7668;
  t7694 = t7685 + t7687;
  t7752 = t1869*t7749;
  t7755 = t7743 + t7752;
  t7766 = t20*t7749;
  t7769 = t7763 + t7766;
  t7786 = -1.*t1869*t7782;
  t7791 = t7743 + t7786;
  t7793 = -1.*t20*t7782;
  t7799 = t7763 + t7793;
  t7834 = t20*t5507;
  t7835 = t1869*t5690;
  t7836 = t7834 + t7835;
  t5787 = 0.642788*t5786;
  t7922 = t20*t7921;
  t7930 = t1869*t7929;
  t7932 = t7922 + t7930;
  t7944 = t1869*t7921;
  t7947 = -1.*t20*t7929;
  t7948 = t7944 + t7947;
  t8500 = -0.766044*t8077;
  t8504 = 0.642788*t8084;
  t8505 = t8500 + t8504;
  t8136 = -1.*t20*t8134;
  t8145 = t1869*t8141;
  t8146 = t8136 + t8145;
  t8148 = t1869*t8134;
  t8149 = t20*t8141;
  t8150 = t8148 + t8149;
  t8179 = -1.*t20*t8178;
  t8183 = t1869*t8182;
  t8184 = t8179 + t8183;
  t8186 = t1869*t8178;
  t8188 = t20*t8182;
  t8189 = t8186 + t8188;
  t8206 = -1.*t20*t8205;
  t8210 = t1869*t8209;
  t8211 = t8206 + t8210;
  t8215 = t1869*t8205;
  t8216 = t20*t8209;
  t8217 = t8215 + t8216;
  t8230 = t1869*t8229;
  t8231 = t8227 + t8230;
  t8235 = t20*t8229;
  t8236 = t8234 + t8235;
  t8261 = -1.*t1869*t8260;
  t8263 = t8227 + t8261;
  t8265 = -1.*t20*t8260;
  t8266 = t8234 + t8265;
  t8315 = t20*t8313;
  t8323 = t1869*t8318;
  t8327 = t8315 + t8323;
  t8331 = t1869*t8313;
  t8332 = -1.*t20*t8318;
  t8333 = t8331 + t8332;
  p_output1[0]=(0.642788*t4302 + 0.766044*t4335)*var2[3] + (0.642788*t3806 + 0.766044*t3832)*var2[4] + (0.642788*t2914 + 0.766044*t2952)*var2[5] + (0.642788*t3381 + 0.766044*t3424)*var2[6] + (0.642788*t1994 + 0.766044*t2142)*var2[7] + (0.766044*t4701 + 0.642788*t4788)*var2[8] + (0.766044*t5065 + 0.642788*t5122)*var2[9] + (0.766044*t5346 + 0.642788*t5373)*var2[10] + (0.766044*t5554 + 0.642788*t5650)*var2[11] + (0.766044*t5737 + t5787)*var2[12];
  p_output1[1]=(0.766044*t5786 + 0.642788*t7836)*var2[3] + (0.642788*t7210 + 0.766044*t7234)*var2[4] + (0.642788*t6632 + 0.766044*t6687)*var2[5] + (0.642788*t6999 + 0.766044*t7014)*var2[6] + (0.642788*t6242 + 0.766044*t6279)*var2[7] + (0.766044*t7404 + 0.642788*t7415)*var2[8] + (0.766044*t7583 + 0.642788*t7605)*var2[9] + (0.766044*t7677 + 0.642788*t7694)*var2[10] + (0.766044*t7755 + 0.642788*t7769)*var2[11] + (0.766044*t7791 + 0.642788*t7799)*var2[12];
  p_output1[2]=(0.642788*t8327 + 0.766044*t8333)*var2[4] + t8087*var2[5] + t8087*var2[6] + (0.642788*t7932 + 0.766044*t7948)*var2[7] + (0.766044*t8146 + 0.642788*t8150)*var2[8] + (0.766044*t8184 + 0.642788*t8189)*var2[9] + (0.766044*t8211 + 0.642788*t8217)*var2[10] + (0.766044*t8231 + 0.642788*t8236)*var2[11] + (0.766044*t8263 + 0.642788*t8266)*var2[12];
  p_output1[3]=(t233*t3930 + t1027*t299*t996)*var2[3] + (t233*t3512 + t1027*t319*t471)*var2[4] + t233*t2473*var2[5] + t233*t3003*var2[6] + (-1.*t1027*t926 - 1.*t233*t319*t996)*var2[7];
  p_output1[4]=t1151*var2[3] + (t1027*t299*t471 + t233*t7055)*var2[4] + t233*t6329*var2[5] + t233*t6730*var2[6] + (t3892 - 1.*t1027*t5926)*var2[7];
  p_output1[5]=(t233*t8276 + t1027*t996)*var2[4] + t233*t7961*var2[5] + t233*t7961*var2[6] + (t233*t471 - 1.*t1027*t7860)*var2[7];
  p_output1[6]=(-0.766044*t4302 + 0.642788*t4335)*var2[3] + (-0.766044*t3806 + 0.642788*t3832)*var2[4] + (-0.766044*t2914 + 0.642788*t2952)*var2[5] + (-0.766044*t3381 + 0.642788*t3424)*var2[6] + (-0.766044*t1994 + 0.642788*t2142)*var2[7] + (0.642788*t4701 - 0.766044*t4788)*var2[8] + (0.642788*t5065 - 0.766044*t5122)*var2[9] + (0.642788*t5346 - 0.766044*t5373)*var2[10] + (0.642788*t5554 - 0.766044*t5650)*var2[11] + (0.642788*t5737 - 0.766044*t5786)*var2[12];
  p_output1[7]=(t5787 - 0.766044*t7836)*var2[3] + (-0.766044*t7210 + 0.642788*t7234)*var2[4] + (-0.766044*t6632 + 0.642788*t6687)*var2[5] + (-0.766044*t6999 + 0.642788*t7014)*var2[6] + (-0.766044*t6242 + 0.642788*t6279)*var2[7] + (0.642788*t7404 - 0.766044*t7415)*var2[8] + (0.642788*t7583 - 0.766044*t7605)*var2[9] + (0.642788*t7677 - 0.766044*t7694)*var2[10] + (0.642788*t7755 - 0.766044*t7769)*var2[11] + (0.642788*t7791 - 0.766044*t7799)*var2[12];
  p_output1[8]=(-0.766044*t8327 + 0.642788*t8333)*var2[4] + t8505*var2[5] + t8505*var2[6] + (-0.766044*t7932 + 0.642788*t7948)*var2[7] + (0.642788*t8146 - 0.766044*t8150)*var2[8] + (0.642788*t8184 - 0.766044*t8189)*var2[9] + (0.642788*t8211 - 0.766044*t8217)*var2[10] + (0.642788*t8231 - 0.766044*t8236)*var2[11] + (0.642788*t8263 - 0.766044*t8266)*var2[12];
}



void dR_LeftToeBottom_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
