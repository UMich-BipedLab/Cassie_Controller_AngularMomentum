/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:25:59 GMT-04:00
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
  double t679;
  double t1896;
  double t2176;
  double t344;
  double t2364;
  double t170;
  double t8;
  double t4203;
  double t4246;
  double t3108;
  double t3921;
  double t4623;
  double t3763;
  double t4256;
  double t4295;
  double t4327;
  double t4478;
  double t1331;
  double t2708;
  double t2734;
  double t2843;
  double t2923;
  double t2971;
  double t3073;
  double t3213;
  double t3249;
  double t6103;
  double t6366;
  double t6420;
  double t6586;
  double t6122;
  double t6193;
  double t6253;
  double t6661;
  double t6784;
  double t6257;
  double t6708;
  double t6735;
  double t5781;
  double t6801;
  double t6830;
  double t6852;
  double t6919;
  double t5697;
  double t7019;
  double t7028;
  double t7040;
  double t7002;
  double t7013;
  double t7015;
  double t7077;
  double t7094;
  double t7102;
  double t7126;
  double t7143;
  double t7018;
  double t7066;
  double t7070;
  double t7154;
  double t7171;
  double t7153;
  double t7173;
  double t7194;
  double t7204;
  double t7208;
  double t7211;
  double t7298;
  double t7299;
  double t7300;
  double t7323;
  double t7329;
  double t7330;
  double t7334;
  double t7303;
  double t7304;
  double t7310;
  double t7349;
  double t7354;
  double t7346;
  double t7358;
  double t7361;
  double t7369;
  double t7371;
  double t7373;
  double t6741;
  double t6877;
  double t6895;
  double t6946;
  double t6947;
  double t6987;
  double t7072;
  double t7145;
  double t7148;
  double t7203;
  double t7221;
  double t7226;
  double t7229;
  double t7232;
  double t7244;
  double t7248;
  double t7259;
  double t7276;
  double t7278;
  double t7285;
  double t7286;
  double t7315;
  double t7338;
  double t7341;
  double t7366;
  double t7375;
  double t7376;
  double t7379;
  double t7381;
  double t7387;
  double t7388;
  double t7392;
  double t7393;
  double t7398;
  double t7402;
  double t7403;
  double t4301;
  double t4491;
  double t4610;
  double t4622;
  double t4677;
  double t4798;
  double t4862;
  double t4888;
  double t5061;
  double t5165;
  double t5240;
  double t5287;
  double t5325;
  double t5350;
  double t5384;
  double t5536;
  double t7437;
  double t7438;
  double t7440;
  double t7441;
  double t7442;
  double t7443;
  double t7449;
  double t7450;
  double t7451;
  double t7472;
  double t7473;
  double t7475;
  double t7463;
  double t7465;
  double t7466;
  double t7469;
  double t7476;
  double t7480;
  double t7484;
  double t7488;
  double t7490;
  double t7291;
  double t7405;
  double t7407;
  double t7482;
  double t7491;
  double t7493;
  double t7495;
  double t7496;
  double t7497;
  double t7424;
  double t7427;
  double t7431;
  double t7452;
  double t7453;
  double t7455;
  double t7494;
  double t7498;
  double t7500;
  double t7502;
  double t7504;
  double t7505;
  double t7521;
  double t7522;
  double t7523;
  double t7527;
  double t7529;
  double t7530;
  double t7514;
  double t7515;
  double t7516;
  double t7545;
  double t7546;
  double t7547;
  double t7541;
  double t7542;
  double t7543;
  double t7544;
  double t7552;
  double t7553;
  double t7555;
  double t7556;
  double t7558;
  double t7554;
  double t7559;
  double t7560;
  double t7562;
  double t7563;
  double t7564;
  double t7508;
  double t7510;
  double t7512;
  double t7575;
  double t7576;
  double t7577;
  double t7578;
  double t7579;
  double t7580;
  double t7581;
  double t7582;
  double t7583;
  double t7603;
  double t7604;
  double t7605;
  double t7598;
  double t7599;
  double t7601;
  double t7602;
  double t7606;
  double t7607;
  double t7609;
  double t7611;
  double t7615;
  double t7524;
  double t7531;
  double t7534;
  double t7608;
  double t7616;
  double t7617;
  double t7619;
  double t7620;
  double t7621;
  double t7536;
  double t7537;
  double t7539;
  double t7589;
  double t7590;
  double t7593;
  double t7634;
  double t7635;
  double t7637;
  double t7629;
  double t7630;
  double t7631;
  double t7632;
  double t7638;
  double t7639;
  double t7642;
  double t7643;
  double t7645;
  double t7640;
  double t7646;
  double t7647;
  double t7649;
  double t7650;
  double t7651;
  double t7661;
  double t7663;
  double t7664;
  double t7685;
  double t7686;
  double t7688;
  double t7679;
  double t7681;
  double t7682;
  double t7683;
  double t7689;
  double t7690;
  double t7694;
  double t7696;
  double t7697;
  double t7691;
  double t7703;
  double t7706;
  double t7709;
  double t7710;
  double t7712;
  double t7670;
  double t7671;
  double t7672;
  double t7729;
  double t7731;
  double t7732;
  double t7725;
  double t7726;
  double t7727;
  double t7728;
  double t7733;
  double t7734;
  double t7737;
  double t7742;
  double t7744;
  double t7736;
  double t7745;
  double t7746;
  double t7748;
  double t7749;
  double t7750;
  double t7787;
  double t7788;
  double t7789;
  double t7790;
  double t7791;
  double t7792;
  double t7793;
  double t7795;
  double t7796;
  double t7797;
  double t7798;
  double t7799;
  double t7800;
  double t7801;
  double t7777;
  double t7778;
  double t7780;
  double t7781;
  double t7783;
  double t7812;
  double t7813;
  double t7814;
  double t7811;
  double t7815;
  double t7816;
  double t7818;
  double t7821;
  double t7823;
  double t7817;
  double t7824;
  double t7826;
  double t7830;
  double t7832;
  double t7834;
  double t7763;
  double t7765;
  double t7766;
  double t7767;
  double t7770;
  double t7773;
  double t7774;
  double t7584;
  double t7585;
  double t7586;
  double t7618;
  double t7622;
  double t7623;
  double t7794;
  double t7803;
  double t7805;
  double t7625;
  double t7626;
  double t7627;
  double t7807;
  double t7808;
  double t7809;
  double t7849;
  double t7850;
  double t7856;
  double t7857;
  double t7854;
  double t7858;
  double t7860;
  double t7862;
  double t7863;
  double t7864;
  double t7861;
  double t7865;
  double t7870;
  double t7874;
  double t7875;
  double t7877;
  double t7665;
  double t7667;
  double t7668;
  double t7707;
  double t7715;
  double t7716;
  double t7718;
  double t7719;
  double t7720;
  double t7896;
  double t7897;
  double t7898;
  double t7894;
  double t7899;
  double t7900;
  double t7902;
  double t7903;
  double t7904;
  double t7901;
  double t7905;
  double t7906;
  double t7908;
  double t7912;
  double t7913;
  double t7939;
  double t7943;
  double t7945;
  double t7948;
  double t7965;
  double t7967;
  double t7968;
  double t7969;
  double t7942;
  double t7949;
  double t7951;
  double t7952;
  double t7954;
  double t7955;
  double t7956;
  double t7957;
  double t7958;
  double t7966;
  double t7970;
  double t7971;
  double t7972;
  double t7974;
  double t7976;
  double t7977;
  double t7978;
  double t7979;
  double t7994;
  double t7996;
  double t7992;
  double t7997;
  double t7998;
  double t8001;
  double t8002;
  double t8003;
  double t7999;
  double t8004;
  double t8005;
  double t8009;
  double t8010;
  double t8012;
  double t7925;
  double t7926;
  double t7928;
  double t7929;
  double t7930;
  double t7931;
  double t7932;
  double t7933;
  double t7934;
  double t7960;
  double t7983;
  double t7984;
  double t7987;
  double t7988;
  double t7990;
  double t8035;
  double t8037;
  double t8034;
  double t8038;
  double t8041;
  double t8044;
  double t8045;
  double t8048;
  double t8042;
  double t8049;
  double t8051;
  double t8054;
  double t8056;
  double t8057;
  double t7891;
  double t7907;
  double t7914;
  double t7915;
  double t7917;
  double t7918;
  double t7919;
  double t7920;
  double t7921;
  double t8098;
  double t8099;
  double t8100;
  double t8101;
  double t8102;
  double t8104;
  double t8105;
  double t8106;
  double t8108;
  double t8110;
  double t8111;
  double t8113;
  double t8114;
  double t8115;
  double t8117;
  double t8118;
  double t8120;
  double t8121;
  double t8123;
  double t8124;
  double t8127;
  double t8131;
  double t8133;
  double t8134;
  double t8136;
  double t8137;
  double t8139;
  double t8141;
  double t8156;
  double t8159;
  double t8160;
  double t8163;
  double t8164;
  double t8165;
  double t8168;
  double t8169;
  double t8170;
  double t8075;
  double t8076;
  double t8077;
  double t8078;
  double t8079;
  double t8082;
  double t8084;
  double t8086;
  double t8087;
  double t8088;
  double t8089;
  double t8090;
  double t8092;
  double t8116;
  double t8142;
  double t8143;
  double t8150;
  double t8152;
  double t8154;
  double t3942;
  double t8194;
  double t8195;
  double t8197;
  double t8200;
  double t8203;
  double t8205;
  double t8207;
  double t8208;
  double t8240;
  double t8241;
  double t8242;
  double t8246;
  double t8247;
  double t8248;
  double t8251;
  double t8254;
  double t8258;
  double t8277;
  double t8279;
  double t8281;
  double t8275;
  double t8286;
  double t8288;
  double t8291;
  double t8295;
  double t8296;
  double t8297;
  double t8322;
  double t8323;
  double t8332;
  double t8336;
  double t8337;
  double t8338;
  double t8340;
  double t8341;
  double t8342;
  double t8276;
  double t8282;
  double t8284;
  double t8292;
  double t8298;
  double t8300;
  double t8301;
  double t8302;
  double t8303;
  double t8307;
  double t8308;
  double t8309;
  double t8313;
  double t8317;
  double t8318;
  double t8326;
  double t8329;
  double t8335;
  double t8339;
  double t8344;
  double t8345;
  double t8347;
  double t8348;
  double t8350;
  double t8351;
  double t8353;
  double t8354;
  double t8355;
  double t8356;
  double t8357;
  double t8370;
  double t8372;
  double t8374;
  double t8378;
  double t8380;
  double t8386;
  double t8387;
  double t8388;
  double t8321;
  double t8358;
  double t8360;
  double t8364;
  double t8365;
  double t8366;
  double t8405;
  double t8408;
  double t8409;
  double t8411;
  double t8412;
  double t8414;
  double t8418;
  double t8419;
  double t8433;
  double t8434;
  double t8436;
  double t8437;
  double t8438;
  double t8440;
  double t8442;
  double t8445;
  double t8459;
  double t8460;
  double t8462;
  double t8457;
  double t8467;
  double t8468;
  double t8470;
  double t8458;
  double t8464;
  double t8465;
  double t8478;
  double t8479;
  double t8484;
  double t8486;
  double t8488;
  double t8489;
  double t8507;
  double t8509;
  double t8510;
  double t8515;
  double t8516;
  double t8517;
  double t8520;
  double t8521;
  double t8522;
  double t8525;
  double t8526;
  double t8527;
  double t8466;
  double t8473;
  double t8477;
  double t8485;
  double t8490;
  double t8491;
  double t8492;
  double t8493;
  double t8494;
  double t8498;
  double t8499;
  double t8500;
  double t8501;
  double t8504;
  double t8505;
  double t8511;
  double t8514;
  double t8519;
  double t8524;
  double t8528;
  double t8529;
  double t8532;
  double t8533;
  double t8534;
  double t8537;
  double t8538;
  double t8539;
  double t8540;
  double t8541;
  double t8542;
  double t8551;
  double t8552;
  double t8506;
  double t8543;
  double t8544;
  double t8546;
  double t8547;
  double t8548;
  double t8571;
  double t8572;
  double t8585;
  double t8586;
  double t8545;
  double t8549;
  double t8550;
  double t8564;
  double t8569;
  double t8570;
  double t8582;
  double t8583;
  double t8584;
  t679 = Cos(var1[5]);
  t1896 = Sin(var1[3]);
  t2176 = Sin(var1[4]);
  t344 = Cos(var1[3]);
  t2364 = Sin(var1[5]);
  t170 = Cos(var1[6]);
  t8 = Cos(var1[7]);
  t4203 = -1.*t8;
  t4246 = 0. + t4203;
  t3108 = Sin(var1[6]);
  t3921 = Sin(var1[7]);
  t4623 = 0. + t3921;
  t3763 = Cos(var1[4]);
  t4256 = t170*t4246;
  t4295 = 0. + t4256;
  t4327 = t4246*t3108;
  t4478 = 0. + t4327;
  t1331 = -1.*t344*t679;
  t2708 = -1.*t1896*t2176*t2364;
  t2734 = t1331 + t2708;
  t2843 = t170*t2734;
  t2923 = -1.*t679*t1896*t2176;
  t2971 = t344*t2364;
  t3073 = t2923 + t2971;
  t3213 = t3073*t3108;
  t3249 = t2843 + t3213;
  t6103 = Cos(var1[8]);
  t6366 = t170*t3073;
  t6420 = -1.*t2734*t3108;
  t6586 = t6366 + t6420;
  t6122 = -1.*t3763*t8*t1896;
  t6193 = t3249*t3921;
  t6253 = t6122 + t6193;
  t6661 = Sin(var1[8]);
  t6784 = Cos(var1[9]);
  t6257 = t6103*t6253;
  t6708 = t6586*t6661;
  t6735 = t6257 + t6708;
  t5781 = Sin(var1[9]);
  t6801 = t6103*t6586;
  t6830 = -1.*t6253*t6661;
  t6852 = t6801 + t6830;
  t6919 = Cos(var1[10]);
  t5697 = Sin(var1[10]);
  t7019 = t6784*t6919;
  t7028 = -1.*t5781*t5697;
  t7040 = 0. + t7019 + t7028;
  t7002 = t6919*t5781;
  t7013 = t6784*t5697;
  t7015 = 0. + t7002 + t7013;
  t7077 = t6103*t7040;
  t7094 = -1.*t7015*t6661;
  t7102 = 0. + t7077 + t7094;
  t7126 = t8*t7102;
  t7143 = 0. + t7126;
  t7018 = t6103*t7015;
  t7066 = t7040*t6661;
  t7070 = 0. + t7018 + t7066;
  t7154 = t3921*t7102;
  t7171 = 0. + t7154;
  t7153 = -1.*t3108*t7070;
  t7173 = t170*t7171;
  t7194 = 0. + t7153 + t7173;
  t7204 = t170*t7070;
  t7208 = t3108*t7171;
  t7211 = 0. + t7204 + t7208;
  t7298 = -1.*t6784*t6919;
  t7299 = t5781*t5697;
  t7300 = 0. + t7298 + t7299;
  t7323 = -1.*t7300*t6661;
  t7329 = 0. + t7018 + t7323;
  t7330 = t8*t7329;
  t7334 = 0. + t7330;
  t7303 = t6103*t7300;
  t7304 = t7015*t6661;
  t7310 = 0. + t7303 + t7304;
  t7349 = t3921*t7329;
  t7354 = 0. + t7349;
  t7346 = -1.*t3108*t7310;
  t7358 = t170*t7354;
  t7361 = 0. + t7346 + t7358;
  t7369 = t170*t7310;
  t7371 = t3108*t7354;
  t7373 = 0. + t7369 + t7371;
  t6741 = -1.*t5781*t6735;
  t6877 = t6784*t6852;
  t6895 = t6741 + t6877;
  t6946 = t6784*t6735;
  t6947 = t5781*t6852;
  t6987 = t6946 + t6947;
  t7072 = var2[7]*t7070;
  t7145 = var2[5]*t7143;
  t7148 = var2[6]*t7143;
  t7203 = t679*t7194;
  t7221 = -1.*t2364*t7211;
  t7226 = 0. + t7203 + t7221;
  t7229 = var2[4]*t7226;
  t7232 = -1.*t2176*t7143;
  t7244 = t2364*t7194;
  t7248 = t679*t7211;
  t7259 = 0. + t7244 + t7248;
  t7276 = t3763*t7259;
  t7278 = 0. + t7232 + t7276;
  t7285 = var2[3]*t7278;
  t7286 = 0. + t7072 + t7145 + t7148 + t7229 + t7285;
  t7315 = var2[7]*t7310;
  t7338 = var2[5]*t7334;
  t7341 = var2[6]*t7334;
  t7366 = t679*t7361;
  t7375 = -1.*t2364*t7373;
  t7376 = 0. + t7366 + t7375;
  t7379 = var2[4]*t7376;
  t7381 = -1.*t2176*t7334;
  t7387 = t2364*t7361;
  t7388 = t679*t7373;
  t7392 = 0. + t7387 + t7388;
  t7393 = t3763*t7392;
  t7398 = 0. + t7381 + t7393;
  t7402 = var2[3]*t7398;
  t7403 = 0. + t7315 + t7338 + t7341 + t7379 + t7402;
  t4301 = t679*t4295;
  t4491 = -1.*t2364*t4478;
  t4610 = 0. + t4301 + t4491;
  t4622 = var2[4]*t4610;
  t4677 = var2[5]*t4623;
  t4798 = var2[6]*t4623;
  t4862 = t4295*t2364;
  t4888 = t679*t4478;
  t5061 = 0. + t4862 + t4888;
  t5165 = t3763*t5061;
  t5240 = -1.*t2176*t4623;
  t5287 = 0. + t5165 + t5240;
  t5325 = var2[3]*t5287;
  t5350 = 0. + var2[9] + var2[10] + var2[8] + t4622 + t4677 + t4798 + t5325;
  t5384 = 0.0341*t5350;
  t5536 = 0. + t5384;
  t7437 = -1.*t679*t1896;
  t7438 = t344*t2176*t2364;
  t7440 = t7437 + t7438;
  t7441 = t170*t7440;
  t7442 = t344*t679*t2176;
  t7443 = t1896*t2364;
  t7449 = t7442 + t7443;
  t7450 = t7449*t3108;
  t7451 = t7441 + t7450;
  t7472 = t170*t7449;
  t7473 = -1.*t7440*t3108;
  t7475 = t7472 + t7473;
  t7463 = t344*t3763*t8;
  t7465 = t7451*t3921;
  t7466 = t7463 + t7465;
  t7469 = t6103*t7466;
  t7476 = t7475*t6661;
  t7480 = t7469 + t7476;
  t7484 = t6103*t7475;
  t7488 = -1.*t7466*t6661;
  t7490 = t7484 + t7488;
  t7291 = 0.0341*t7286;
  t7405 = 0.000334*t7403;
  t7407 = 0. + t7291 + t7405;
  t7482 = -1.*t5781*t7480;
  t7491 = t6784*t7490;
  t7493 = t7482 + t7491;
  t7495 = t6784*t7480;
  t7496 = t5781*t7490;
  t7497 = t7495 + t7496;
  t7424 = 0.000334*t7286;
  t7427 = 0.00036*t7403;
  t7431 = 0. + t7424 + t7427;
  t7452 = -1.*t8*t7451;
  t7453 = t344*t3763*t3921;
  t7455 = t7452 + t7453;
  t7494 = t5697*t7493;
  t7498 = t6919*t7497;
  t7500 = t7494 + t7498;
  t7502 = -1.*t6919*t7493;
  t7504 = t5697*t7497;
  t7505 = t7502 + t7504;
  t7521 = -1.*t3763*t7143;
  t7522 = -1.*t2176*t7259;
  t7523 = t7521 + t7522;
  t7527 = -1.*t3763*t7334;
  t7529 = -1.*t2176*t7392;
  t7530 = t7527 + t7529;
  t7514 = t344*t3763*t170*t2364;
  t7515 = t344*t3763*t679*t3108;
  t7516 = t7514 + t7515;
  t7545 = t344*t3763*t679*t170;
  t7546 = -1.*t344*t3763*t2364*t3108;
  t7547 = t7545 + t7546;
  t7541 = -1.*t344*t8*t2176;
  t7542 = t7516*t3921;
  t7543 = t7541 + t7542;
  t7544 = t6103*t7543;
  t7552 = t7547*t6661;
  t7553 = t7544 + t7552;
  t7555 = t6103*t7547;
  t7556 = -1.*t7543*t6661;
  t7558 = t7555 + t7556;
  t7554 = -1.*t5781*t7553;
  t7559 = t6784*t7558;
  t7560 = t7554 + t7559;
  t7562 = t6784*t7553;
  t7563 = t5781*t7558;
  t7564 = t7562 + t7563;
  t7508 = -1.*t2176*t5061;
  t7510 = -1.*t3763*t4623;
  t7512 = t7508 + t7510;
  t7575 = t344*t679;
  t7576 = t1896*t2176*t2364;
  t7577 = t7575 + t7576;
  t7578 = t170*t7577;
  t7579 = t679*t1896*t2176;
  t7580 = -1.*t344*t2364;
  t7581 = t7579 + t7580;
  t7582 = t7581*t3108;
  t7583 = t7578 + t7582;
  t7603 = t170*t7581;
  t7604 = -1.*t7577*t3108;
  t7605 = t7603 + t7604;
  t7598 = t3763*t8*t1896;
  t7599 = t7583*t3921;
  t7601 = t7598 + t7599;
  t7602 = t6103*t7601;
  t7606 = t7605*t6661;
  t7607 = t7602 + t7606;
  t7609 = t6103*t7605;
  t7611 = -1.*t7601*t6661;
  t7615 = t7609 + t7611;
  t7524 = 0.0341*var2[3]*t7523;
  t7531 = 0.000334*var2[3]*t7530;
  t7534 = t7524 + t7531;
  t7608 = -1.*t5781*t7607;
  t7616 = t6784*t7615;
  t7617 = t7608 + t7616;
  t7619 = t6784*t7607;
  t7620 = t5781*t7615;
  t7621 = t7619 + t7620;
  t7536 = 0.000334*var2[3]*t7523;
  t7537 = 0.00036*var2[3]*t7530;
  t7539 = t7536 + t7537;
  t7589 = t3763*t170*t1896*t2364;
  t7590 = t3763*t679*t1896*t3108;
  t7593 = t7589 + t7590;
  t7634 = t3763*t679*t170*t1896;
  t7635 = -1.*t3763*t1896*t2364*t3108;
  t7637 = t7634 + t7635;
  t7629 = -1.*t8*t1896*t2176;
  t7630 = t7593*t3921;
  t7631 = t7629 + t7630;
  t7632 = t6103*t7631;
  t7638 = t7637*t6661;
  t7639 = t7632 + t7638;
  t7642 = t6103*t7637;
  t7643 = -1.*t7631*t6661;
  t7645 = t7642 + t7643;
  t7640 = -1.*t5781*t7639;
  t7646 = t6784*t7645;
  t7647 = t7640 + t7646;
  t7649 = t6784*t7639;
  t7650 = t5781*t7645;
  t7651 = t7649 + t7650;
  t7661 = t3763*t170*t2364;
  t7663 = t3763*t679*t3108;
  t7664 = t7661 + t7663;
  t7685 = t3763*t679*t170;
  t7686 = -1.*t3763*t2364*t3108;
  t7688 = t7685 + t7686;
  t7679 = -1.*t8*t2176;
  t7681 = t7664*t3921;
  t7682 = t7679 + t7681;
  t7683 = t6103*t7682;
  t7689 = t7688*t6661;
  t7690 = t7683 + t7689;
  t7694 = t6103*t7688;
  t7696 = -1.*t7682*t6661;
  t7697 = t7694 + t7696;
  t7691 = -1.*t5781*t7690;
  t7703 = t6784*t7697;
  t7706 = t7691 + t7703;
  t7709 = t6784*t7690;
  t7710 = t5781*t7697;
  t7712 = t7709 + t7710;
  t7670 = -1.*t170*t2176*t2364;
  t7671 = -1.*t679*t2176*t3108;
  t7672 = t7670 + t7671;
  t7729 = -1.*t679*t170*t2176;
  t7731 = t2176*t2364*t3108;
  t7732 = t7729 + t7731;
  t7725 = -1.*t3763*t8;
  t7726 = t7672*t3921;
  t7727 = t7725 + t7726;
  t7728 = t6103*t7727;
  t7733 = t7732*t6661;
  t7734 = t7728 + t7733;
  t7737 = t6103*t7732;
  t7742 = -1.*t7727*t6661;
  t7744 = t7737 + t7742;
  t7736 = -1.*t5781*t7734;
  t7745 = t6784*t7744;
  t7746 = t7736 + t7745;
  t7748 = t6784*t7734;
  t7749 = t5781*t7744;
  t7750 = t7748 + t7749;
  t7787 = -1.*t2364*t7194;
  t7788 = -1.*t679*t7211;
  t7789 = t7787 + t7788;
  t7790 = var2[4]*t7789;
  t7791 = t7203 + t7221;
  t7792 = var2[3]*t3763*t7791;
  t7793 = t7790 + t7792;
  t7795 = -1.*t2364*t7361;
  t7796 = -1.*t679*t7373;
  t7797 = t7795 + t7796;
  t7798 = var2[4]*t7797;
  t7799 = t7366 + t7375;
  t7800 = var2[3]*t3763*t7799;
  t7801 = t7798 + t7800;
  t7777 = t679*t1896;
  t7778 = -1.*t344*t2176*t2364;
  t7780 = t7777 + t7778;
  t7781 = t7780*t3108;
  t7783 = t7472 + t7781;
  t7812 = t170*t7780;
  t7813 = -1.*t7449*t3108;
  t7814 = t7812 + t7813;
  t7811 = t6103*t7783*t3921;
  t7815 = t7814*t6661;
  t7816 = t7811 + t7815;
  t7818 = t6103*t7814;
  t7821 = -1.*t7783*t3921*t6661;
  t7823 = t7818 + t7821;
  t7817 = -1.*t5781*t7816;
  t7824 = t6784*t7823;
  t7826 = t7817 + t7824;
  t7830 = t6784*t7816;
  t7832 = t5781*t7823;
  t7834 = t7830 + t7832;
  t7763 = -1.*t4295*t2364;
  t7765 = -1.*t679*t4478;
  t7766 = t7763 + t7765;
  t7767 = var2[4]*t7766;
  t7770 = t4301 + t4491;
  t7773 = var2[3]*t3763*t7770;
  t7774 = t7767 + t7773;
  t7584 = -1.*t8*t7583;
  t7585 = t3763*t1896*t3921;
  t7586 = t7584 + t7585;
  t7618 = t5697*t7617;
  t7622 = t6919*t7621;
  t7623 = t7618 + t7622;
  t7794 = 0.0341*t7793;
  t7803 = 0.000334*t7801;
  t7805 = t7794 + t7803;
  t7625 = -1.*t6919*t7617;
  t7626 = t5697*t7621;
  t7627 = t7625 + t7626;
  t7807 = 0.000334*t7793;
  t7808 = 0.00036*t7801;
  t7809 = t7807 + t7808;
  t7849 = t2734*t3108;
  t7850 = t7603 + t7849;
  t7856 = -1.*t7581*t3108;
  t7857 = t2843 + t7856;
  t7854 = t6103*t7850*t3921;
  t7858 = t7857*t6661;
  t7860 = t7854 + t7858;
  t7862 = t6103*t7857;
  t7863 = -1.*t7850*t3921*t6661;
  t7864 = t7862 + t7863;
  t7861 = -1.*t5781*t7860;
  t7865 = t6784*t7864;
  t7870 = t7861 + t7865;
  t7874 = t6784*t7860;
  t7875 = t5781*t7864;
  t7877 = t7874 + t7875;
  t7665 = -1.*t8*t7664;
  t7667 = -1.*t2176*t3921;
  t7668 = t7665 + t7667;
  t7707 = t5697*t7706;
  t7715 = t6919*t7712;
  t7716 = t7707 + t7715;
  t7718 = -1.*t6919*t7706;
  t7719 = t5697*t7712;
  t7720 = t7718 + t7719;
  t7896 = -1.*t3763*t170*t2364;
  t7897 = -1.*t3763*t679*t3108;
  t7898 = t7896 + t7897;
  t7894 = t6103*t7688*t3921;
  t7899 = t7898*t6661;
  t7900 = t7894 + t7899;
  t7902 = t6103*t7898;
  t7903 = -1.*t7688*t3921*t6661;
  t7904 = t7902 + t7903;
  t7901 = -1.*t5781*t7900;
  t7905 = t6784*t7904;
  t7906 = t7901 + t7905;
  t7908 = t6784*t7900;
  t7912 = t5781*t7904;
  t7913 = t7908 + t7912;
  t7939 = t7153 + t7173;
  t7943 = -1.*t170*t7070;
  t7945 = -1.*t3108*t7171;
  t7948 = t7943 + t7945;
  t7965 = t7346 + t7358;
  t7967 = -1.*t170*t7310;
  t7968 = -1.*t3108*t7354;
  t7969 = t7967 + t7968;
  t7942 = -1.*t2364*t7939;
  t7949 = t679*t7948;
  t7951 = t7942 + t7949;
  t7952 = var2[4]*t7951;
  t7954 = t679*t7939;
  t7955 = t2364*t7948;
  t7956 = t7954 + t7955;
  t7957 = var2[3]*t3763*t7956;
  t7958 = t7952 + t7957;
  t7966 = -1.*t2364*t7965;
  t7970 = t679*t7969;
  t7971 = t7966 + t7970;
  t7972 = var2[4]*t7971;
  t7974 = t679*t7965;
  t7976 = t2364*t7969;
  t7977 = t7974 + t7976;
  t7978 = var2[3]*t3763*t7977;
  t7979 = t7972 + t7978;
  t7994 = -1.*t170*t7440;
  t7996 = t7994 + t7813;
  t7992 = t6103*t7475*t3921;
  t7997 = t7996*t6661;
  t7998 = t7992 + t7997;
  t8001 = t6103*t7996;
  t8002 = -1.*t7475*t3921*t6661;
  t8003 = t8001 + t8002;
  t7999 = -1.*t5781*t7998;
  t8004 = t6784*t8003;
  t8005 = t7999 + t8004;
  t8009 = t6784*t7998;
  t8010 = t5781*t8003;
  t8012 = t8009 + t8010;
  t7925 = -1.*t170*t4246*t2364;
  t7926 = -1.*t679*t4246*t3108;
  t7928 = t7925 + t7926;
  t7929 = var2[4]*t7928;
  t7930 = t679*t170*t4246;
  t7931 = -1.*t4246*t2364*t3108;
  t7932 = t7930 + t7931;
  t7933 = var2[3]*t3763*t7932;
  t7934 = t7929 + t7933;
  t7960 = 0.0341*t7958;
  t7983 = 0.000334*t7979;
  t7984 = t7960 + t7983;
  t7987 = 0.000334*t7958;
  t7988 = 0.00036*t7979;
  t7990 = t7987 + t7988;
  t8035 = -1.*t170*t7577;
  t8037 = t8035 + t7856;
  t8034 = t6103*t7605*t3921;
  t8038 = t8037*t6661;
  t8041 = t8034 + t8038;
  t8044 = t6103*t8037;
  t8045 = -1.*t7605*t3921*t6661;
  t8048 = t8044 + t8045;
  t8042 = -1.*t5781*t8041;
  t8049 = t6784*t8048;
  t8051 = t8042 + t8049;
  t8054 = t6784*t8041;
  t8056 = t5781*t8048;
  t8057 = t8054 + t8056;
  t7891 = -1.*t8*t7688*t5536;
  t7907 = t5697*t7906;
  t7914 = t6919*t7913;
  t7915 = t7907 + t7914;
  t7917 = t7915*t7407;
  t7918 = -1.*t6919*t7906;
  t7919 = t5697*t7913;
  t7920 = t7918 + t7919;
  t7921 = t7920*t7431;
  t8098 = -1.*var2[5]*t3921*t7102;
  t8099 = -1.*var2[6]*t3921*t7102;
  t8100 = t679*t170*t8*t7102;
  t8101 = -1.*t8*t2364*t3108*t7102;
  t8102 = t8100 + t8101;
  t8104 = var2[4]*t8102;
  t8105 = t2176*t3921*t7102;
  t8106 = t170*t8*t2364*t7102;
  t8108 = t679*t8*t3108*t7102;
  t8110 = t8106 + t8108;
  t8111 = t3763*t8110;
  t8113 = t8105 + t8111;
  t8114 = var2[3]*t8113;
  t8115 = t8098 + t8099 + t8104 + t8114;
  t8117 = -1.*var2[5]*t3921*t7329;
  t8118 = -1.*var2[6]*t3921*t7329;
  t8120 = t679*t170*t8*t7329;
  t8121 = -1.*t8*t2364*t3108*t7329;
  t8123 = t8120 + t8121;
  t8124 = var2[4]*t8123;
  t8127 = t2176*t3921*t7329;
  t8131 = t170*t8*t2364*t7329;
  t8133 = t679*t8*t3108*t7329;
  t8134 = t8131 + t8133;
  t8136 = t3763*t8134;
  t8137 = t8127 + t8136;
  t8139 = var2[3]*t8137;
  t8141 = t8117 + t8118 + t8124 + t8139;
  t8156 = t8*t7451;
  t8159 = -1.*t344*t3763*t3921;
  t8160 = t8156 + t8159;
  t8163 = -1.*t6103*t5781*t8160;
  t8164 = -1.*t6784*t8160*t6661;
  t8165 = t8163 + t8164;
  t8168 = t6784*t6103*t8160;
  t8169 = -1.*t5781*t8160*t6661;
  t8170 = t8168 + t8169;
  t8075 = var2[5]*t8;
  t8076 = var2[6]*t8;
  t8077 = t679*t170*t3921;
  t8078 = -1.*t2364*t3108*t3921;
  t8079 = t8077 + t8078;
  t8082 = var2[4]*t8079;
  t8084 = t170*t2364*t3921;
  t8086 = t679*t3108*t3921;
  t8087 = t8084 + t8086;
  t8088 = t3763*t8087;
  t8089 = t7679 + t8088;
  t8090 = var2[3]*t8089;
  t8092 = t8075 + t8076 + t8082 + t8090;
  t8116 = 0.0341*t8115;
  t8142 = 0.000334*t8141;
  t8143 = t8116 + t8142;
  t8150 = 0.000334*t8115;
  t8152 = 0.00036*t8141;
  t8154 = t8150 + t8152;
  t3942 = -1.*t3763*t1896*t3921;
  t8194 = t8*t7583;
  t8195 = t8194 + t3942;
  t8197 = -1.*t6103*t5781*t8195;
  t8200 = -1.*t6784*t8195*t6661;
  t8203 = t8197 + t8200;
  t8205 = t6784*t6103*t8195;
  t8207 = -1.*t5781*t8195*t6661;
  t8208 = t8205 + t8207;
  t8240 = t8*t7664;
  t8241 = t2176*t3921;
  t8242 = t8240 + t8241;
  t8246 = -1.*t6103*t5781*t8242;
  t8247 = -1.*t6784*t8242*t6661;
  t8248 = t8246 + t8247;
  t8251 = t6784*t6103*t8242;
  t8254 = -1.*t5781*t8242*t6661;
  t8258 = t8251 + t8254;
  t8277 = -1.*t6103*t7015;
  t8279 = -1.*t7040*t6661;
  t8281 = t8277 + t8279;
  t8275 = t7077 + t7094;
  t8286 = -1.*t3108*t8275;
  t8288 = t170*t3921*t8281;
  t8291 = t8286 + t8288;
  t8295 = t170*t8275;
  t8296 = t3108*t3921*t8281;
  t8297 = t8295 + t8296;
  t8322 = -1.*t6103*t7300;
  t8323 = t8322 + t7094;
  t8332 = t7018 + t7323;
  t8336 = t3108*t3921*t8323;
  t8337 = t170*t8332;
  t8338 = t8336 + t8337;
  t8340 = t170*t3921*t8323;
  t8341 = -1.*t3108*t8332;
  t8342 = t8340 + t8341;
  t8276 = var2[7]*t8275;
  t8282 = var2[5]*t8*t8281;
  t8284 = var2[6]*t8*t8281;
  t8292 = t679*t8291;
  t8298 = -1.*t2364*t8297;
  t8300 = t8292 + t8298;
  t8301 = var2[4]*t8300;
  t8302 = -1.*t8*t2176*t8281;
  t8303 = t2364*t8291;
  t8307 = t679*t8297;
  t8308 = t8303 + t8307;
  t8309 = t3763*t8308;
  t8313 = t8302 + t8309;
  t8317 = var2[3]*t8313;
  t8318 = t8276 + t8282 + t8284 + t8301 + t8317;
  t8326 = var2[5]*t8*t8323;
  t8329 = var2[6]*t8*t8323;
  t8335 = var2[7]*t8332;
  t8339 = -1.*t2364*t8338;
  t8344 = t679*t8342;
  t8345 = t8339 + t8344;
  t8347 = var2[4]*t8345;
  t8348 = -1.*t8*t2176*t8323;
  t8350 = t679*t8338;
  t8351 = t2364*t8342;
  t8353 = t8350 + t8351;
  t8354 = t3763*t8353;
  t8355 = t8348 + t8354;
  t8356 = var2[3]*t8355;
  t8357 = t8326 + t8329 + t8335 + t8347 + t8356;
  t8370 = -1.*t6103*t7466;
  t8372 = -1.*t7475*t6661;
  t8374 = t8370 + t8372;
  t8378 = t5781*t8374;
  t8380 = t8378 + t7491;
  t8386 = t6784*t8374;
  t8387 = -1.*t5781*t7490;
  t8388 = t8386 + t8387;
  t8321 = 0.0341*t8318;
  t8358 = 0.000334*t8357;
  t8360 = t8321 + t8358;
  t8364 = 0.000334*t8318;
  t8365 = 0.00036*t8357;
  t8366 = t8364 + t8365;
  t8405 = -1.*t6103*t7601;
  t8408 = -1.*t7605*t6661;
  t8409 = t8405 + t8408;
  t8411 = t5781*t8409;
  t8412 = t8411 + t7616;
  t8414 = t6784*t8409;
  t8418 = -1.*t5781*t7615;
  t8419 = t8414 + t8418;
  t8433 = -1.*t6103*t7682;
  t8434 = -1.*t7688*t6661;
  t8436 = t8433 + t8434;
  t8437 = t5781*t8436;
  t8438 = t8437 + t7703;
  t8440 = t6784*t8436;
  t8442 = -1.*t5781*t7697;
  t8445 = t8440 + t8442;
  t8459 = -1.*t6919*t5781;
  t8460 = -1.*t6784*t5697;
  t8462 = t8459 + t8460;
  t8457 = t7019 + t7028;
  t8467 = t6103*t8462;
  t8468 = -1.*t8457*t6661;
  t8470 = t8467 + t8468;
  t8458 = t6103*t8457;
  t8464 = t8462*t6661;
  t8465 = t8458 + t8464;
  t8478 = -1.*t3108*t8465;
  t8479 = t170*t3921*t8470;
  t8484 = t8478 + t8479;
  t8486 = t170*t8465;
  t8488 = t3108*t3921*t8470;
  t8489 = t8486 + t8488;
  t8507 = t7002 + t7013;
  t8509 = -1.*t8507*t6661;
  t8510 = t8458 + t8509;
  t8515 = t6103*t8507;
  t8516 = t8457*t6661;
  t8517 = t8515 + t8516;
  t8520 = t3108*t3921*t8510;
  t8521 = t170*t8517;
  t8522 = t8520 + t8521;
  t8525 = t170*t3921*t8510;
  t8526 = -1.*t3108*t8517;
  t8527 = t8525 + t8526;
  t8466 = var2[7]*t8465;
  t8473 = var2[5]*t8*t8470;
  t8477 = var2[6]*t8*t8470;
  t8485 = t679*t8484;
  t8490 = -1.*t2364*t8489;
  t8491 = t8485 + t8490;
  t8492 = var2[4]*t8491;
  t8493 = -1.*t8*t2176*t8470;
  t8494 = t2364*t8484;
  t8498 = t679*t8489;
  t8499 = t8494 + t8498;
  t8500 = t3763*t8499;
  t8501 = t8493 + t8500;
  t8504 = var2[3]*t8501;
  t8505 = t8466 + t8473 + t8477 + t8492 + t8504;
  t8511 = var2[5]*t8*t8510;
  t8514 = var2[6]*t8*t8510;
  t8519 = var2[7]*t8517;
  t8524 = -1.*t2364*t8522;
  t8528 = t679*t8527;
  t8529 = t8524 + t8528;
  t8532 = var2[4]*t8529;
  t8533 = -1.*t8*t2176*t8510;
  t8534 = t679*t8522;
  t8537 = t2364*t8527;
  t8538 = t8534 + t8537;
  t8539 = t3763*t8538;
  t8540 = t8533 + t8539;
  t8541 = var2[3]*t8540;
  t8542 = t8511 + t8514 + t8519 + t8532 + t8541;
  t8551 = -1.*t6784*t7480;
  t8552 = t8551 + t8387;
  t8506 = 0.0341*t8505;
  t8543 = 0.000334*t8542;
  t8544 = t8506 + t8543;
  t8546 = 0.000334*t8505;
  t8547 = 0.00036*t8542;
  t8548 = t8546 + t8547;
  t8571 = -1.*t6784*t7607;
  t8572 = t8571 + t8418;
  t8585 = -1.*t6784*t7690;
  t8586 = t8585 + t8442;
  t8545 = t7500*t8544;
  t8549 = t7505*t8548;
  t8550 = t6919*t7493;
  t8564 = t7623*t8544;
  t8569 = t7627*t8548;
  t8570 = t6919*t7617;
  t8582 = t7716*t8544;
  t8583 = t7720*t8548;
  t8584 = t6919*t7706;
  p_output1[0]=0;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=0;
  p_output1[9]=(t5697*t6895 + t6919*t6987)*t7407 + (-1.*t6895*t6919 + t5697*t6987)*t7431 + t5536*(t3942 - 1.*t3249*t8);
  p_output1[10]=t5536*t7455 + t7407*t7500 + t7431*t7505;
  p_output1[11]=0;
  p_output1[12]=t7500*t7534 + t7505*t7539 + t7431*(-1.*t6919*t7560 + t5697*t7564) + t7407*(t5697*t7560 + t6919*t7564) + t5536*(-1.*t2176*t344*t3921 - 1.*t7516*t8) + 0.0341*t7455*t7512*var2[3];
  p_output1[13]=t7534*t7623 + t7539*t7627 + t7431*(-1.*t6919*t7647 + t5697*t7651) + t7407*(t5697*t7647 + t6919*t7651) + t5536*(-1.*t1896*t2176*t3921 - 1.*t7593*t8) + 0.0341*t7512*t7586*var2[3];
  p_output1[14]=t7534*t7716 + t7539*t7720 + t7431*(-1.*t6919*t7746 + t5697*t7750) + t7407*(t5697*t7746 + t6919*t7750) + t5536*(-1.*t3763*t3921 - 1.*t7672*t8) + 0.0341*t7512*t7668*var2[3];
  p_output1[15]=0.0341*t7455*t7774 + t7500*t7805 + t7505*t7809 + t7431*(-1.*t6919*t7826 + t5697*t7834) + t7407*(t5697*t7826 + t6919*t7834) - 1.*t5536*t7783*t8;
  p_output1[16]=0.0341*t7586*t7774 + t7623*t7805 + t7627*t7809 + t7431*(-1.*t6919*t7870 + t5697*t7877) + t7407*(t5697*t7870 + t6919*t7877) - 1.*t5536*t7850*t8;
  p_output1[17]=0.0341*t7668*t7774 + t7716*t7805 + t7720*t7809 + t7891 + t7917 + t7921;
  p_output1[18]=0.0341*t7455*t7934 + t7500*t7984 + t7505*t7990 - 1.*t5536*t7475*t8 + t7431*(-1.*t6919*t8005 + t5697*t8012) + t7407*(t5697*t8005 + t6919*t8012);
  p_output1[19]=0.0341*t7586*t7934 + t7623*t7984 + t7627*t7990 - 1.*t5536*t7605*t8 + t7431*(-1.*t6919*t8051 + t5697*t8057) + t7407*(t5697*t8051 + t6919*t8057);
  p_output1[20]=t7891 + t7917 + t7921 + 0.0341*t7668*t7934 + t7716*t7984 + t7720*t7990;
  p_output1[21]=t5536*t7466 + 0.0341*t7455*t8092 + t7500*t8143 + t7505*t8154 + t7431*(-1.*t6919*t8165 + t5697*t8170) + t7407*(t5697*t8165 + t6919*t8170);
  p_output1[22]=t5536*t7601 + 0.0341*t7586*t8092 + t7623*t8143 + t7627*t8154 + t7431*(-1.*t6919*t8203 + t5697*t8208) + t7407*(t5697*t8203 + t6919*t8208);
  p_output1[23]=t5536*t7682 + 0.0341*t7668*t8092 + t7716*t8143 + t7720*t8154 + t7431*(-1.*t6919*t8248 + t5697*t8258) + t7407*(t5697*t8248 + t6919*t8258);
  p_output1[24]=t7500*t8360 + t7505*t8366 + t7407*(t6919*t8380 + t5697*t8388) + t7431*(t5697*t8380 - 1.*t6919*t8388);
  p_output1[25]=t7623*t8360 + t7627*t8366 + t7407*(t6919*t8412 + t5697*t8419) + t7431*(t5697*t8412 - 1.*t6919*t8419);
  p_output1[26]=t7716*t8360 + t7720*t8366 + t7407*(t6919*t8438 + t5697*t8445) + t7431*(t5697*t8438 - 1.*t6919*t8445);
  p_output1[27]=t8545 + t8549 + t7407*(t8550 + t5697*t8552) + t7431*(t7494 - 1.*t6919*t8552);
  p_output1[28]=t8564 + t8569 + t7407*(t8570 + t5697*t8572) + t7431*(t7618 - 1.*t6919*t8572);
  p_output1[29]=t8582 + t8583 + t7407*(t8584 + t5697*t8586) + t7431*(t7707 - 1.*t6919*t8586);
  p_output1[30]=t7431*t7500 + t8545 + t8549 + t7407*(-1.*t5697*t7497 + t8550);
  p_output1[31]=t7431*t7623 + t8564 + t8569 + t7407*(-1.*t5697*t7621 + t8570);
  p_output1[32]=t7431*t7716 + t8582 + t8583 + t7407*(-1.*t5697*t7712 + t8584);
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
  plhs[0] = mxCreateDoubleMatrix((mwSize) 3, (mwSize) 20, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,var1,var2);


}

#else // MATLAB_MEX_FILE

#include "Jq_AMWorld_LeftShin_mex.hh"

namespace SymExpression
{

void Jq_AMWorld_LeftShin_mex_raw(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}

}

#endif // MATLAB_MEX_FILE