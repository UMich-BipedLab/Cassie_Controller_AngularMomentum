/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:23:02 GMT-04:00
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
  double t1937;
  double t1969;
  double t2060;
  double t2147;
  double t1929;
  double t2386;
  double t3504;
  double t2137;
  double t3304;
  double t3462;
  double t1481;
  double t3581;
  double t3640;
  double t3644;
  double t3648;
  double t3943;
  double t3954;
  double t3957;
  double t3995;
  double t4242;
  double t4277;
  double t3468;
  double t4266;
  double t4267;
  double t1433;
  double t4281;
  double t4311;
  double t4312;
  double t4325;
  double t4276;
  double t4316;
  double t4321;
  double t660;
  double t4334;
  double t4352;
  double t4374;
  double t4420;
  double t4322;
  double t4383;
  double t4402;
  double t362;
  double t4432;
  double t4452;
  double t4459;
  double t122;
  double t4560;
  double t4522;
  double t4561;
  double t4579;
  double t4601;
  double t4622;
  double t4629;
  double t4589;
  double t4631;
  double t4636;
  double t4667;
  double t4669;
  double t4671;
  double t4641;
  double t4685;
  double t4687;
  double t4708;
  double t4715;
  double t4732;
  double t4696;
  double t4733;
  double t4742;
  double t4752;
  double t4796;
  double t4813;
  double t4491;
  double t4748;
  double t4827;
  double t4830;
  double t4847;
  double t4853;
  double t4867;
  double t4948;
  double t4960;
  double t4961;
  double t4975;
  double t4989;
  double t5000;
  double t5001;
  double t5004;
  double t5011;
  double t5016;
  double t5039;
  double t5042;
  double t5050;
  double t5057;
  double t5060;
  double t5045;
  double t5083;
  double t5102;
  double t5126;
  double t5127;
  double t5140;
  double t5201;
  double t5205;
  double t5217;
  double t5218;
  double t5214;
  double t5220;
  double t5233;
  double t5239;
  double t5241;
  double t5242;
  double t5238;
  double t5245;
  double t5258;
  double t5262;
  double t5285;
  double t5286;
  double t5260;
  double t5294;
  double t5295;
  double t5300;
  double t5306;
  double t5307;
  double t5441;
  double t5442;
  double t5453;
  double t5400;
  double t5469;
  double t5475;
  double t5484;
  double t5489;
  double t5497;
  double t5483;
  double t5500;
  double t5518;
  double t5542;
  double t5548;
  double t5550;
  double t5528;
  double t5559;
  double t5570;
  double t5591;
  double t5612;
  double t5624;
  double t5704;
  double t5715;
  double t5719;
  double t5730;
  double t5732;
  double t5744;
  double t5750;
  double t5743;
  double t5757;
  double t5759;
  double t5779;
  double t5780;
  double t5782;
  double t5843;
  double t5848;
  double t5851;
  double t5855;
  double t5866;
  double t5880;
  double t5883;
  double t5885;
  double t5960;
  double t5965;
  double t5966;
  double t5941;
  double t5969;
  double t5971;
  double t5986;
  double t6059;
  double t6068;
  double t6072;
  double t6079;
  double t6087;
  double t6089;
  double t6077;
  double t6090;
  double t6102;
  double t6129;
  double t6138;
  double t6139;
  double t6157;
  double t6165;
  double t6168;
  double t6123;
  double t6172;
  double t6182;
  double t6185;
  double t6190;
  double t6237;
  double t6184;
  double t6245;
  double t6256;
  double t6266;
  double t6269;
  double t6271;
  double t6263;
  double t6278;
  double t6281;
  double t6293;
  double t6302;
  double t6304;
  double t6348;
  double t6358;
  double t6368;
  double t6373;
  double t6382;
  double t6389;
  double t6390;
  double t6395;
  double t6410;
  double t6372;
  double t6417;
  double t6423;
  double t6436;
  double t6437;
  double t6439;
  double t6428;
  double t6440;
  double t6442;
  double t6454;
  double t6459;
  double t6470;
  double t6450;
  double t6472;
  double t6486;
  double t6501;
  double t6502;
  double t6509;
  double t6551;
  double t6563;
  double t6565;
  double t6574;
  double t6576;
  double t6602;
  double t6603;
  double t6605;
  double t6580;
  double t6606;
  double t6610;
  double t6623;
  double t6628;
  double t6631;
  double t6614;
  double t6634;
  double t6639;
  double t6651;
  double t6654;
  double t6660;
  double t6644;
  double t6664;
  double t6666;
  double t6674;
  double t6677;
  double t6684;
  double t6733;
  double t6734;
  double t6735;
  double t6744;
  double t6753;
  double t6766;
  double t6767;
  double t6769;
  double t6779;
  double t6782;
  double t6784;
  double t6785;
  double t6791;
  double t6795;
  double t6802;
  double t6787;
  double t6804;
  double t6808;
  double t6819;
  double t6820;
  double t6826;
  double t6849;
  double t6850;
  double t6860;
  double t6870;
  double t6857;
  double t6872;
  double t6873;
  double t6876;
  double t6877;
  double t6878;
  double t6875;
  double t6880;
  double t6881;
  double t6884;
  double t6890;
  double t6891;
  double t6883;
  double t6895;
  double t6896;
  double t6900;
  double t6901;
  double t6910;
  double t6020;
  double t6027;
  double t6028;
  double t6959;
  double t6961;
  double t6962;
  double t6958;
  double t6964;
  double t6967;
  double t6983;
  double t6984;
  double t6988;
  double t6970;
  double t6997;
  double t7000;
  double t7011;
  double t7017;
  double t7019;
  double t7001;
  double t7030;
  double t7037;
  double t7039;
  double t7040;
  double t7048;
  double t7083;
  double t7084;
  double t7095;
  double t7109;
  double t7112;
  double t7121;
  double t7125;
  double t7120;
  double t7128;
  double t7137;
  double t7150;
  double t7154;
  double t7155;
  double t7185;
  double t7186;
  double t7187;
  double t7197;
  double t7203;
  double t7210;
  double t7211;
  double t7212;
  double t7241;
  double t7248;
  double t7249;
  double t7221;
  double t7252;
  double t7259;
  double t7264;
  double t7298;
  double t7299;
  double t7300;
  double t7303;
  double t7306;
  double t7307;
  double t7308;
  double t7310;
  double t7311;
  double t7325;
  double t7327;
  double t7329;
  double t7316;
  double t7331;
  double t7333;
  double t7336;
  double t7337;
  double t7339;
  double t7380;
  double t7383;
  double t7386;
  double t7389;
  double t7393;
  double t7396;
  double t7388;
  double t7398;
  double t7399;
  double t7408;
  double t7410;
  double t7419;
  double t7402;
  double t7423;
  double t7428;
  double t7433;
  double t7435;
  double t7437;
  double t7432;
  double t7440;
  double t7447;
  double t7463;
  double t7464;
  double t7465;
  double t7451;
  double t7470;
  double t7472;
  double t7475;
  double t7478;
  double t7480;
  double t7481;
  double t7484;
  double t7487;
  double t7502;
  double t7508;
  double t7509;
  double t7500;
  double t7514;
  double t7516;
  double t7525;
  double t7528;
  double t7529;
  double t7519;
  double t7530;
  double t7532;
  double t7535;
  double t7539;
  double t7540;
  double t7534;
  double t7541;
  double t7545;
  double t7555;
  double t7556;
  double t7558;
  double t7577;
  double t7578;
  double t7579;
  double t7580;
  double t7589;
  double t7593;
  double t7597;
  double t7590;
  double t7604;
  double t7608;
  double t7615;
  double t7616;
  double t7617;
  double t7642;
  double t7645;
  double t7682;
  double t7684;
  double t7688;
  double t7694;
  double t7695;
  double t7696;
  double t7709;
  double t7710;
  double t7711;
  double t7702;
  double t7712;
  double t7713;
  double t7719;
  double t7742;
  double t7744;
  double t7745;
  double t7750;
  double t7753;
  double t7754;
  double t7755;
  double t7756;
  double t7759;
  double t7749;
  double t7761;
  double t7764;
  double t7766;
  double t7769;
  double t7776;
  double t7765;
  double t7777;
  double t7780;
  double t7783;
  double t7784;
  double t7785;
  double t7782;
  double t7788;
  double t7792;
  double t7797;
  double t7798;
  double t7799;
  double t4403;
  double t4461;
  double t4463;
  double t4496;
  double t4497;
  double t4499;
  double t4840;
  double t4877;
  double t4888;
  double t4904;
  double t4906;
  double t4918;
  double t5105;
  double t5144;
  double t5149;
  double t5156;
  double t5157;
  double t5163;
  double t5297;
  double t5310;
  double t5325;
  double t5334;
  double t5344;
  double t5353;
  double t5571;
  double t5628;
  double t5632;
  double t5646;
  double t5652;
  double t5682;
  double t5768;
  double t5795;
  double t5801;
  double t5807;
  double t5814;
  double t5816;
  double t5875;
  double t5926;
  double t5934;
  double t5943;
  double t6001;
  double t6002;
  double t7887;
  double t6287;
  double t6314;
  double t6316;
  double t6323;
  double t6328;
  double t6332;
  double t6500;
  double t6512;
  double t6514;
  double t6516;
  double t6518;
  double t6528;
  double t6669;
  double t6690;
  double t6703;
  double t6708;
  double t6713;
  double t6722;
  double t6812;
  double t6827;
  double t6829;
  double t6835;
  double t6839;
  double t6840;
  double t6897;
  double t6913;
  double t6916;
  double t6923;
  double t6925;
  double t6928;
  double t6045;
  double t6047;
  double t7038;
  double t7051;
  double t7055;
  double t7060;
  double t7066;
  double t7068;
  double t7144;
  double t7157;
  double t7158;
  double t7162;
  double t7179;
  double t7180;
  double t7208;
  double t7213;
  double t7219;
  double t7232;
  double t7266;
  double t7275;
  double t8005;
  double t7284;
  double t7285;
  double t7286;
  double t7335;
  double t7343;
  double t7346;
  double t7352;
  double t7359;
  double t7366;
  double t8020;
  double t8022;
  double t8023;
  double t7550;
  double t7566;
  double t7567;
  double t7569;
  double t7570;
  double t7571;
  double t7611;
  double t7619;
  double t7620;
  double t7624;
  double t7629;
  double t7631;
  double t7689;
  double t7697;
  double t7701;
  double t7704;
  double t7722;
  double t7723;
  double t8042;
  double t7731;
  double t7732;
  double t7733;
  double t7796;
  double t7801;
  double t7802;
  double t7814;
  double t7815;
  double t7816;
  t1937 = Cos(var1[3]);
  t1969 = Cos(var1[4]);
  t2060 = Cos(var1[5]);
  t2147 = Sin(var1[13]);
  t1929 = Cos(var1[13]);
  t2386 = Sin(var1[5]);
  t3504 = Cos(var1[15]);
  t2137 = t1929*t1937*t1969*t2060;
  t3304 = -1.*t1937*t1969*t2147*t2386;
  t3462 = t2137 + t3304;
  t1481 = Sin(var1[15]);
  t3581 = Cos(var1[14]);
  t3640 = Sin(var1[4]);
  t3644 = -1.*t3581*t1937*t3640;
  t3648 = Sin(var1[14]);
  t3943 = t1937*t1969*t2060*t2147;
  t3954 = t1929*t1937*t1969*t2386;
  t3957 = t3943 + t3954;
  t3995 = t3648*t3957;
  t4242 = t3644 + t3995;
  t4277 = Cos(var1[16]);
  t3468 = t1481*t3462;
  t4266 = t3504*t4242;
  t4267 = t3468 + t4266;
  t1433 = Sin(var1[16]);
  t4281 = t3504*t3462;
  t4311 = -1.*t1481*t4242;
  t4312 = t4281 + t4311;
  t4325 = Cos(var1[17]);
  t4276 = -1.*t1433*t4267;
  t4316 = t4277*t4312;
  t4321 = t4276 + t4316;
  t660 = Sin(var1[17]);
  t4334 = t4277*t4267;
  t4352 = t1433*t4312;
  t4374 = t4334 + t4352;
  t4420 = Cos(var1[18]);
  t4322 = t660*t4321;
  t4383 = t4325*t4374;
  t4402 = t4322 + t4383;
  t362 = Sin(var1[18]);
  t4432 = t4325*t4321;
  t4452 = -1.*t660*t4374;
  t4459 = t4432 + t4452;
  t122 = Cos(var1[19]);
  t4560 = Sin(var1[3]);
  t4522 = t1937*t2060*t3640;
  t4561 = t4560*t2386;
  t4579 = t4522 + t4561;
  t4601 = t2060*t4560;
  t4622 = -1.*t1937*t3640*t2386;
  t4629 = t4601 + t4622;
  t4589 = -1.*t2147*t4579;
  t4631 = t1929*t4629;
  t4636 = t4589 + t4631;
  t4667 = t1929*t4579;
  t4669 = t2147*t4629;
  t4671 = t4667 + t4669;
  t4641 = t1481*t4636;
  t4685 = t3504*t3648*t4671;
  t4687 = t4641 + t4685;
  t4708 = t3504*t4636;
  t4715 = -1.*t3648*t1481*t4671;
  t4732 = t4708 + t4715;
  t4696 = -1.*t1433*t4687;
  t4733 = t4277*t4732;
  t4742 = t4696 + t4733;
  t4752 = t4277*t4687;
  t4796 = t1433*t4732;
  t4813 = t4752 + t4796;
  t4491 = Sin(var1[19]);
  t4748 = t660*t4742;
  t4827 = t4325*t4813;
  t4830 = t4748 + t4827;
  t4847 = t4325*t4742;
  t4853 = -1.*t660*t4813;
  t4867 = t4847 + t4853;
  t4948 = -1.*t1937*t1969*t3648;
  t4960 = t2147*t4579;
  t4961 = -1.*t2060*t4560;
  t4975 = t1937*t3640*t2386;
  t4989 = t4961 + t4975;
  t5000 = t1929*t4989;
  t5001 = t4960 + t5000;
  t5004 = t3581*t5001;
  t5011 = t4948 + t5004;
  t5016 = -1.*t4277*t1481*t5011;
  t5039 = -1.*t3504*t1433*t5011;
  t5042 = t5016 + t5039;
  t5050 = t3504*t4277*t5011;
  t5057 = -1.*t1481*t1433*t5011;
  t5060 = t5050 + t5057;
  t5045 = t660*t5042;
  t5083 = t4325*t5060;
  t5102 = t5045 + t5083;
  t5126 = t4325*t5042;
  t5127 = -1.*t660*t5060;
  t5140 = t5126 + t5127;
  t5201 = -1.*t1929*t4989;
  t5205 = t4589 + t5201;
  t5217 = -1.*t2147*t4989;
  t5218 = t4667 + t5217;
  t5214 = t1481*t5205;
  t5220 = t3504*t3648*t5218;
  t5233 = t5214 + t5220;
  t5239 = t3504*t5205;
  t5241 = -1.*t3648*t1481*t5218;
  t5242 = t5239 + t5241;
  t5238 = -1.*t1433*t5233;
  t5245 = t4277*t5242;
  t5258 = t5238 + t5245;
  t5262 = t4277*t5233;
  t5285 = t1433*t5242;
  t5286 = t5262 + t5285;
  t5260 = t660*t5258;
  t5294 = t4325*t5286;
  t5295 = t5260 + t5294;
  t5300 = t4325*t5258;
  t5306 = -1.*t660*t5286;
  t5307 = t5300 + t5306;
  t5441 = t3581*t1937*t1969;
  t5442 = t3648*t5001;
  t5453 = t5441 + t5442;
  t5400 = -1.*t1481*t5218;
  t5469 = -1.*t3504*t5453;
  t5475 = t5400 + t5469;
  t5484 = t3504*t5218;
  t5489 = -1.*t1481*t5453;
  t5497 = t5484 + t5489;
  t5483 = t1433*t5475;
  t5500 = t4277*t5497;
  t5518 = t5483 + t5500;
  t5542 = t4277*t5475;
  t5548 = -1.*t1433*t5497;
  t5550 = t5542 + t5548;
  t5528 = -1.*t660*t5518;
  t5559 = t4325*t5550;
  t5570 = t5528 + t5559;
  t5591 = t4325*t5518;
  t5612 = t660*t5550;
  t5624 = t5591 + t5612;
  t5704 = t1481*t5218;
  t5715 = t3504*t5453;
  t5719 = t5704 + t5715;
  t5730 = -1.*t1433*t5719;
  t5732 = t5730 + t5500;
  t5744 = -1.*t4277*t5719;
  t5750 = t5744 + t5548;
  t5743 = -1.*t660*t5732;
  t5757 = t4325*t5750;
  t5759 = t5743 + t5757;
  t5779 = t4325*t5732;
  t5780 = t660*t5750;
  t5782 = t5779 + t5780;
  t5843 = t4277*t5719;
  t5848 = t1433*t5497;
  t5851 = t5843 + t5848;
  t5855 = -1.*t4325*t5851;
  t5866 = t5743 + t5855;
  t5880 = -1.*t660*t5851;
  t5883 = t5779 + t5880;
  t5885 = t4420*t5883;
  t5960 = t660*t5732;
  t5965 = t4325*t5851;
  t5966 = t5960 + t5965;
  t5941 = -1.*t362*t5883;
  t5969 = -1.*t362*t5966;
  t5971 = t5969 + t5885;
  t5986 = t4491*t5971;
  t6059 = -1.*t2060*t4560*t3640;
  t6068 = t1937*t2386;
  t6072 = t6059 + t6068;
  t6079 = -1.*t1937*t2060;
  t6087 = -1.*t4560*t3640*t2386;
  t6089 = t6079 + t6087;
  t6077 = t1929*t6072;
  t6090 = -1.*t2147*t6089;
  t6102 = t6077 + t6090;
  t6129 = -1.*t3581*t1969*t4560;
  t6138 = t2147*t6072;
  t6139 = t1929*t6089;
  t6157 = t6138 + t6139;
  t6165 = t3648*t6157;
  t6168 = t6129 + t6165;
  t6123 = t1481*t6102;
  t6172 = t3504*t6168;
  t6182 = t6123 + t6172;
  t6185 = t3504*t6102;
  t6190 = -1.*t1481*t6168;
  t6237 = t6185 + t6190;
  t6184 = -1.*t1433*t6182;
  t6245 = t4277*t6237;
  t6256 = t6184 + t6245;
  t6266 = t4277*t6182;
  t6269 = t1433*t6237;
  t6271 = t6266 + t6269;
  t6263 = t660*t6256;
  t6278 = t4325*t6271;
  t6281 = t6263 + t6278;
  t6293 = t4325*t6256;
  t6302 = -1.*t660*t6271;
  t6304 = t6293 + t6302;
  t6348 = t1929*t1969*t2060*t4560;
  t6358 = -1.*t1969*t2147*t4560*t2386;
  t6368 = t6348 + t6358;
  t6373 = -1.*t3581*t4560*t3640;
  t6382 = t1969*t2060*t2147*t4560;
  t6389 = t1929*t1969*t4560*t2386;
  t6390 = t6382 + t6389;
  t6395 = t3648*t6390;
  t6410 = t6373 + t6395;
  t6372 = t1481*t6368;
  t6417 = t3504*t6410;
  t6423 = t6372 + t6417;
  t6436 = t3504*t6368;
  t6437 = -1.*t1481*t6410;
  t6439 = t6436 + t6437;
  t6428 = -1.*t1433*t6423;
  t6440 = t4277*t6439;
  t6442 = t6428 + t6440;
  t6454 = t4277*t6423;
  t6459 = t1433*t6439;
  t6470 = t6454 + t6459;
  t6450 = t660*t6442;
  t6472 = t4325*t6470;
  t6486 = t6450 + t6472;
  t6501 = t4325*t6442;
  t6502 = -1.*t660*t6470;
  t6509 = t6501 + t6502;
  t6551 = t2060*t4560*t3640;
  t6563 = -1.*t1937*t2386;
  t6565 = t6551 + t6563;
  t6574 = -1.*t2147*t6565;
  t6576 = t6574 + t6139;
  t6602 = t1929*t6565;
  t6603 = t2147*t6089;
  t6605 = t6602 + t6603;
  t6580 = t1481*t6576;
  t6606 = t3504*t3648*t6605;
  t6610 = t6580 + t6606;
  t6623 = t3504*t6576;
  t6628 = -1.*t3648*t1481*t6605;
  t6631 = t6623 + t6628;
  t6614 = -1.*t1433*t6610;
  t6634 = t4277*t6631;
  t6639 = t6614 + t6634;
  t6651 = t4277*t6610;
  t6654 = t1433*t6631;
  t6660 = t6651 + t6654;
  t6644 = t660*t6639;
  t6664 = t4325*t6660;
  t6666 = t6644 + t6664;
  t6674 = t4325*t6639;
  t6677 = -1.*t660*t6660;
  t6684 = t6674 + t6677;
  t6733 = -1.*t1969*t3648*t4560;
  t6734 = t2147*t6565;
  t6735 = t1937*t2060;
  t6744 = t4560*t3640*t2386;
  t6753 = t6735 + t6744;
  t6766 = t1929*t6753;
  t6767 = t6734 + t6766;
  t6769 = t3581*t6767;
  t6779 = t6733 + t6769;
  t6782 = -1.*t4277*t1481*t6779;
  t6784 = -1.*t3504*t1433*t6779;
  t6785 = t6782 + t6784;
  t6791 = t3504*t4277*t6779;
  t6795 = -1.*t1481*t1433*t6779;
  t6802 = t6791 + t6795;
  t6787 = t660*t6785;
  t6804 = t4325*t6802;
  t6808 = t6787 + t6804;
  t6819 = t4325*t6785;
  t6820 = -1.*t660*t6802;
  t6826 = t6819 + t6820;
  t6849 = -1.*t1929*t6753;
  t6850 = t6574 + t6849;
  t6860 = -1.*t2147*t6753;
  t6870 = t6602 + t6860;
  t6857 = t1481*t6850;
  t6872 = t3504*t3648*t6870;
  t6873 = t6857 + t6872;
  t6876 = t3504*t6850;
  t6877 = -1.*t3648*t1481*t6870;
  t6878 = t6876 + t6877;
  t6875 = -1.*t1433*t6873;
  t6880 = t4277*t6878;
  t6881 = t6875 + t6880;
  t6884 = t4277*t6873;
  t6890 = t1433*t6878;
  t6891 = t6884 + t6890;
  t6883 = t660*t6881;
  t6895 = t4325*t6891;
  t6896 = t6883 + t6895;
  t6900 = t4325*t6881;
  t6901 = -1.*t660*t6891;
  t6910 = t6900 + t6901;
  t6020 = t4420*t5966;
  t6027 = t362*t5883;
  t6028 = t6020 + t6027;
  t6959 = t3581*t1969*t4560;
  t6961 = t3648*t6767;
  t6962 = t6959 + t6961;
  t6958 = -1.*t1481*t6870;
  t6964 = -1.*t3504*t6962;
  t6967 = t6958 + t6964;
  t6983 = t3504*t6870;
  t6984 = -1.*t1481*t6962;
  t6988 = t6983 + t6984;
  t6970 = t1433*t6967;
  t6997 = t4277*t6988;
  t7000 = t6970 + t6997;
  t7011 = t4277*t6967;
  t7017 = -1.*t1433*t6988;
  t7019 = t7011 + t7017;
  t7001 = -1.*t660*t7000;
  t7030 = t4325*t7019;
  t7037 = t7001 + t7030;
  t7039 = t4325*t7000;
  t7040 = t660*t7019;
  t7048 = t7039 + t7040;
  t7083 = t1481*t6870;
  t7084 = t3504*t6962;
  t7095 = t7083 + t7084;
  t7109 = -1.*t1433*t7095;
  t7112 = t7109 + t6997;
  t7121 = -1.*t4277*t7095;
  t7125 = t7121 + t7017;
  t7120 = -1.*t660*t7112;
  t7128 = t4325*t7125;
  t7137 = t7120 + t7128;
  t7150 = t4325*t7112;
  t7154 = t660*t7125;
  t7155 = t7150 + t7154;
  t7185 = t4277*t7095;
  t7186 = t1433*t6988;
  t7187 = t7185 + t7186;
  t7197 = -1.*t4325*t7187;
  t7203 = t7120 + t7197;
  t7210 = -1.*t660*t7187;
  t7211 = t7150 + t7210;
  t7212 = t4420*t7211;
  t7241 = t660*t7112;
  t7248 = t4325*t7187;
  t7249 = t7241 + t7248;
  t7221 = -1.*t362*t7211;
  t7252 = -1.*t362*t7249;
  t7259 = t7252 + t7212;
  t7264 = t4491*t7259;
  t7298 = t3648*t3640;
  t7299 = t1969*t2060*t2147;
  t7300 = t1929*t1969*t2386;
  t7303 = t7299 + t7300;
  t7306 = t3581*t7303;
  t7307 = t7298 + t7306;
  t7308 = -1.*t4277*t1481*t7307;
  t7310 = -1.*t3504*t1433*t7307;
  t7311 = t7308 + t7310;
  t7325 = t3504*t4277*t7307;
  t7327 = -1.*t1481*t1433*t7307;
  t7329 = t7325 + t7327;
  t7316 = t660*t7311;
  t7331 = t4325*t7329;
  t7333 = t7316 + t7331;
  t7336 = t4325*t7311;
  t7337 = -1.*t660*t7329;
  t7339 = t7336 + t7337;
  t7380 = -1.*t1969*t2060*t2147;
  t7383 = -1.*t1929*t1969*t2386;
  t7386 = t7380 + t7383;
  t7389 = t1929*t1969*t2060;
  t7393 = -1.*t1969*t2147*t2386;
  t7396 = t7389 + t7393;
  t7388 = t1481*t7386;
  t7398 = t3504*t3648*t7396;
  t7399 = t7388 + t7398;
  t7408 = t3504*t7386;
  t7410 = -1.*t3648*t1481*t7396;
  t7419 = t7408 + t7410;
  t7402 = -1.*t1433*t7399;
  t7423 = t4277*t7419;
  t7428 = t7402 + t7423;
  t7433 = t4277*t7399;
  t7435 = t1433*t7419;
  t7437 = t7433 + t7435;
  t7432 = t660*t7428;
  t7440 = t4325*t7437;
  t7447 = t7432 + t7440;
  t7463 = t4325*t7428;
  t7464 = -1.*t660*t7437;
  t7465 = t7463 + t7464;
  t7451 = -1.*t362*t7447;
  t7470 = t4420*t7465;
  t7472 = t7451 + t7470;
  t7475 = -1.*t122*t7472;
  t7478 = t4420*t7447;
  t7480 = t362*t7465;
  t7481 = t7478 + t7480;
  t7484 = t4491*t7481;
  t7487 = t7475 + t7484;
  t7502 = -1.*t3581*t3640;
  t7508 = t3648*t7303;
  t7509 = t7502 + t7508;
  t7500 = -1.*t1481*t7396;
  t7514 = -1.*t3504*t7509;
  t7516 = t7500 + t7514;
  t7525 = t3504*t7396;
  t7528 = -1.*t1481*t7509;
  t7529 = t7525 + t7528;
  t7519 = t1433*t7516;
  t7530 = t4277*t7529;
  t7532 = t7519 + t7530;
  t7535 = t4277*t7516;
  t7539 = -1.*t1433*t7529;
  t7540 = t7535 + t7539;
  t7534 = -1.*t660*t7532;
  t7541 = t4325*t7540;
  t7545 = t7534 + t7541;
  t7555 = t4325*t7532;
  t7556 = t660*t7540;
  t7558 = t7555 + t7556;
  t7577 = t1481*t7396;
  t7578 = t3504*t7509;
  t7579 = t7577 + t7578;
  t7580 = -1.*t1433*t7579;
  t7589 = t7580 + t7530;
  t7593 = -1.*t4277*t7579;
  t7597 = t7593 + t7539;
  t7590 = -1.*t660*t7589;
  t7604 = t4325*t7597;
  t7608 = t7590 + t7604;
  t7615 = t4325*t7589;
  t7616 = t660*t7597;
  t7617 = t7615 + t7616;
  t7642 = t4277*t7579;
  t7645 = t1433*t7529;
  t7682 = t7642 + t7645;
  t7684 = -1.*t4325*t7682;
  t7688 = t7590 + t7684;
  t7694 = -1.*t660*t7682;
  t7695 = t7615 + t7694;
  t7696 = t4420*t7695;
  t7709 = t660*t7589;
  t7710 = t4325*t7682;
  t7711 = t7709 + t7710;
  t7702 = -1.*t362*t7695;
  t7712 = -1.*t362*t7711;
  t7713 = t7712 + t7696;
  t7719 = t4491*t7713;
  t7742 = -1.*t1929*t2060*t3640;
  t7744 = t2147*t3640*t2386;
  t7745 = t7742 + t7744;
  t7750 = -1.*t3581*t1969;
  t7753 = -1.*t2060*t2147*t3640;
  t7754 = -1.*t1929*t3640*t2386;
  t7755 = t7753 + t7754;
  t7756 = t3648*t7755;
  t7759 = t7750 + t7756;
  t7749 = t1481*t7745;
  t7761 = t3504*t7759;
  t7764 = t7749 + t7761;
  t7766 = t3504*t7745;
  t7769 = -1.*t1481*t7759;
  t7776 = t7766 + t7769;
  t7765 = -1.*t1433*t7764;
  t7777 = t4277*t7776;
  t7780 = t7765 + t7777;
  t7783 = t4277*t7764;
  t7784 = t1433*t7776;
  t7785 = t7783 + t7784;
  t7782 = t660*t7780;
  t7788 = t4325*t7785;
  t7792 = t7782 + t7788;
  t7797 = t4325*t7780;
  t7798 = -1.*t660*t7785;
  t7799 = t7797 + t7798;
  t4403 = -1.*t362*t4402;
  t4461 = t4420*t4459;
  t4463 = t4403 + t4461;
  t4496 = t4420*t4402;
  t4497 = t362*t4459;
  t4499 = t4496 + t4497;
  t4840 = -1.*t362*t4830;
  t4877 = t4420*t4867;
  t4888 = t4840 + t4877;
  t4904 = t4420*t4830;
  t4906 = t362*t4867;
  t4918 = t4904 + t4906;
  t5105 = -1.*t362*t5102;
  t5144 = t4420*t5140;
  t5149 = t5105 + t5144;
  t5156 = t4420*t5102;
  t5157 = t362*t5140;
  t5163 = t5156 + t5157;
  t5297 = -1.*t362*t5295;
  t5310 = t4420*t5307;
  t5325 = t5297 + t5310;
  t5334 = t4420*t5295;
  t5344 = t362*t5307;
  t5353 = t5334 + t5344;
  t5571 = t362*t5570;
  t5628 = t4420*t5624;
  t5632 = t5571 + t5628;
  t5646 = t4420*t5570;
  t5652 = -1.*t362*t5624;
  t5682 = t5646 + t5652;
  t5768 = t362*t5759;
  t5795 = t4420*t5782;
  t5801 = t5768 + t5795;
  t5807 = t4420*t5759;
  t5814 = -1.*t362*t5782;
  t5816 = t5807 + t5814;
  t5875 = t362*t5866;
  t5926 = t5875 + t5885;
  t5934 = t4420*t5866;
  t5943 = t5934 + t5941;
  t6001 = -1.*t4420*t5966;
  t6002 = t6001 + t5941;
  t7887 = t122*t5971;
  t6287 = -1.*t362*t6281;
  t6314 = t4420*t6304;
  t6316 = t6287 + t6314;
  t6323 = t4420*t6281;
  t6328 = t362*t6304;
  t6332 = t6323 + t6328;
  t6500 = -1.*t362*t6486;
  t6512 = t4420*t6509;
  t6514 = t6500 + t6512;
  t6516 = t4420*t6486;
  t6518 = t362*t6509;
  t6528 = t6516 + t6518;
  t6669 = -1.*t362*t6666;
  t6690 = t4420*t6684;
  t6703 = t6669 + t6690;
  t6708 = t4420*t6666;
  t6713 = t362*t6684;
  t6722 = t6708 + t6713;
  t6812 = -1.*t362*t6808;
  t6827 = t4420*t6826;
  t6829 = t6812 + t6827;
  t6835 = t4420*t6808;
  t6839 = t362*t6826;
  t6840 = t6835 + t6839;
  t6897 = -1.*t362*t6896;
  t6913 = t4420*t6910;
  t6916 = t6897 + t6913;
  t6923 = t4420*t6896;
  t6925 = t362*t6910;
  t6928 = t6923 + t6925;
  t6045 = t122*t6028;
  t6047 = t5986 + t6045;
  t7038 = t362*t7037;
  t7051 = t4420*t7048;
  t7055 = t7038 + t7051;
  t7060 = t4420*t7037;
  t7066 = -1.*t362*t7048;
  t7068 = t7060 + t7066;
  t7144 = t362*t7137;
  t7157 = t4420*t7155;
  t7158 = t7144 + t7157;
  t7162 = t4420*t7137;
  t7179 = -1.*t362*t7155;
  t7180 = t7162 + t7179;
  t7208 = t362*t7203;
  t7213 = t7208 + t7212;
  t7219 = t4420*t7203;
  t7232 = t7219 + t7221;
  t7266 = -1.*t4420*t7249;
  t7275 = t7266 + t7221;
  t8005 = t122*t7259;
  t7284 = t4420*t7249;
  t7285 = t362*t7211;
  t7286 = t7284 + t7285;
  t7335 = -1.*t362*t7333;
  t7343 = t4420*t7339;
  t7346 = t7335 + t7343;
  t7352 = t4420*t7333;
  t7359 = t362*t7339;
  t7366 = t7352 + t7359;
  t8020 = t4491*t7472;
  t8022 = t122*t7481;
  t8023 = t8020 + t8022;
  t7550 = t362*t7545;
  t7566 = t4420*t7558;
  t7567 = t7550 + t7566;
  t7569 = t4420*t7545;
  t7570 = -1.*t362*t7558;
  t7571 = t7569 + t7570;
  t7611 = t362*t7608;
  t7619 = t4420*t7617;
  t7620 = t7611 + t7619;
  t7624 = t4420*t7608;
  t7629 = -1.*t362*t7617;
  t7631 = t7624 + t7629;
  t7689 = t362*t7688;
  t7697 = t7689 + t7696;
  t7701 = t4420*t7688;
  t7704 = t7701 + t7702;
  t7722 = -1.*t4420*t7711;
  t7723 = t7722 + t7702;
  t8042 = t122*t7713;
  t7731 = t4420*t7711;
  t7732 = t362*t7695;
  t7733 = t7731 + t7732;
  t7796 = -1.*t362*t7792;
  t7801 = t4420*t7799;
  t7802 = t7796 + t7801;
  t7814 = t4420*t7792;
  t7815 = t362*t7799;
  t7816 = t7814 + t7815;
  p_output1[0]=(-1.*t122*t6316 + t4491*t6332)*var2[3] + (-1.*t122*t4463 + t4491*t4499)*var2[4] + (-1.*t122*t4888 + t4491*t4918)*var2[5] + (-1.*t122*t5325 + t4491*t5353)*var2[13] + (-1.*t122*t5149 + t4491*t5163)*var2[14] + (t4491*t5632 - 1.*t122*t5682)*var2[15] + (t4491*t5801 - 1.*t122*t5816)*var2[16] + (t4491*t5926 - 1.*t122*t5943)*var2[17] + (t5986 - 1.*t122*t6002)*var2[18] + t6047*var2[19];
  p_output1[1]=(-1.*t122*t5971 + t4491*t6028)*var2[3] + (-1.*t122*t6514 + t4491*t6528)*var2[4] + (-1.*t122*t6703 + t4491*t6722)*var2[5] + (-1.*t122*t6916 + t4491*t6928)*var2[13] + (-1.*t122*t6829 + t4491*t6840)*var2[14] + (t4491*t7055 - 1.*t122*t7068)*var2[15] + (t4491*t7158 - 1.*t122*t7180)*var2[16] + (t4491*t7213 - 1.*t122*t7232)*var2[17] + (t7264 - 1.*t122*t7275)*var2[18] + (t7264 + t122*t7286)*var2[19];
  p_output1[2]=(-1.*t122*t7802 + t4491*t7816)*var2[4] + t7487*var2[5] + t7487*var2[13] + (-1.*t122*t7346 + t4491*t7366)*var2[14] + (t4491*t7567 - 1.*t122*t7571)*var2[15] + (t4491*t7620 - 1.*t122*t7631)*var2[16] + (t4491*t7697 - 1.*t122*t7704)*var2[17] + (t7719 - 1.*t122*t7723)*var2[18] + (t7719 + t122*t7733)*var2[19];
  p_output1[3]=(t4491*t6316 + t122*t6332)*var2[3] + (t4463*t4491 + t122*t4499)*var2[4] + (t4491*t4888 + t122*t4918)*var2[5] + (t4491*t5325 + t122*t5353)*var2[13] + (t4491*t5149 + t122*t5163)*var2[14] + (t122*t5632 + t4491*t5682)*var2[15] + (t122*t5801 + t4491*t5816)*var2[16] + (t122*t5926 + t4491*t5943)*var2[17] + (t4491*t6002 + t7887)*var2[18] + (-1.*t4491*t6028 + t7887)*var2[19];
  p_output1[4]=t6047*var2[3] + (t4491*t6514 + t122*t6528)*var2[4] + (t4491*t6703 + t122*t6722)*var2[5] + (t4491*t6916 + t122*t6928)*var2[13] + (t4491*t6829 + t122*t6840)*var2[14] + (t122*t7055 + t4491*t7068)*var2[15] + (t122*t7158 + t4491*t7180)*var2[16] + (t122*t7213 + t4491*t7232)*var2[17] + (t4491*t7275 + t8005)*var2[18] + (-1.*t4491*t7286 + t8005)*var2[19];
  p_output1[5]=(t4491*t7802 + t122*t7816)*var2[4] + t8023*var2[5] + t8023*var2[13] + (t4491*t7346 + t122*t7366)*var2[14] + (t122*t7567 + t4491*t7571)*var2[15] + (t122*t7620 + t4491*t7631)*var2[16] + (t122*t7697 + t4491*t7704)*var2[17] + (t4491*t7723 + t8042)*var2[18] + (-1.*t4491*t7733 + t8042)*var2[19];
  p_output1[6]=(-1.*t3581*t6157 + t6733)*var2[3] + (-1.*t1937*t3640*t3648 - 1.*t3581*t3957)*var2[4] - 1.*t3581*t4671*var2[5] - 1.*t3581*t5218*var2[13] + t5453*var2[14];
  p_output1[7]=(t1937*t1969*t3648 - 1.*t3581*t5001)*var2[3] + (-1.*t3640*t3648*t4560 - 1.*t3581*t6390)*var2[4] - 1.*t3581*t6605*var2[5] - 1.*t3581*t6870*var2[13] + t6962*var2[14];
  p_output1[8]=(-1.*t1969*t3648 - 1.*t3581*t7755)*var2[4] - 1.*t3581*t7396*var2[5] - 1.*t3581*t7396*var2[13] + t7509*var2[14];
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
  plhs[0] = mxCreateDoubleMatrix((mwSize) 3, (mwSize) 3, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,var1,var2);


}

#else // MATLAB_MEX_FILE

#include "dR_RightToeJoint_mex.hh"

namespace SymExpression
{

void dR_RightToeJoint_mex_raw(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}

}

#endif // MATLAB_MEX_FILE
