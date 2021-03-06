/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:27:09 GMT-04:00
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
  double t255;
  double t277;
  double t430;
  double t1269;
  double t2483;
  double t2104;
  double t2114;
  double t2788;
  double t3242;
  double t2220;
  double t3031;
  double t3228;
  double t1859;
  double t3252;
  double t3261;
  double t3345;
  double t3747;
  double t3235;
  double t3477;
  double t3523;
  double t1847;
  double t3880;
  double t3907;
  double t3927;
  double t188;
  double t560;
  double t813;
  double t1834;
  double t4352;
  double t4486;
  double t4562;
  double t3691;
  double t3953;
  double t3967;
  double t4707;
  double t217;
  double t1279;
  double t4111;
  double t4712;
  double t4835;
  double t636;
  double t5682;
  double t5810;
  double t5841;
  double t6017;
  double t6223;
  double t6829;
  double t7040;
  double t7041;
  double t6711;
  double t7114;
  double t7115;
  double t7282;
  double t7473;
  double t7888;
  double t7899;
  double t7904;
  double t7272;
  double t7506;
  double t7700;
  double t7709;
  double t7907;
  double t7918;
  double t8003;
  double t8115;
  double t8143;
  double t8210;
  double t8216;
  double t5117;
  double t5162;
  double t5545;
  double t5869;
  double t6245;
  double t6247;
  double t6317;
  double t6333;
  double t6383;
  double t6397;
  double t6516;
  double t6517;
  double t6526;
  double t6528;
  double t7919;
  double t7943;
  double t7946;
  double t8168;
  double t8396;
  double t8428;
  double t8429;
  double t8465;
  double t8472;
  double t8518;
  double t8527;
  double t8599;
  double t8613;
  double t8633;
  double t487;
  double t523;
  double t585;
  double t936;
  double t1011;
  double t1058;
  double t1101;
  double t1226;
  double t1309;
  double t1426;
  double t1634;
  double t8882;
  double t8903;
  double t8908;
  double t8910;
  double t8911;
  double t8924;
  double t8928;
  double t8929;
  double t8930;
  double t8942;
  double t8944;
  double t8950;
  double t8951;
  double t8952;
  double t8954;
  double t8959;
  double t8960;
  double t8963;
  double t8797;
  double t8799;
  double t8834;
  double t8841;
  double t8842;
  double t8844;
  double t8874;
  double t8876;
  double t8880;
  double t9120;
  double t9130;
  double t9132;
  double t9133;
  double t9193;
  double t9196;
  double t9197;
  double t9199;
  double t9121;
  double t9141;
  double t9143;
  double t9144;
  double t9183;
  double t9187;
  double t9188;
  double t9189;
  double t9190;
  double t9195;
  double t9200;
  double t9201;
  double t9202;
  double t9203;
  double t9204;
  double t9205;
  double t9206;
  double t9208;
  double t9065;
  double t9069;
  double t9070;
  double t9071;
  double t9091;
  double t9099;
  double t9100;
  double t9101;
  double t9115;
  double t9238;
  double t9240;
  double t9241;
  double t9242;
  double t9243;
  double t9244;
  double t9245;
  double t9246;
  double t9247;
  double t9248;
  double t9249;
  double t9251;
  double t9252;
  double t9253;
  double t9256;
  double t9257;
  double t9258;
  double t9259;
  double t9260;
  double t9261;
  double t9262;
  double t9263;
  double t9264;
  double t9266;
  double t9267;
  double t9268;
  double t9270;
  double t9271;
  double t9219;
  double t9220;
  double t9222;
  double t9223;
  double t9224;
  double t9225;
  double t9227;
  double t9228;
  double t9229;
  double t9230;
  double t9232;
  double t9233;
  double t9234;
  double t9235;
  double t9283;
  double t9284;
  double t9285;
  double t9281;
  double t9292;
  double t9293;
  double t9295;
  double t9299;
  double t9300;
  double t9302;
  double t9320;
  double t9321;
  double t9322;
  double t9325;
  double t9327;
  double t9328;
  double t9329;
  double t9331;
  double t9334;
  double t9335;
  double t9282;
  double t9289;
  double t9290;
  double t9296;
  double t9303;
  double t9305;
  double t9306;
  double t9307;
  double t9308;
  double t9309;
  double t9310;
  double t9313;
  double t9316;
  double t9317;
  double t9318;
  double t9323;
  double t9324;
  double t9326;
  double t9330;
  double t9336;
  double t9337;
  double t9338;
  double t9339;
  double t9340;
  double t9342;
  double t9343;
  double t9344;
  double t9345;
  double t9346;
  double t9347;
  double t9356;
  double t9357;
  double t9358;
  double t9354;
  double t9364;
  double t9365;
  double t9366;
  double t9355;
  double t9360;
  double t9361;
  double t9370;
  double t9371;
  double t9373;
  double t9375;
  double t9376;
  double t9377;
  double t9391;
  double t9392;
  double t9393;
  double t9394;
  double t9396;
  double t9397;
  double t9398;
  double t9401;
  double t9402;
  double t9403;
  double t9405;
  double t9406;
  double t9407;
  double t9409;
  double t9410;
  double t9411;
  double t9362;
  double t9367;
  double t9368;
  double t9374;
  double t9378;
  double t9379;
  double t9380;
  double t9381;
  double t9382;
  double t9383;
  double t9385;
  double t9386;
  double t9387;
  double t9388;
  double t9389;
  double t9399;
  double t9400;
  double t9404;
  double t9408;
  double t9412;
  double t9414;
  double t9415;
  double t9416;
  double t9417;
  double t9418;
  double t9419;
  double t9420;
  double t9421;
  double t9422;
  double t9423;
  double t9430;
  double t9432;
  double t9433;
  double t9434;
  double t9438;
  double t9439;
  double t9440;
  double t9431;
  double t9435;
  double t9436;
  double t9444;
  double t9445;
  double t9446;
  double t9437;
  double t9441;
  double t9442;
  double t9449;
  double t9450;
  double t9451;
  double t9453;
  double t9454;
  double t9455;
  double t9469;
  double t9470;
  double t9472;
  double t9471;
  double t9473;
  double t9474;
  double t9475;
  double t9476;
  double t9477;
  double t9478;
  double t9480;
  double t9481;
  double t9484;
  double t9485;
  double t9486;
  double t9488;
  double t9491;
  double t9492;
  double t9494;
  double t9496;
  double t9497;
  double t9443;
  double t9447;
  double t9448;
  double t9452;
  double t9456;
  double t9457;
  double t9458;
  double t9459;
  double t9460;
  double t9461;
  double t9462;
  double t9463;
  double t9464;
  double t9465;
  double t9466;
  double t9482;
  double t9483;
  double t9487;
  double t9493;
  double t9498;
  double t9500;
  double t9502;
  double t9503;
  double t9504;
  double t9505;
  double t9508;
  double t9511;
  double t9512;
  double t9513;
  double t9514;
  double t9522;
  double t9524;
  double t9526;
  double t9528;
  double t9527;
  double t9529;
  double t9531;
  double t9533;
  double t9534;
  double t9535;
  double t9539;
  double t9540;
  double t9541;
  double t9532;
  double t9536;
  double t9537;
  double t9545;
  double t9546;
  double t9547;
  double t9538;
  double t9542;
  double t9543;
  double t9551;
  double t9552;
  double t9553;
  double t9555;
  double t9556;
  double t9557;
  double t9571;
  double t9572;
  double t9575;
  double t9577;
  double t9578;
  double t9579;
  double t9576;
  double t9581;
  double t9582;
  double t9583;
  double t9585;
  double t9586;
  double t9587;
  double t9588;
  double t9589;
  double t9592;
  double t9593;
  double t9594;
  double t9596;
  double t9597;
  double t9599;
  double t9601;
  double t9602;
  double t9603;
  double t9544;
  double t9549;
  double t9550;
  double t9554;
  double t9558;
  double t9559;
  double t9561;
  double t9562;
  double t9563;
  double t9564;
  double t9565;
  double t9566;
  double t9567;
  double t9568;
  double t9569;
  double t9590;
  double t9591;
  double t9595;
  double t9600;
  double t9604;
  double t9605;
  double t9606;
  double t9607;
  double t9608;
  double t9609;
  double t9610;
  double t9611;
  double t9612;
  double t9613;
  double t9614;
  double t9570;
  double t9615;
  double t9616;
  double t9617;
  double t9618;
  double t9619;
  double t9621;
  t255 = Cos(var1[7]);
  t277 = -1.*t255;
  t430 = 0. + t277;
  t1269 = Cos(var1[4]);
  t2483 = Cos(var1[11]);
  t2104 = Cos(var1[12]);
  t2114 = Sin(var1[11]);
  t2788 = Sin(var1[12]);
  t3242 = Cos(var1[10]);
  t2220 = t2104*t2114;
  t3031 = t2483*t2788;
  t3228 = 0. + t2220 + t3031;
  t1859 = Sin(var1[10]);
  t3252 = t2483*t2104;
  t3261 = -1.*t2114*t2788;
  t3345 = 0. + t3252 + t3261;
  t3747 = Sin(var1[9]);
  t3235 = -1.*t1859*t3228;
  t3477 = t3242*t3345;
  t3523 = 0. + t3235 + t3477;
  t1847 = Cos(var1[9]);
  t3880 = t3242*t3228;
  t3907 = t1859*t3345;
  t3927 = 0. + t3880 + t3907;
  t188 = Sin(var1[4]);
  t560 = Sin(var1[5]);
  t813 = Sin(var1[6]);
  t1834 = Cos(var1[8]);
  t4352 = t3747*t3523;
  t4486 = t1847*t3927;
  t4562 = 0. + t4352 + t4486;
  t3691 = t1847*t3523;
  t3953 = -1.*t3747*t3927;
  t3967 = 0. + t3691 + t3953;
  t4707 = Sin(var1[8]);
  t217 = Cos(var1[6]);
  t1279 = Sin(var1[7]);
  t4111 = t1834*t3967;
  t4712 = -1.*t4562*t4707;
  t4835 = 0. + t4111 + t4712;
  t636 = Cos(var1[5]);
  t5682 = t1834*t4562;
  t5810 = t3967*t4707;
  t5841 = 0. + t5682 + t5810;
  t6017 = t1279*t4835;
  t6223 = 0. + t6017;
  t6829 = -1.*t2483*t2104;
  t7040 = t2114*t2788;
  t7041 = 0. + t6829 + t7040;
  t6711 = t1859*t3228;
  t7114 = t3242*t7041;
  t7115 = 0. + t6711 + t7114;
  t7282 = -1.*t1859*t7041;
  t7473 = 0. + t3880 + t7282;
  t7888 = t1847*t7115;
  t7899 = t3747*t7473;
  t7904 = 0. + t7888 + t7899;
  t7272 = -1.*t3747*t7115;
  t7506 = t1847*t7473;
  t7700 = 0. + t7272 + t7506;
  t7709 = t1834*t7700;
  t7907 = -1.*t7904*t4707;
  t7918 = 0. + t7709 + t7907;
  t8003 = t1834*t7904;
  t8115 = t7700*t4707;
  t8143 = 0. + t8003 + t8115;
  t8210 = t1279*t7918;
  t8216 = 0. + t8210;
  t5117 = t255*t4835;
  t5162 = 0. + t5117;
  t5545 = -1.*t1269*t5162;
  t5869 = -1.*t813*t5841;
  t6245 = t217*t6223;
  t6247 = 0. + t5869 + t6245;
  t6317 = t560*t6247;
  t6333 = t217*t5841;
  t6383 = t813*t6223;
  t6397 = 0. + t6333 + t6383;
  t6516 = t636*t6397;
  t6517 = 0. + t6317 + t6516;
  t6526 = -1.*t188*t6517;
  t6528 = t5545 + t6526;
  t7919 = t255*t7918;
  t7943 = 0. + t7919;
  t7946 = -1.*t1269*t7943;
  t8168 = -1.*t813*t8143;
  t8396 = t217*t8216;
  t8428 = 0. + t8168 + t8396;
  t8429 = t560*t8428;
  t8465 = t217*t8143;
  t8472 = t813*t8216;
  t8518 = 0. + t8465 + t8472;
  t8527 = t636*t8518;
  t8599 = 0. + t8429 + t8527;
  t8613 = -1.*t188*t8599;
  t8633 = t7946 + t8613;
  t487 = t217*t430;
  t523 = 0. + t487;
  t585 = t523*t560;
  t936 = t430*t813;
  t1011 = 0. + t936;
  t1058 = t636*t1011;
  t1101 = 0. + t585 + t1058;
  t1226 = -1.*t188*t1101;
  t1309 = 0. + t1279;
  t1426 = -1.*t1269*t1309;
  t1634 = t1226 + t1426;
  t8882 = -1.*t560*t6247;
  t8903 = -1.*t636*t6397;
  t8908 = t8882 + t8903;
  t8910 = var2[4]*t8908;
  t8911 = t636*t6247;
  t8924 = -1.*t560*t6397;
  t8928 = t8911 + t8924;
  t8929 = var2[3]*t1269*t8928;
  t8930 = t8910 + t8929;
  t8942 = -1.*t560*t8428;
  t8944 = -1.*t636*t8518;
  t8950 = t8942 + t8944;
  t8951 = var2[4]*t8950;
  t8952 = t636*t8428;
  t8954 = -1.*t560*t8518;
  t8959 = t8952 + t8954;
  t8960 = var2[3]*t1269*t8959;
  t8963 = t8951 + t8960;
  t8797 = -1.*t523*t560;
  t8799 = -1.*t636*t1011;
  t8834 = t8797 + t8799;
  t8841 = var2[4]*t8834;
  t8842 = t636*t523;
  t8844 = -1.*t560*t1011;
  t8874 = t8842 + t8844;
  t8876 = var2[3]*t1269*t8874;
  t8880 = t8841 + t8876;
  t9120 = t5869 + t6245;
  t9130 = -1.*t217*t5841;
  t9132 = -1.*t813*t6223;
  t9133 = t9130 + t9132;
  t9193 = t8168 + t8396;
  t9196 = -1.*t217*t8143;
  t9197 = -1.*t813*t8216;
  t9199 = t9196 + t9197;
  t9121 = -1.*t560*t9120;
  t9141 = t636*t9133;
  t9143 = t9121 + t9141;
  t9144 = var2[4]*t9143;
  t9183 = t636*t9120;
  t9187 = t560*t9133;
  t9188 = t9183 + t9187;
  t9189 = var2[3]*t1269*t9188;
  t9190 = t9144 + t9189;
  t9195 = -1.*t560*t9193;
  t9200 = t636*t9199;
  t9201 = t9195 + t9200;
  t9202 = var2[4]*t9201;
  t9203 = t636*t9193;
  t9204 = t560*t9199;
  t9205 = t9203 + t9204;
  t9206 = var2[3]*t1269*t9205;
  t9208 = t9202 + t9206;
  t9065 = -1.*t217*t430*t560;
  t9069 = -1.*t636*t430*t813;
  t9070 = t9065 + t9069;
  t9071 = var2[4]*t9070;
  t9091 = t636*t217*t430;
  t9099 = -1.*t430*t560*t813;
  t9100 = t9091 + t9099;
  t9101 = var2[3]*t1269*t9100;
  t9115 = t9071 + t9101;
  t9238 = -1.*var2[5]*t1279*t4835;
  t9240 = -1.*var2[6]*t1279*t4835;
  t9241 = t636*t217*t255*t4835;
  t9242 = -1.*t255*t560*t813*t4835;
  t9243 = t9241 + t9242;
  t9244 = var2[4]*t9243;
  t9245 = t188*t1279*t4835;
  t9246 = t217*t255*t560*t4835;
  t9247 = t636*t255*t813*t4835;
  t9248 = t9246 + t9247;
  t9249 = t1269*t9248;
  t9251 = t9245 + t9249;
  t9252 = var2[3]*t9251;
  t9253 = t9238 + t9240 + t9244 + t9252;
  t9256 = -1.*var2[5]*t1279*t7918;
  t9257 = -1.*var2[6]*t1279*t7918;
  t9258 = t636*t217*t255*t7918;
  t9259 = -1.*t255*t560*t813*t7918;
  t9260 = t9258 + t9259;
  t9261 = var2[4]*t9260;
  t9262 = t188*t1279*t7918;
  t9263 = t217*t255*t560*t7918;
  t9264 = t636*t255*t813*t7918;
  t9266 = t9263 + t9264;
  t9267 = t1269*t9266;
  t9268 = t9262 + t9267;
  t9270 = var2[3]*t9268;
  t9271 = t9256 + t9257 + t9261 + t9270;
  t9219 = var2[5]*t255;
  t9220 = var2[6]*t255;
  t9222 = t636*t217*t1279;
  t9223 = -1.*t560*t813*t1279;
  t9224 = t9222 + t9223;
  t9225 = var2[4]*t9224;
  t9227 = -1.*t255*t188;
  t9228 = t217*t560*t1279;
  t9229 = t636*t813*t1279;
  t9230 = t9228 + t9229;
  t9232 = t1269*t9230;
  t9233 = t9227 + t9232;
  t9234 = var2[3]*t9233;
  t9235 = t9219 + t9220 + t9225 + t9234;
  t9283 = -1.*t1834*t4562;
  t9284 = -1.*t3967*t4707;
  t9285 = t9283 + t9284;
  t9281 = t4111 + t4712;
  t9292 = -1.*t813*t9281;
  t9293 = t217*t1279*t9285;
  t9295 = t9292 + t9293;
  t9299 = t217*t9281;
  t9300 = t813*t1279*t9285;
  t9302 = t9299 + t9300;
  t9320 = -1.*t1834*t7904;
  t9321 = -1.*t7700*t4707;
  t9322 = t9320 + t9321;
  t9325 = t7709 + t7907;
  t9327 = t813*t1279*t9322;
  t9328 = t217*t9325;
  t9329 = t9327 + t9328;
  t9331 = t217*t1279*t9322;
  t9334 = -1.*t813*t9325;
  t9335 = t9331 + t9334;
  t9282 = var2[7]*t9281;
  t9289 = var2[5]*t255*t9285;
  t9290 = var2[6]*t255*t9285;
  t9296 = t636*t9295;
  t9303 = -1.*t560*t9302;
  t9305 = t9296 + t9303;
  t9306 = var2[4]*t9305;
  t9307 = -1.*t255*t188*t9285;
  t9308 = t560*t9295;
  t9309 = t636*t9302;
  t9310 = t9308 + t9309;
  t9313 = t1269*t9310;
  t9316 = t9307 + t9313;
  t9317 = var2[3]*t9316;
  t9318 = t9282 + t9289 + t9290 + t9306 + t9317;
  t9323 = var2[5]*t255*t9322;
  t9324 = var2[6]*t255*t9322;
  t9326 = var2[7]*t9325;
  t9330 = -1.*t560*t9329;
  t9336 = t636*t9335;
  t9337 = t9330 + t9336;
  t9338 = var2[4]*t9337;
  t9339 = -1.*t255*t188*t9322;
  t9340 = t636*t9329;
  t9342 = t560*t9335;
  t9343 = t9340 + t9342;
  t9344 = t1269*t9343;
  t9345 = t9339 + t9344;
  t9346 = var2[3]*t9345;
  t9347 = t9323 + t9324 + t9326 + t9338 + t9346;
  t9356 = -1.*t3747*t3523;
  t9357 = -1.*t1847*t3927;
  t9358 = t9356 + t9357;
  t9354 = t3691 + t3953;
  t9364 = t1834*t9358;
  t9365 = -1.*t9354*t4707;
  t9366 = t9364 + t9365;
  t9355 = t1834*t9354;
  t9360 = t9358*t4707;
  t9361 = t9355 + t9360;
  t9370 = -1.*t813*t9361;
  t9371 = t217*t1279*t9366;
  t9373 = t9370 + t9371;
  t9375 = t217*t9361;
  t9376 = t813*t1279*t9366;
  t9377 = t9375 + t9376;
  t9391 = -1.*t1847*t7115;
  t9392 = -1.*t3747*t7473;
  t9393 = t9391 + t9392;
  t9394 = t1834*t9393;
  t9396 = t7272 + t7506;
  t9397 = -1.*t9396*t4707;
  t9398 = t9394 + t9397;
  t9401 = t1834*t9396;
  t9402 = t9393*t4707;
  t9403 = t9401 + t9402;
  t9405 = t813*t1279*t9398;
  t9406 = t217*t9403;
  t9407 = t9405 + t9406;
  t9409 = t217*t1279*t9398;
  t9410 = -1.*t813*t9403;
  t9411 = t9409 + t9410;
  t9362 = var2[7]*t9361;
  t9367 = var2[5]*t255*t9366;
  t9368 = var2[6]*t255*t9366;
  t9374 = t636*t9373;
  t9378 = -1.*t560*t9377;
  t9379 = t9374 + t9378;
  t9380 = var2[4]*t9379;
  t9381 = -1.*t255*t188*t9366;
  t9382 = t560*t9373;
  t9383 = t636*t9377;
  t9385 = t9382 + t9383;
  t9386 = t1269*t9385;
  t9387 = t9381 + t9386;
  t9388 = var2[3]*t9387;
  t9389 = t9362 + t9367 + t9368 + t9380 + t9388;
  t9399 = var2[5]*t255*t9398;
  t9400 = var2[6]*t255*t9398;
  t9404 = var2[7]*t9403;
  t9408 = -1.*t560*t9407;
  t9412 = t636*t9411;
  t9414 = t9408 + t9412;
  t9415 = var2[4]*t9414;
  t9416 = -1.*t255*t188*t9398;
  t9417 = t636*t9407;
  t9418 = t560*t9411;
  t9419 = t9417 + t9418;
  t9420 = t1269*t9419;
  t9421 = t9416 + t9420;
  t9422 = var2[3]*t9421;
  t9423 = t9399 + t9400 + t9404 + t9415 + t9422;
  t9430 = t3235 + t3477;
  t9432 = -1.*t3242*t3228;
  t9433 = -1.*t1859*t3345;
  t9434 = t9432 + t9433;
  t9438 = -1.*t3747*t9430;
  t9439 = t1847*t9434;
  t9440 = t9438 + t9439;
  t9431 = t1847*t9430;
  t9435 = t3747*t9434;
  t9436 = t9431 + t9435;
  t9444 = t1834*t9440;
  t9445 = -1.*t9436*t4707;
  t9446 = t9444 + t9445;
  t9437 = t1834*t9436;
  t9441 = t9440*t4707;
  t9442 = t9437 + t9441;
  t9449 = -1.*t813*t9442;
  t9450 = t217*t1279*t9446;
  t9451 = t9449 + t9450;
  t9453 = t217*t9442;
  t9454 = t813*t1279*t9446;
  t9455 = t9453 + t9454;
  t9469 = -1.*t3242*t7041;
  t9470 = t3235 + t9469;
  t9472 = t3880 + t7282;
  t9471 = t1847*t9470;
  t9473 = -1.*t3747*t9472;
  t9474 = t9471 + t9473;
  t9475 = t1834*t9474;
  t9476 = t3747*t9470;
  t9477 = t1847*t9472;
  t9478 = t9476 + t9477;
  t9480 = -1.*t9478*t4707;
  t9481 = t9475 + t9480;
  t9484 = t1834*t9478;
  t9485 = t9474*t4707;
  t9486 = t9484 + t9485;
  t9488 = t813*t1279*t9481;
  t9491 = t217*t9486;
  t9492 = t9488 + t9491;
  t9494 = t217*t1279*t9481;
  t9496 = -1.*t813*t9486;
  t9497 = t9494 + t9496;
  t9443 = var2[7]*t9442;
  t9447 = var2[5]*t255*t9446;
  t9448 = var2[6]*t255*t9446;
  t9452 = t636*t9451;
  t9456 = -1.*t560*t9455;
  t9457 = t9452 + t9456;
  t9458 = var2[4]*t9457;
  t9459 = -1.*t255*t188*t9446;
  t9460 = t560*t9451;
  t9461 = t636*t9455;
  t9462 = t9460 + t9461;
  t9463 = t1269*t9462;
  t9464 = t9459 + t9463;
  t9465 = var2[3]*t9464;
  t9466 = t9443 + t9447 + t9448 + t9458 + t9465;
  t9482 = var2[5]*t255*t9481;
  t9483 = var2[6]*t255*t9481;
  t9487 = var2[7]*t9486;
  t9493 = -1.*t560*t9492;
  t9498 = t636*t9497;
  t9500 = t9493 + t9498;
  t9502 = var2[4]*t9500;
  t9503 = -1.*t255*t188*t9481;
  t9504 = t636*t9492;
  t9505 = t560*t9497;
  t9508 = t9504 + t9505;
  t9511 = t1269*t9508;
  t9512 = t9503 + t9511;
  t9513 = var2[3]*t9512;
  t9514 = t9482 + t9483 + t9487 + t9502 + t9513;
  t9522 = -1.*t2104*t2114;
  t9524 = -1.*t2483*t2788;
  t9526 = t9522 + t9524;
  t9528 = t3252 + t3261;
  t9527 = t1859*t9526;
  t9529 = t3242*t9528;
  t9531 = t9527 + t9529;
  t9533 = t3242*t9526;
  t9534 = -1.*t1859*t9528;
  t9535 = t9533 + t9534;
  t9539 = -1.*t3747*t9531;
  t9540 = t1847*t9535;
  t9541 = t9539 + t9540;
  t9532 = t1847*t9531;
  t9536 = t3747*t9535;
  t9537 = t9532 + t9536;
  t9545 = t1834*t9541;
  t9546 = -1.*t9537*t4707;
  t9547 = t9545 + t9546;
  t9538 = t1834*t9537;
  t9542 = t9541*t4707;
  t9543 = t9538 + t9542;
  t9551 = -1.*t813*t9543;
  t9552 = t217*t1279*t9547;
  t9553 = t9551 + t9552;
  t9555 = t217*t9543;
  t9556 = t813*t1279*t9547;
  t9557 = t9555 + t9556;
  t9571 = t2220 + t3031;
  t9572 = -1.*t1859*t9571;
  t9575 = t9572 + t9529;
  t9577 = t3242*t9571;
  t9578 = t1859*t9528;
  t9579 = t9577 + t9578;
  t9576 = t1847*t9575;
  t9581 = -1.*t3747*t9579;
  t9582 = t9576 + t9581;
  t9583 = t1834*t9582;
  t9585 = t3747*t9575;
  t9586 = t1847*t9579;
  t9587 = t9585 + t9586;
  t9588 = -1.*t9587*t4707;
  t9589 = t9583 + t9588;
  t9592 = t1834*t9587;
  t9593 = t9582*t4707;
  t9594 = t9592 + t9593;
  t9596 = t813*t1279*t9589;
  t9597 = t217*t9594;
  t9599 = t9596 + t9597;
  t9601 = t217*t1279*t9589;
  t9602 = -1.*t813*t9594;
  t9603 = t9601 + t9602;
  t9544 = var2[7]*t9543;
  t9549 = var2[5]*t255*t9547;
  t9550 = var2[6]*t255*t9547;
  t9554 = t636*t9553;
  t9558 = -1.*t560*t9557;
  t9559 = t9554 + t9558;
  t9561 = var2[4]*t9559;
  t9562 = -1.*t255*t188*t9547;
  t9563 = t560*t9553;
  t9564 = t636*t9557;
  t9565 = t9563 + t9564;
  t9566 = t1269*t9565;
  t9567 = t9562 + t9566;
  t9568 = var2[3]*t9567;
  t9569 = t9544 + t9549 + t9550 + t9561 + t9568;
  t9590 = var2[5]*t255*t9589;
  t9591 = var2[6]*t255*t9589;
  t9595 = var2[7]*t9594;
  t9600 = -1.*t560*t9599;
  t9604 = t636*t9603;
  t9605 = t9600 + t9604;
  t9606 = var2[4]*t9605;
  t9607 = -1.*t255*t188*t9589;
  t9608 = t636*t9599;
  t9609 = t560*t9603;
  t9610 = t9608 + t9609;
  t9611 = t1269*t9610;
  t9612 = t9607 + t9611;
  t9613 = var2[3]*t9612;
  t9614 = t9590 + t9591 + t9595 + t9606 + t9613;
  t9570 = -0.000099*t9569;
  t9615 = 0.000287*t9614;
  t9616 = t9570 + t9615;
  t9617 = 0.000171*t9569;
  t9618 = -0.000099*t9614;
  t9619 = t9617 + t9618;
  t9621 = -1.e-6*t9614;
  p_output1[0]=0;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=0;
  p_output1[9]=0;
  p_output1[10]=0;
  p_output1[11]=0;
  p_output1[12]=-1.e-6*t1634*var2[3] - 0.000099*t6528*var2[3] + 0.000287*t8633*var2[3];
  p_output1[13]=0.000171*t6528*var2[3] - 0.000099*t8633*var2[3];
  p_output1[14]=0.000449*t1634*var2[3] - 1.e-6*t8633*var2[3];
  p_output1[15]=-1.e-6*t8880 - 0.000099*t8930 + 0.000287*t8963;
  p_output1[16]=0.000171*t8930 - 0.000099*t8963;
  p_output1[17]=0.000449*t8880 - 1.e-6*t8963;
  p_output1[18]=-1.e-6*t9115 - 0.000099*t9190 + 0.000287*t9208;
  p_output1[19]=0.000171*t9190 - 0.000099*t9208;
  p_output1[20]=0.000449*t9115 - 1.e-6*t9208;
  p_output1[21]=-1.e-6*t9235 - 0.000099*t9253 + 0.000287*t9271;
  p_output1[22]=0.000171*t9253 - 0.000099*t9271;
  p_output1[23]=0.000449*t9235 - 1.e-6*t9271;
  p_output1[24]=-0.000099*t9318 + 0.000287*t9347;
  p_output1[25]=0.000171*t9318 - 0.000099*t9347;
  p_output1[26]=-1.e-6*t9347;
  p_output1[27]=-0.000099*t9389 + 0.000287*t9423;
  p_output1[28]=0.000171*t9389 - 0.000099*t9423;
  p_output1[29]=-1.e-6*t9423;
  p_output1[30]=-0.000099*t9466 + 0.000287*t9514;
  p_output1[31]=0.000171*t9466 - 0.000099*t9514;
  p_output1[32]=-1.e-6*t9514;
  p_output1[33]=t9616;
  p_output1[34]=t9619;
  p_output1[35]=t9621;
  p_output1[36]=t9616;
  p_output1[37]=t9619;
  p_output1[38]=t9621;
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

#include "Jq_AMBody_LeftToe_mex.hh"

namespace SymExpression
{

void Jq_AMBody_LeftToe_mex_raw(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}

}

#endif // MATLAB_MEX_FILE
