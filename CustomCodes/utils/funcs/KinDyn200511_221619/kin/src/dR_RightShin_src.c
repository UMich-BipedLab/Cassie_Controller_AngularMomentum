/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:29:42 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "dR_RightShin_src.h"

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
  double t491;
  double t670;
  double t775;
  double t1003;
  double t413;
  double t1063;
  double t1219;
  double t991;
  double t1171;
  double t1186;
  double t169;
  double t1248;
  double t1308;
  double t1339;
  double t1351;
  double t1403;
  double t1510;
  double t1536;
  double t1565;
  double t1596;
  double t1755;
  double t1187;
  double t1656;
  double t1662;
  double t129;
  double t1767;
  double t1771;
  double t1776;
  double t49;
  double t2450;
  double t2419;
  double t2461;
  double t2486;
  double t2814;
  double t3225;
  double t3369;
  double t2783;
  double t3514;
  double t3708;
  double t3807;
  double t3827;
  double t3830;
  double t2043;
  double t3776;
  double t3875;
  double t3918;
  double t4173;
  double t4256;
  double t4324;
  double t4579;
  double t4630;
  double t4648;
  double t4693;
  double t4708;
  double t4721;
  double t4725;
  double t4840;
  double t4867;
  double t5031;
  double t5071;
  double t5086;
  double t5103;
  double t5075;
  double t5108;
  double t5113;
  double t5156;
  double t5157;
  double t5159;
  double t5389;
  double t5390;
  double t5391;
  double t5377;
  double t5464;
  double t5475;
  double t5483;
  double t5497;
  double t5511;
  double t5527;
  double t5661;
  double t5666;
  double t5667;
  double t5579;
  double t5681;
  double t5686;
  double t5742;
  double t5883;
  double t5889;
  double t5894;
  double t5987;
  double t5988;
  double t6047;
  double t5899;
  double t6054;
  double t6057;
  double t6064;
  double t6091;
  double t6093;
  double t6094;
  double t6114;
  double t6115;
  double t6058;
  double t6119;
  double t6165;
  double t6205;
  double t6215;
  double t6218;
  double t6243;
  double t6248;
  double t6252;
  double t6255;
  double t6258;
  double t6259;
  double t6260;
  double t6262;
  double t6263;
  double t6253;
  double t6268;
  double t6270;
  double t6275;
  double t6277;
  double t6281;
  double t6313;
  double t6315;
  double t6319;
  double t6328;
  double t6334;
  double t6344;
  double t6351;
  double t6352;
  double t6338;
  double t6359;
  double t6361;
  double t6364;
  double t6366;
  double t6367;
  double t6401;
  double t6402;
  double t6403;
  double t6407;
  double t6408;
  double t6409;
  double t6412;
  double t6422;
  double t6428;
  double t6518;
  double t6520;
  double t6526;
  double t6529;
  double t6522;
  double t6536;
  double t6538;
  double t6545;
  double t6546;
  double t6547;
  double t5829;
  double t5830;
  double t5851;
  double t6597;
  double t6598;
  double t6601;
  double t6596;
  double t6605;
  double t6606;
  double t6608;
  double t6609;
  double t6614;
  double t6615;
  double t6636;
  double t6638;
  double t6639;
  double t6622;
  double t6641;
  double t6642;
  double t6644;
  double t6684;
  double t6685;
  double t6689;
  double t6690;
  double t6696;
  double t6697;
  double t6726;
  double t6730;
  double t6735;
  double t6740;
  double t6742;
  double t6746;
  double t6738;
  double t6748;
  double t6750;
  double t6759;
  double t6774;
  double t6776;
  double t6751;
  double t6786;
  double t6813;
  double t6818;
  double t6824;
  double t6826;
  double t6827;
  double t6828;
  double t6829;
  double t6838;
  double t6839;
  double t6840;
  double t6836;
  double t6841;
  double t6843;
  double t6846;
  double t6852;
  double t6855;
  double t6856;
  double t6868;
  double t6869;
  double t6870;
  double t6861;
  double t6871;
  double t6880;
  double t6882;
  double t6905;
  double t6908;
  double t6909;
  double t6915;
  double t6917;
  double t6919;
  double t6923;
  double t6929;
  double t6930;
  double t6911;
  double t6932;
  double t6933;
  double t6938;
  double t6939;
  double t6941;
  double t1748;
  double t1870;
  double t1976;
  double t2052;
  double t2082;
  double t2116;
  double t4146;
  double t4336;
  double t4342;
  double t4482;
  double t4518;
  double t4522;
  double t4872;
  double t4908;
  double t4918;
  double t5002;
  double t5011;
  double t5016;
  double t5134;
  double t5183;
  double t5209;
  double t5277;
  double t5284;
  double t5285;
  double t5477;
  double t5565;
  double t5572;
  double t5649;
  double t5775;
  double t5776;
  double t7003;
  double t6199;
  double t6219;
  double t6221;
  double t6223;
  double t6226;
  double t6227;
  double t6271;
  double t6284;
  double t6287;
  double t6295;
  double t6298;
  double t6300;
  double t6362;
  double t6371;
  double t6372;
  double t6378;
  double t6379;
  double t6386;
  double t6429;
  double t6451;
  double t6456;
  double t6495;
  double t6496;
  double t6498;
  double t6544;
  double t6548;
  double t6550;
  double t6552;
  double t6571;
  double t6581;
  double t5852;
  double t5857;
  double t6607;
  double t6616;
  double t6621;
  double t6623;
  double t6645;
  double t6653;
  double t7080;
  double t6661;
  double t6663;
  double t6665;
  double t6698;
  double t6699;
  double t6701;
  double t6709;
  double t6714;
  double t6716;
  double t7110;
  double t7115;
  double t7116;
  double t6844;
  double t6858;
  double t6860;
  double t6863;
  double t6883;
  double t6886;
  double t7129;
  double t6894;
  double t6895;
  double t6898;
  double t6937;
  double t6942;
  double t6944;
  double t6947;
  double t6949;
  double t6950;
  t491 = Cos(var1[3]);
  t670 = Cos(var1[4]);
  t775 = Cos(var1[5]);
  t1003 = Sin(var1[13]);
  t413 = Cos(var1[13]);
  t1063 = Sin(var1[5]);
  t1219 = Cos(var1[15]);
  t991 = t413*t491*t670*t775;
  t1171 = -1.*t491*t670*t1003*t1063;
  t1186 = t991 + t1171;
  t169 = Sin(var1[15]);
  t1248 = Cos(var1[14]);
  t1308 = Sin(var1[4]);
  t1339 = -1.*t1248*t491*t1308;
  t1351 = Sin(var1[14]);
  t1403 = t491*t670*t775*t1003;
  t1510 = t413*t491*t670*t1063;
  t1536 = t1403 + t1510;
  t1565 = t1351*t1536;
  t1596 = t1339 + t1565;
  t1755 = Cos(var1[16]);
  t1187 = t169*t1186;
  t1656 = t1219*t1596;
  t1662 = t1187 + t1656;
  t129 = Sin(var1[16]);
  t1767 = t1219*t1186;
  t1771 = -1.*t169*t1596;
  t1776 = t1767 + t1771;
  t49 = Cos(var1[17]);
  t2450 = Sin(var1[3]);
  t2419 = t491*t775*t1308;
  t2461 = t2450*t1063;
  t2486 = t2419 + t2461;
  t2814 = t775*t2450;
  t3225 = -1.*t491*t1308*t1063;
  t3369 = t2814 + t3225;
  t2783 = -1.*t1003*t2486;
  t3514 = t413*t3369;
  t3708 = t2783 + t3514;
  t3807 = t413*t2486;
  t3827 = t1003*t3369;
  t3830 = t3807 + t3827;
  t2043 = Sin(var1[17]);
  t3776 = t169*t3708;
  t3875 = t1219*t1351*t3830;
  t3918 = t3776 + t3875;
  t4173 = t1219*t3708;
  t4256 = -1.*t1351*t169*t3830;
  t4324 = t4173 + t4256;
  t4579 = -1.*t491*t670*t1351;
  t4630 = t1003*t2486;
  t4648 = -1.*t775*t2450;
  t4693 = t491*t1308*t1063;
  t4708 = t4648 + t4693;
  t4721 = t413*t4708;
  t4725 = t4630 + t4721;
  t4840 = t1248*t4725;
  t4867 = t4579 + t4840;
  t5031 = -1.*t413*t4708;
  t5071 = t2783 + t5031;
  t5086 = -1.*t1003*t4708;
  t5103 = t3807 + t5086;
  t5075 = t169*t5071;
  t5108 = t1219*t1351*t5103;
  t5113 = t5075 + t5108;
  t5156 = t1219*t5071;
  t5157 = -1.*t1351*t169*t5103;
  t5159 = t5156 + t5157;
  t5389 = t1248*t491*t670;
  t5390 = t1351*t4725;
  t5391 = t5389 + t5390;
  t5377 = -1.*t169*t5103;
  t5464 = -1.*t1219*t5391;
  t5475 = t5377 + t5464;
  t5483 = t1219*t5103;
  t5497 = -1.*t169*t5391;
  t5511 = t5483 + t5497;
  t5527 = t1755*t5511;
  t5661 = t169*t5103;
  t5666 = t1219*t5391;
  t5667 = t5661 + t5666;
  t5579 = -1.*t129*t5511;
  t5681 = -1.*t129*t5667;
  t5686 = t5681 + t5527;
  t5742 = t2043*t5686;
  t5883 = -1.*t775*t2450*t1308;
  t5889 = t491*t1063;
  t5894 = t5883 + t5889;
  t5987 = -1.*t491*t775;
  t5988 = -1.*t2450*t1308*t1063;
  t6047 = t5987 + t5988;
  t5899 = t413*t5894;
  t6054 = -1.*t1003*t6047;
  t6057 = t5899 + t6054;
  t6064 = -1.*t1248*t670*t2450;
  t6091 = t1003*t5894;
  t6093 = t413*t6047;
  t6094 = t6091 + t6093;
  t6114 = t1351*t6094;
  t6115 = t6064 + t6114;
  t6058 = t169*t6057;
  t6119 = t1219*t6115;
  t6165 = t6058 + t6119;
  t6205 = t1219*t6057;
  t6215 = -1.*t169*t6115;
  t6218 = t6205 + t6215;
  t6243 = t413*t670*t775*t2450;
  t6248 = -1.*t670*t1003*t2450*t1063;
  t6252 = t6243 + t6248;
  t6255 = -1.*t1248*t2450*t1308;
  t6258 = t670*t775*t1003*t2450;
  t6259 = t413*t670*t2450*t1063;
  t6260 = t6258 + t6259;
  t6262 = t1351*t6260;
  t6263 = t6255 + t6262;
  t6253 = t169*t6252;
  t6268 = t1219*t6263;
  t6270 = t6253 + t6268;
  t6275 = t1219*t6252;
  t6277 = -1.*t169*t6263;
  t6281 = t6275 + t6277;
  t6313 = t775*t2450*t1308;
  t6315 = -1.*t491*t1063;
  t6319 = t6313 + t6315;
  t6328 = -1.*t1003*t6319;
  t6334 = t6328 + t6093;
  t6344 = t413*t6319;
  t6351 = t1003*t6047;
  t6352 = t6344 + t6351;
  t6338 = t169*t6334;
  t6359 = t1219*t1351*t6352;
  t6361 = t6338 + t6359;
  t6364 = t1219*t6334;
  t6366 = -1.*t1351*t169*t6352;
  t6367 = t6364 + t6366;
  t6401 = -1.*t670*t1351*t2450;
  t6402 = t1003*t6319;
  t6403 = t491*t775;
  t6407 = t2450*t1308*t1063;
  t6408 = t6403 + t6407;
  t6409 = t413*t6408;
  t6412 = t6402 + t6409;
  t6422 = t1248*t6412;
  t6428 = t6401 + t6422;
  t6518 = -1.*t413*t6408;
  t6520 = t6328 + t6518;
  t6526 = -1.*t1003*t6408;
  t6529 = t6344 + t6526;
  t6522 = t169*t6520;
  t6536 = t1219*t1351*t6529;
  t6538 = t6522 + t6536;
  t6545 = t1219*t6520;
  t6546 = -1.*t1351*t169*t6529;
  t6547 = t6545 + t6546;
  t5829 = t1755*t5667;
  t5830 = t129*t5511;
  t5851 = t5829 + t5830;
  t6597 = t1248*t670*t2450;
  t6598 = t1351*t6412;
  t6601 = t6597 + t6598;
  t6596 = -1.*t169*t6529;
  t6605 = -1.*t1219*t6601;
  t6606 = t6596 + t6605;
  t6608 = t1219*t6529;
  t6609 = -1.*t169*t6601;
  t6614 = t6608 + t6609;
  t6615 = t1755*t6614;
  t6636 = t169*t6529;
  t6638 = t1219*t6601;
  t6639 = t6636 + t6638;
  t6622 = -1.*t129*t6614;
  t6641 = -1.*t129*t6639;
  t6642 = t6641 + t6615;
  t6644 = t2043*t6642;
  t6684 = t1351*t1308;
  t6685 = t670*t775*t1003;
  t6689 = t413*t670*t1063;
  t6690 = t6685 + t6689;
  t6696 = t1248*t6690;
  t6697 = t6684 + t6696;
  t6726 = -1.*t670*t775*t1003;
  t6730 = -1.*t413*t670*t1063;
  t6735 = t6726 + t6730;
  t6740 = t413*t670*t775;
  t6742 = -1.*t670*t1003*t1063;
  t6746 = t6740 + t6742;
  t6738 = t169*t6735;
  t6748 = t1219*t1351*t6746;
  t6750 = t6738 + t6748;
  t6759 = t1219*t6735;
  t6774 = -1.*t1351*t169*t6746;
  t6776 = t6759 + t6774;
  t6751 = -1.*t129*t6750;
  t6786 = t1755*t6776;
  t6813 = t6751 + t6786;
  t6818 = -1.*t49*t6813;
  t6824 = t1755*t6750;
  t6826 = t129*t6776;
  t6827 = t6824 + t6826;
  t6828 = t2043*t6827;
  t6829 = t6818 + t6828;
  t6838 = -1.*t1248*t1308;
  t6839 = t1351*t6690;
  t6840 = t6838 + t6839;
  t6836 = -1.*t169*t6746;
  t6841 = -1.*t1219*t6840;
  t6843 = t6836 + t6841;
  t6846 = t1219*t6746;
  t6852 = -1.*t169*t6840;
  t6855 = t6846 + t6852;
  t6856 = t1755*t6855;
  t6868 = t169*t6746;
  t6869 = t1219*t6840;
  t6870 = t6868 + t6869;
  t6861 = -1.*t129*t6855;
  t6871 = -1.*t129*t6870;
  t6880 = t6871 + t6856;
  t6882 = t2043*t6880;
  t6905 = -1.*t413*t775*t1308;
  t6908 = t1003*t1308*t1063;
  t6909 = t6905 + t6908;
  t6915 = -1.*t1248*t670;
  t6917 = -1.*t775*t1003*t1308;
  t6919 = -1.*t413*t1308*t1063;
  t6923 = t6917 + t6919;
  t6929 = t1351*t6923;
  t6930 = t6915 + t6929;
  t6911 = t169*t6909;
  t6932 = t1219*t6930;
  t6933 = t6911 + t6932;
  t6938 = t1219*t6909;
  t6939 = -1.*t169*t6930;
  t6941 = t6938 + t6939;
  t1748 = -1.*t129*t1662;
  t1870 = t1755*t1776;
  t1976 = t1748 + t1870;
  t2052 = t1755*t1662;
  t2082 = t129*t1776;
  t2116 = t2052 + t2082;
  t4146 = -1.*t129*t3918;
  t4336 = t1755*t4324;
  t4342 = t4146 + t4336;
  t4482 = t1755*t3918;
  t4518 = t129*t4324;
  t4522 = t4482 + t4518;
  t4872 = -1.*t1755*t169*t4867;
  t4908 = -1.*t1219*t129*t4867;
  t4918 = t4872 + t4908;
  t5002 = t1219*t1755*t4867;
  t5011 = -1.*t169*t129*t4867;
  t5016 = t5002 + t5011;
  t5134 = -1.*t129*t5113;
  t5183 = t1755*t5159;
  t5209 = t5134 + t5183;
  t5277 = t1755*t5113;
  t5284 = t129*t5159;
  t5285 = t5277 + t5284;
  t5477 = t129*t5475;
  t5565 = t5477 + t5527;
  t5572 = t1755*t5475;
  t5649 = t5572 + t5579;
  t5775 = -1.*t1755*t5667;
  t5776 = t5775 + t5579;
  t7003 = t49*t5686;
  t6199 = -1.*t129*t6165;
  t6219 = t1755*t6218;
  t6221 = t6199 + t6219;
  t6223 = t1755*t6165;
  t6226 = t129*t6218;
  t6227 = t6223 + t6226;
  t6271 = -1.*t129*t6270;
  t6284 = t1755*t6281;
  t6287 = t6271 + t6284;
  t6295 = t1755*t6270;
  t6298 = t129*t6281;
  t6300 = t6295 + t6298;
  t6362 = -1.*t129*t6361;
  t6371 = t1755*t6367;
  t6372 = t6362 + t6371;
  t6378 = t1755*t6361;
  t6379 = t129*t6367;
  t6386 = t6378 + t6379;
  t6429 = -1.*t1755*t169*t6428;
  t6451 = -1.*t1219*t129*t6428;
  t6456 = t6429 + t6451;
  t6495 = t1219*t1755*t6428;
  t6496 = -1.*t169*t129*t6428;
  t6498 = t6495 + t6496;
  t6544 = -1.*t129*t6538;
  t6548 = t1755*t6547;
  t6550 = t6544 + t6548;
  t6552 = t1755*t6538;
  t6571 = t129*t6547;
  t6581 = t6552 + t6571;
  t5852 = t49*t5851;
  t5857 = t5742 + t5852;
  t6607 = t129*t6606;
  t6616 = t6607 + t6615;
  t6621 = t1755*t6606;
  t6623 = t6621 + t6622;
  t6645 = -1.*t1755*t6639;
  t6653 = t6645 + t6622;
  t7080 = t49*t6642;
  t6661 = t1755*t6639;
  t6663 = t129*t6614;
  t6665 = t6661 + t6663;
  t6698 = -1.*t1755*t169*t6697;
  t6699 = -1.*t1219*t129*t6697;
  t6701 = t6698 + t6699;
  t6709 = t1219*t1755*t6697;
  t6714 = -1.*t169*t129*t6697;
  t6716 = t6709 + t6714;
  t7110 = t2043*t6813;
  t7115 = t49*t6827;
  t7116 = t7110 + t7115;
  t6844 = t129*t6843;
  t6858 = t6844 + t6856;
  t6860 = t1755*t6843;
  t6863 = t6860 + t6861;
  t6883 = -1.*t1755*t6870;
  t6886 = t6883 + t6861;
  t7129 = t49*t6880;
  t6894 = t1755*t6870;
  t6895 = t129*t6855;
  t6898 = t6894 + t6895;
  t6937 = -1.*t129*t6933;
  t6942 = t1755*t6941;
  t6944 = t6937 + t6942;
  t6947 = t1755*t6933;
  t6949 = t129*t6941;
  t6950 = t6947 + t6949;
  p_output1[0]=(-1.*t49*t6221 + t2043*t6227)*var2[3] + (t2043*t2116 - 1.*t1976*t49)*var2[4] + (t2043*t4522 - 1.*t4342*t49)*var2[5] + (-1.*t49*t5209 + t2043*t5285)*var2[13] + (-1.*t49*t4918 + t2043*t5016)*var2[14] + (t2043*t5565 - 1.*t49*t5649)*var2[15] + (t5742 - 1.*t49*t5776)*var2[16] + t5857*var2[17];
  p_output1[1]=(-1.*t49*t5686 + t2043*t5851)*var2[3] + (-1.*t49*t6287 + t2043*t6300)*var2[4] + (-1.*t49*t6372 + t2043*t6386)*var2[5] + (-1.*t49*t6550 + t2043*t6581)*var2[13] + (-1.*t49*t6456 + t2043*t6498)*var2[14] + (t2043*t6616 - 1.*t49*t6623)*var2[15] + (t6644 - 1.*t49*t6653)*var2[16] + (t6644 + t49*t6665)*var2[17];
  p_output1[2]=(-1.*t49*t6944 + t2043*t6950)*var2[4] + t6829*var2[5] + t6829*var2[13] + (-1.*t49*t6701 + t2043*t6716)*var2[14] + (t2043*t6858 - 1.*t49*t6863)*var2[15] + (t6882 - 1.*t49*t6886)*var2[16] + (t6882 + t49*t6898)*var2[17];
  p_output1[3]=(t2043*t6221 + t49*t6227)*var2[3] + (t1976*t2043 + t2116*t49)*var2[4] + (t2043*t4342 + t4522*t49)*var2[5] + (t2043*t5209 + t49*t5285)*var2[13] + (t2043*t4918 + t49*t5016)*var2[14] + (t49*t5565 + t2043*t5649)*var2[15] + (t2043*t5776 + t7003)*var2[16] + (-1.*t2043*t5851 + t7003)*var2[17];
  p_output1[4]=t5857*var2[3] + (t2043*t6287 + t49*t6300)*var2[4] + (t2043*t6372 + t49*t6386)*var2[5] + (t2043*t6550 + t49*t6581)*var2[13] + (t2043*t6456 + t49*t6498)*var2[14] + (t49*t6616 + t2043*t6623)*var2[15] + (t2043*t6653 + t7080)*var2[16] + (-1.*t2043*t6665 + t7080)*var2[17];
  p_output1[5]=(t2043*t6944 + t49*t6950)*var2[4] + t7116*var2[5] + t7116*var2[13] + (t2043*t6701 + t49*t6716)*var2[14] + (t49*t6858 + t2043*t6863)*var2[15] + (t2043*t6886 + t7129)*var2[16] + (-1.*t2043*t6898 + t7129)*var2[17];
  p_output1[6]=(-1.*t1248*t6094 + t6401)*var2[3] + (-1.*t1248*t1536 - 1.*t1308*t1351*t491)*var2[4] - 1.*t1248*t3830*var2[5] - 1.*t1248*t5103*var2[13] + t5391*var2[14];
  p_output1[7]=(-1.*t1248*t4725 + t1351*t491*t670)*var2[3] + (-1.*t1308*t1351*t2450 - 1.*t1248*t6260)*var2[4] - 1.*t1248*t6352*var2[5] - 1.*t1248*t6529*var2[13] + t6601*var2[14];
  p_output1[8]=(-1.*t1351*t670 - 1.*t1248*t6923)*var2[4] - 1.*t1248*t6746*var2[5] - 1.*t1248*t6746*var2[13] + t6840*var2[14];
}



void dR_RightShin_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
