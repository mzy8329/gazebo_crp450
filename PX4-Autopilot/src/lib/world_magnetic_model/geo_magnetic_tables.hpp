/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>

static constexpr float SAMPLING_RES = 10;
static constexpr float SAMPLING_MIN_LAT = -90;
static constexpr float SAMPLING_MAX_LAT = 90;
static constexpr float SAMPLING_MIN_LON = -180;
static constexpr float SAMPLING_MAX_LON = 180;

static constexpr int LAT_DIM = 19;
static constexpr int LON_DIM = 37;


// *INDENT-OFF*
// Magnetic declination data in radians * 10^-4
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2022.8137,
static constexpr const int16_t declination_table[19][37] {
	//    LONGITUDE:   -180,  -170,  -160,  -150,  -140,  -130,  -120,  -110,  -100,   -90,   -80,   -70,   -60,   -50,   -40,   -30,   -20,   -10,     0,    10,    20,    30,    40,    50,    60,    70,    80,    90,   100,   110,   120,   130,   140,   150,   160,   170,   180,
	/* LAT: -90 */ {  25980, 24235, 22489, 20744, 18999, 17253, 15508, 13763, 12017, 10272,  8527,  6782,  5036,  3291,  1546,  -200, -1945, -3690, -5435, -7181, -8926,-10671,-12417,-14162,-15907,-17653,-19398,-21143,-22889,-24634,-26379,-28125,-29870, 31216, 29471, 27726, 25980, },
	/* LAT: -80 */ {  22546, 20415, 18476, 16702, 15061, 13523, 12060, 10648,  9273,  7923,  6591,  5273,  3964,  2659,  1351,    32, -1309, -2682, -4092, -5544, -7039, -8576,-10154,-11773,-13438,-15156,-16942,-18815,-20798,-22916,-25191,-27625,-30194, 29999, 27381, 24876, 22546, },
	/* LAT: -70 */ {  14978, 13580, 12453, 11491, 10622,  9790,  8948,  8060,  7108,  6088,  5018,  3925,  2840,  1786,   764,  -246, -1283, -2389, -3591, -4892, -6273, -7704, -9152,-10595,-12021,-13438,-14867,-16353,-17977,-19893,-22416,-26218, 30695, 24136, 19622, 16850, 14978, },
	/* LAT: -60 */ {   8423,  8177,  7895,  7621,  7367,  7113,  6804,  6371,  5756,  4936,  3935,  2824,  1704,   677,  -203,  -960, -1688, -2512, -3517, -4716, -6048, -7419, -8741, -9952,-11016,-11915,-12630,-13125,-13297,-12848,-10746, -3486,  4925,  7665,  8433,  8558,  8423, },
	/* LAT: -50 */ {   5487,  5523,  5467,  5377,  5303,  5266,  5231,  5104,  4760,  4096,  3085,  1813,   472,  -706, -1574, -2136, -2531, -2972, -3659, -4677, -5925, -7204, -8346, -9247, -9844,-10082, -9886, -9127, -7616, -5254, -2348,   400,  2510,  3934,  4809,  5283,  5487, },
	/* LAT: -40 */ {   3956,  4049,  4056,  4011,  3951,  3917,  3921,  3910,  3737,  3202,  2178,   735,  -828, -2140, -3000, -3448, -3629, -3687, -3858, -4434, -5418, -6498, -7383, -7919, -8024, -7643, -6750, -5365, -3654, -1954,  -496,   725,  1769,  2636,  3293,  3724,  3956, },
	/* LAT: -30 */ {   2984,  3071,  3101,  3086,  3027,  2948,  2888,  2854,  2728,  2249,  1212,  -309, -1916, -3167, -3903, -4245, -4321, -4107, -3673, -3458, -3831, -4584, -5281, -5627, -5498, -4908, -3946, -2740, -1535,  -594,    82,   682,  1306,  1911,  2422,  2785,  2984, },
	/* LAT: -20 */ {   2342,  2388,  2405,  2406,  2364,  2269,  2161,  2085,  1939,  1443,   385, -1106, -2584, -3642, -4171, -4283, -4077, -3515, -2652, -1860, -1592, -1952, -2602, -3068, -3101, -2740, -2103, -1288,  -515,   -29,   231,   532,   974,  1454,  1879,  2188,  2342, },
	/* LAT: -10 */ {   1948,  1942,  1920,  1917,  1888,  1803,  1692,  1601,  1417,   866,  -198, -1576, -2848, -3674, -3937, -3705, -3128, -2345, -1500,  -747,  -286,  -323,  -796, -1301, -1511, -1420, -1106,  -608,  -113,   125,   171,   330,   702,  1143,  1538,  1828,  1948, },
	/* LAT:   0 */ {   1735,  1702,  1645,  1636,  1624,  1555,  1449,  1334,  1081,   460,  -581, -1808, -2861, -3444, -3438, -2936, -2167, -1386,  -735,  -200,   212,   321,    40,  -377,  -631,  -686,  -587,  -327,   -38,    48,   -15,    72,   414,   860,  1282,  1605,  1735, },
	/* LAT:  10 */ {   1599,  1606,  1564,  1579,  1605,  1555,  1432,  1245,   870,   152,  -868, -1944, -2768, -3105, -2898, -2290, -1514,  -803,  -289,    91,   416,   564,   400,    78,  -156,  -264,  -288,  -209,  -107,  -150,  -292,  -266,    37,   494,   978,  1387,  1599, },
	/* LAT:  20 */ {   1413,  1562,  1622,  1714,  1800,  1779,  1624,  1321,   777,   -78, -1120, -2070, -2665, -2772, -2443, -1840, -1130,  -481,   -21,   288,   546,   691,   599,   356,   157,    38,   -55,  -122,  -204,  -395,  -634,  -693,  -457,    -8,   535,  1055,  1413, },
	/* LAT:  30 */ {   1109,  1476,  1736,  1960,  2120,  2131,  1943,  1521,   794,  -234, -1349, -2224, -2638, -2575, -2175, -1596,  -941,  -323,   139,   444,   675,   821,   800,   651,   501,   376,   217,     3,  -282,  -658, -1028, -1187, -1026,  -599,   -22,   588,  1109, },
	/* LAT:  40 */ {    749,  1335,  1830,  2223,  2473,  2517,  2302,  1769,   859,  -366, -1601, -2466, -2786, -2634, -2188, -1596,  -942,  -313,   197,   562,   832,  1029,  1122,  1111,  1035,   886,   615,   200,  -341,  -949, -1470, -1714, -1596, -1179,  -581,    91,   749, },
	/* LAT:  50 */ {    456,  1202,  1883,  2442,  2810,  2918,  2691,  2038,   900,  -593, -2012, -2925, -3219, -3028, -2539, -1893, -1183,  -485,   131,   637,  1055,  1409,  1689,  1858,  1877,  1688,  1239,   527,  -370, -1274, -1951, -2236, -2111, -1670, -1037,  -307,   456, },
	/* LAT:  60 */ {    255,  1106,  1913,  2610,  3117,  3332,  3122,  2315,   808, -1144, -2856, -3836, -4090, -3831, -3260, -2515, -1689,  -848,   -40,   711,  1403,  2033,  2576,  2970,  3128,  2939,  2295,  1173,  -245, -1565, -2431, -2738, -2573, -2080, -1390,  -592,   255, },
	/* LAT:  70 */ {     11,   945,  1842,  2642,  3260,  3558,  3313,  2193,   -20, -2713, -4662, -5494, -5509, -5030, -4262, -3330, -2306, -1241,  -167,   893,  1918,  2884,  3752,  4458,  4904,  4933,  4318,  2836,   635, -1475, -2771, -3214, -3055, -2526, -1780,  -913,    11, },
	/* LAT:  80 */ {   -726,   195,  1047,  1739,  2126,  1954,   793, -1737, -4876, -6994, -7767, -7659, -7045, -6138, -5054, -3860, -2597, -1294,    30,  1357,  2671,  3953,  5181,  6320,  7317,  8079,  8423,  7976,  6029,  2205, -1425, -3142, -3503, -3172, -2496, -1647,  -726, },
	/* LAT:  90 */ { -29738,-27992,-26247,-24501,-22756,-21011,-19265,-17520,-15775,-14030,-12284,-10539, -8794, -7049, -5304, -3558, -1813,   -68,  1677,  3422,  5167,  6913,  8658, 10403, 12149, 13894, 15639, 17385, 19130, 20876, 22621, 24367, 26112, 27858, 29603, 31349,-29738, },
};

// Magnetic inclination data in radians * 10^-4
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2022.8137,
static constexpr const int16_t inclination_table[19][37] {
	//    LONGITUDE:   -180,  -170,  -160,  -150,  -140,  -130,  -120,  -110,  -100,   -90,   -80,   -70,   -60,   -50,   -40,   -30,   -20,   -10,     0,    10,    20,    30,    40,    50,    60,    70,    80,    90,   100,   110,   120,   130,   140,   150,   160,   170,   180,
	/* LAT: -90 */ { -12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570,-12570, },
	/* LAT: -80 */ { -13654,-13520,-13360,-13179,-12986,-12784,-12580,-12380,-12189,-12013,-11856,-11722,-11612,-11526,-11462,-11421,-11401,-11403,-11430,-11485,-11568,-11683,-11828,-12001,-12199,-12416,-12646,-12879,-13107,-13319,-13506,-13656,-13759,-13810,-13807,-13753,-13654, },
	/* LAT: -70 */ { -14102,-13784,-13464,-13141,-12810,-12466,-12111,-11754,-11411,-11103,-10850,-10667,-10555,-10502,-10488,-10491,-10498,-10507,-10530,-10585,-10694,-10868,-11115,-11430,-11803,-12221,-12669,-13133,-13599,-14052,-14469,-14814,-15002,-14947,-14716,-14418,-14102, },
	/* LAT: -60 */ { -13516,-13163,-12825,-12492,-12148,-11776,-11361,-10907,-10439,-10009, -9680, -9505, -9502, -9639, -9843,-10035,-10159,-10201,-10187,-10172,-10219,-10379,-10669,-11078,-11577,-12134,-12720,-13316,-13905,-14466,-14963,-15253,-15075,-14690,-14283,-13890,-13516, },
	/* LAT: -50 */ { -12495,-12153,-11822,-11499,-11176,-10830,-10430, -9959, -9430, -8909, -8520, -8396, -8598, -9059, -9623,-10133,-10491,-10651,-10619,-10465,-10323,-10330,-10552,-10970,-11513,-12106,-12693,-13231,-13677,-13974,-14082,-14009,-13805,-13522,-13193,-12845,-12495, },
	/* LAT: -40 */ { -11239,-10891,-10543,-10198, -9858, -9520, -9160, -8735, -8214, -7650, -7225, -7186, -7648, -8473, -9394,-10222,-10878,-11310,-11447,-11276,-10932,-10661,-10657,-10941,-11404,-11909,-12354,-12675,-12833,-12838,-12748,-12612,-12433,-12201,-11914,-11586,-11239, },
	/* LAT: -30 */ {  -9602, -9222, -8843, -8452, -8060, -7685, -7329, -6938, -6426, -5815, -5365, -5452, -6226, -7431, -8681, -9775,-10686,-11393,-11788,-11774,-11399,-10891,-10560,-10563,-10816,-11138,-11393,-11504,-11441,-11265,-11092,-10959,-10814,-10608,-10326, -9980, -9602, },
	/* LAT: -20 */ {  -7372, -6930, -6512, -6083, -5639, -5209, -4819, -4407, -3842, -3157, -2712, -2980, -4097, -5713, -7340, -8706, -9768,-10539,-10973,-11003,-10635,-10020, -9463, -9221, -9272, -9434, -9574, -9591, -9417, -9146, -8953, -8869, -8767, -8561, -8244, -7832, -7372, },
	/* LAT: -10 */ {  -4416, -3878, -3423, -2985, -2528, -2078, -1665, -1212,  -588,   114,   472,    37, -1301, -3223, -5196, -6799, -7888, -8517, -8777, -8710, -8296, -7609, -6946, -6603, -6565, -6658, -6774, -6797, -6608, -6313, -6161, -6175, -6138, -5924, -5540, -5012, -4416, },
	/* LAT:   0 */ {   -908,  -282,   185,   588,  1007,  1423,  1812,  2253,  2835,  3410,  3608,  3104,  1801,  -115, -2157, -3800, -4800, -5222, -5277, -5103, -4658, -3939, -3234, -2865, -2805, -2874, -2999, -3071, -2941, -2707, -2655, -2804, -2872, -2688, -2264, -1632,  -908, },
	/* LAT:  10 */ {   2560,  3188,  3624,  3965,  4321,  4688,  5038,  5423,  5872,  6240,  6272,  5778,  4689,  3120,  1430,    61,  -736,  -977,  -882,  -644,  -229,   412,  1045,  1381,  1444,  1401,  1301,  1210,  1252,  1354,  1275,  1003,   806,   876,  1229,  1832,  2560, },
	/* LAT:  20 */ {   5415,  5945,  6325,  6621,  6934,  7278,  7620,  7965,  8296,  8494,  8404,  7938,  7099,  6001,  4877,  3972,  3449,  3335,  3488,  3733,  4065,  4533,  4993,  5247,  5305,  5289,  5241,  5184,  5171,  5152,  4975,  4639,  4338,  4247,  4419,  4843,  5415, },
	/* LAT:  30 */ {   7568,  7942,  8260,  8543,  8852,  9199,  9555,  9895, 10170, 10282, 10136,  9712,  9082,  8371,  7711,  7202,  6914,  6875,  7019,  7232,  7480,  7780,  8069,  8242,  8300,  8314,  8317,  8308,  8284,  8201,  7977,  7622,  7265,  7044,  7028,  7223,  7568, },
	/* LAT:  40 */ {   9266,  9487,  9743, 10028, 10354, 10715, 11082, 11420, 11672, 11752, 11602, 11241, 10768, 10295,  9900,  9618,  9471,  9469,  9580,  9743,  9919, 10104, 10276, 10401, 10479, 10540, 10596, 10629, 10612, 10501, 10258,  9906,  9538,  9253,  9110,  9123,  9266, },
	/* LAT:  50 */ {  10802, 10923, 11124, 11393, 11716, 12070, 12424, 12740, 12961, 13019, 12880, 12582, 12218, 11873, 11599, 11413, 11321, 11319, 11386, 11490, 11604, 11720, 11837, 11953, 12071, 12194, 12308, 12380, 12371, 12245, 11998, 11670, 11330, 11045, 10855, 10775, 10802, },
	/* LAT:  60 */ {  12319, 12392, 12541, 12759, 13029, 13329, 13631, 13896, 14072, 14100, 13964, 13716, 13427, 13157, 12936, 12781, 12690, 12660, 12676, 12724, 12792, 12878, 12984, 13116, 13276, 13452, 13617, 13725, 13730, 13610, 13385, 13106, 12827, 12590, 12420, 12329, 12319, },
	/* LAT:  70 */ {  13758, 13800, 13894, 14036, 14215, 14418, 14626, 14807, 14911, 14893, 14757, 14556, 14339, 14135, 13962, 13827, 13733, 13679, 13660, 13673, 13715, 13787, 13891, 14028, 14195, 14382, 14566, 14706, 14752, 14679, 14518, 14319, 14125, 13960, 13840, 13771, 13758, },
	/* LAT:  80 */ {  14996, 15008, 15045, 15104, 15180, 15265, 15343, 15388, 15372, 15295, 15183, 15060, 14937, 14824, 14724, 14642, 14580, 14540, 14521, 14525, 14552, 14601, 14673, 14766, 14877, 15004, 15140, 15272, 15379, 15423, 15384, 15296, 15201, 15116, 15051, 15010, 14996, },
	/* LAT:  90 */ {  15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, 15396, },
};

// Magnetic strength data in milli-Gauss * 10
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2022.8137,
static constexpr const int16_t strength_table[19][37] {
	//    LONGITUDE:  -180, -170, -160, -150, -140, -130, -120, -110, -100,  -90,  -80,  -70,  -60,  -50,  -40,  -30,  -20,  -10,    0,   10,   20,   30,   40,   50,   60,   70,   80,   90,  100,  110,  120,  130,  140,  150,  160,  170,  180,
	/* LAT: -90 */ {  5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, 5450, },
	/* LAT: -80 */ {  6056, 5993, 5914, 5821, 5718, 5607, 5489, 5367, 5244, 5125, 5010, 4905, 4811, 4732, 4669, 4624, 4600, 4598, 4619, 4665, 4736, 4830, 4945, 5077, 5220, 5370, 5520, 5663, 5794, 5908, 6001, 6069, 6114, 6133, 6129, 6102, 6056, },
	/* LAT: -70 */ {  6301, 6167, 6017, 5852, 5673, 5481, 5276, 5061, 4842, 4627, 4424, 4242, 4084, 3953, 3849, 3773, 3726, 3713, 3742, 3818, 3946, 4127, 4356, 4624, 4919, 5226, 5530, 5814, 6063, 6267, 6418, 6514, 6555, 6547, 6497, 6413, 6301, },
	/* LAT: -60 */ {  6186, 5994, 5792, 5583, 5364, 5129, 4873, 4594, 4302, 4013, 3746, 3519, 3339, 3205, 3106, 3032, 2982, 2962, 2988, 3079, 3248, 3499, 3827, 4213, 4635, 5070, 5491, 5875, 6199, 6448, 6611, 6690, 6691, 6628, 6514, 6363, 6186, },
	/* LAT: -50 */ {  5843, 5613, 5381, 5150, 4917, 4672, 4402, 4099, 3770, 3438, 3136, 2897, 2735, 2644, 2596, 2562, 2528, 2501, 2506, 2579, 2753, 3044, 3440, 3912, 4419, 4925, 5400, 5819, 6157, 6399, 6537, 6577, 6534, 6424, 6262, 6064, 5843, },
	/* LAT: -40 */ {  5393, 5147, 4902, 4662, 4427, 4190, 3935, 3651, 3337, 3011, 2713, 2491, 2374, 2348, 2367, 2388, 2393, 2380, 2367, 2396, 2527, 2804, 3222, 3737, 4284, 4808, 5274, 5662, 5954, 6142, 6230, 6232, 6163, 6032, 5850, 5632, 5393, },
	/* LAT: -30 */ {  4878, 4637, 4398, 4163, 3937, 3718, 3499, 3268, 3012, 2737, 2479, 2297, 2227, 2252, 2319, 2390, 2455, 2505, 2526, 2539, 2606, 2806, 3170, 3659, 4190, 4687, 5105, 5421, 5624, 5723, 5750, 5723, 5643, 5511, 5331, 5115, 4878, },
	/* LAT: -20 */ {  4321, 4108, 3899, 3694, 3498, 3315, 3147, 2983, 2806, 2607, 2417, 2283, 2242, 2286, 2375, 2486, 2614, 2742, 2830, 2865, 2890, 2987, 3228, 3613, 4064, 4490, 4838, 5072, 5177, 5185, 5154, 5107, 5025, 4898, 4731, 4534, 4321, },
	/* LAT: -10 */ {  3790, 3629, 3476, 3329, 3194, 3074, 2970, 2878, 2781, 2666, 2544, 2445, 2400, 2424, 2511, 2640, 2796, 2954, 3077, 3139, 3154, 3181, 3305, 3558, 3884, 4206, 4470, 4632, 4667, 4615, 4547, 4484, 4396, 4271, 4122, 3958, 3790, },
	/* LAT:   0 */ {  3412, 3319, 3235, 3162, 3107, 3068, 3042, 3023, 2999, 2951, 2872, 2777, 2697, 2666, 2709, 2812, 2944, 3079, 3194, 3269, 3300, 3322, 3397, 3555, 3764, 3978, 4158, 4263, 4269, 4202, 4113, 4021, 3910, 3778, 3644, 3520, 3412, },
	/* LAT:  10 */ {  3283, 3251, 3231, 3227, 3252, 3299, 3354, 3407, 3442, 3432, 3363, 3249, 3122, 3028, 3002, 3043, 3124, 3223, 3323, 3408, 3472, 3534, 3622, 3741, 3877, 4015, 4135, 4205, 4207, 4145, 4035, 3892, 3730, 3571, 3435, 3339, 3283, },
	/* LAT:  20 */ {  3399, 3402, 3428, 3482, 3574, 3695, 3823, 3940, 4021, 4033, 3960, 3818, 3651, 3512, 3436, 3424, 3460, 3533, 3629, 3726, 3817, 3916, 4027, 4138, 4247, 4359, 4461, 4527, 4538, 4479, 4342, 4139, 3912, 3702, 3539, 3438, 3399, },
	/* LAT:  30 */ {  3723, 3729, 3783, 3882, 4025, 4196, 4372, 4528, 4636, 4662, 4589, 4434, 4245, 4081, 3976, 3931, 3934, 3985, 4073, 4171, 4268, 4374, 4491, 4608, 4728, 4855, 4976, 5062, 5088, 5030, 4874, 4634, 4360, 4106, 3906, 3778, 3723, },
	/* LAT:  40 */ {  4222, 4220, 4284, 4408, 4574, 4762, 4945, 5101, 5206, 5232, 5164, 5015, 4829, 4655, 4527, 4452, 4426, 4449, 4509, 4589, 4675, 4773, 4887, 5021, 5172, 5334, 5485, 5595, 5634, 5580, 5424, 5186, 4912, 4653, 4442, 4296, 4222, },
	/* LAT:  50 */ {  4832, 4824, 4879, 4989, 5135, 5294, 5444, 5565, 5639, 5648, 5585, 5458, 5296, 5132, 4995, 4898, 4843, 4832, 4857, 4907, 4976, 5067, 5186, 5337, 5512, 5696, 5863, 5980, 6024, 5980, 5849, 5653, 5428, 5213, 5033, 4903, 4832, },
	/* LAT:  60 */ {  5392, 5379, 5407, 5469, 5554, 5648, 5736, 5802, 5836, 5827, 5773, 5679, 5559, 5431, 5312, 5216, 5150, 5116, 5114, 5142, 5198, 5285, 5403, 5550, 5717, 5885, 6032, 6136, 6179, 6157, 6074, 5948, 5803, 5660, 5538, 5446, 5392, },
	/* LAT:  70 */ {  5726, 5706, 5702, 5714, 5736, 5762, 5786, 5800, 5800, 5782, 5743, 5686, 5616, 5540, 5466, 5402, 5354, 5326, 5322, 5342, 5387, 5456, 5548, 5656, 5773, 5888, 5987, 6059, 6099, 6102, 6073, 6020, 5953, 5882, 5817, 5764, 5726, },
	/* LAT:  80 */ {  5789, 5772, 5757, 5745, 5735, 5726, 5716, 5704, 5689, 5671, 5649, 5624, 5597, 5570, 5545, 5525, 5512, 5507, 5511, 5527, 5552, 5588, 5630, 5678, 5728, 5776, 5819, 5853, 5877, 5890, 5892, 5885, 5871, 5852, 5831, 5809, 5789, },
	/* LAT:  90 */ {  5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, 5682, },
};