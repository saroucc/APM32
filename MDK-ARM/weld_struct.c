#include "weld_struct.h"



const uint16_t GMAW_GAS_Speed[]={

/*****CO2-1.0-SOLID****D*/
//	481,520,540,590,628, 669,710,760,800,850,
//	892,970,1020,1120,1160, 1216,1270,1330,1440,1500,
//	1560,1670,1750,1820,1910, 1996,2090,2180,2270,2360,
//	2448,2500,2550,2570,2595, 2628,2628,2628,2628,2628,
//	2628,2628,2628,2628,2628, 2628,2628,2628,2628,2628
/*****CO2-1.2-SOLID****D*/
	409,433,455,479,496, 515,543,580,614,641,
	691,745,794,848,905, 968,1000,1022,1050,1073,
	1154,1190,1250,1311,1367, 1421,1493,1573,1644,1719,
	1833,1901,1954,2035,2113, 2197,2197,2197,2197,2197,
	2197,2197,2197,2197,2197, 2197,2197,2197,2197,2197
/*****CO2-1.6-SOLID****D*/
//	409,413,417,421,425, 429,439,448,457,466,
//	471,490,505,522,538, 563,583,605,625,648,
//	667,690,712,733,755, 779,800,822,842,865,
//	892,920,950,975,1000, 1020,1050,1080,1110,1140,
//	1176,1220,1260,1300,1335, 1363,1390,1490,1550,1650

/*****MAG-1.0-SOLID****D*/
//	534,560,585,610,638, 675,720,760,800,850,
//	892,950,1010,1060,1120, 1188,1270,1320,1400,1460,
//	1518,1600,1680,1750,1820, 1906,1990,2080,2180,2260,
//	2330,2500,2550,2570,2595, 2628,2628,2628,2628,2628,
//	2628,2628,2628,2628,2628, 2628,2628,2628,2628,2628
/*****MAG-1.2-SOLID****D*/
//	390,409,433,455,479, 496,515,543,580,614,
//	641,691,745,794,848, 880,920,960,1000,1030,
//	1040,1060,1154,1190,1250, 1311,1357,1400,1425,1450,
//	1493,1573,1630,1700,1750, 1810,1870,1950,2115,2197,
//	2221,2257,2307,2357,2407, 2457,2507,2557,2607,2657
/*****MAG-1.6-SOLID****D*/
//	400,409,413,417,421, 425,429,439,448,457,
//	466,471,490,505,522, 538,563,583,605,625,
//	648,667,690,712,733, 755,779,800,822,842,
//	865,892,920,940,950, 975,1000,1020,1050,1080,
//	1110,1140,1176,1220,1260, 1300,1335,1363,1390,1490

/*****CO2-1.2-FLUX****D*/
//	526,560,590,625,662, 698,736,772,800,840,
//	885,940,1000,1055,1110, 1166,1230,1290,1350,1420,
//	1476,1525,1600,1655,1738, 1813,1910,2010,2120,2230,
//	2340,2400,2460,2520,2565, 2628,2628,2628,2628,2628,
//	2628,2628,2628,2628,2628, 2628,2628,2628,2628,2628
/*****CO2-1.6-FLUX****W*/
//	412,423,435,449,466, 476,490,505,520,532,
//	547,567,587,605,626, 646,675,702,730,758,
//	787,815,845,875,905, 936,966,1000,1030,1070,
//	1110,1150,1190,1230,1270, 1313,1360,1408,1455,1502,
//	1553,1600,1650,1700,1755, 1819,1919,2019,2217,2327
};

const uint16_t GMAW_GAS_Volt[]={

/*****CO2-1.0-SOLID****D*/	
//	1067,1087,1105,1115,1125, 1133,1142,1158,1175,1198,
//	1228,1268,1318,1426,1555, 1637,1650,1680,1710,1730,
//	1750,1760,1770,1780,1800, 1820,1800,1790,1780,1800,
//	1857,1907,2000,2080,2150, 2187,2227,2267,2310,2360,
//	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
/*****CO2-1.2-SOLID****D*/	
	1120,1147,1173,1200,1233, 1250,1260,1270,1280,1300,
	1330,1370,1386,1426,1466, 1500,1580,1620,1680,1750,
	1800,1950,2100,2100,2050, 2020,1970,1970,1970,1970,
	1980,2000,2067,2120,2147, 2187,2227,2227,2300,2360,
	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
/*****CO2-1.6-SOLID****D*/	
//	1250,1255,1260,1265,1267, 1270,1273,1277,1280,1285,
//	1290,1306,1326,1336,1350, 1370,1390,1410,1425,1440,
//	1460,1490,1510,1550,1580, 1610,1650,1660,1650,1660,
//	1670,1680,1767,1820,1857, 1957,2007,2067,2110,2160,
//	2200,2260,2290,2310,2330, 2340,2350,2360,2370,2380

/*****MAG-1.0-SOLID****D*/	
//	1000,1010,1138,1155,1160, 1070,1080,1090,1125,1165,
//	1188,1220,1245,1268,1295, 1340,1370,1450,1490,1500,
//	1510,1520,1530,1550,1590, 1640,1660,1680,1710,1750,
//	1850,2017,2067,2110,2147, 2187,2227,2267,2310,2360,
//	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
/*****MAG-1.2-SOLID****D*/	
//	927,945,953,980,1020, 1050,1060,1080,1100,1120,
//	1130,1150,1170,1190,1210, 1230,1270,1290,1310,1330,
//	1350,1420,1480,1500,1530, 1587,1670,1730,1790,1850,
//	1920,2000,2067,2090,2150, 2200,2250,2290,2320,2360,
//	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
/*****MAG-1.6-SOLID****D*/	
//	1000,1020,1040,1060,1080, 1150,1155,1160,1162,1165,
//	1175,1178,1182,1188,1198, 1210,1225,1238,1265,1285,
//	1300,1320,1350,1358,1366, 1380,1410,1430,1450,1470,
//	1490,1510,1530,1575,1610, 1650,1720,1810,1910,1960,
//	1973,2010,2060,2110,2150, 2187,2350,2360,2370,2380

/*****CO2-1.2-FLUX****D*/	
//	1213,1230,1250,1270,1285, 1290,1300,1310,1330,1350,
//	1370,1420,1495,1542,1585, 1625,1660,1710,1770,1805,
//	1855,1858,1860,1862,1865, 1870,1875,1878,1885,1895,
//	1927,2007,2067,2110,2147, 2187,2227,2227,2300,2360,
//	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
/*****CO2-1.6-FLUX****W*/	
//	1213,1230,1250,1270,1285, 1290,1300,1310,1330,1350,
//	1370,1420,1495,1542,1585, 1625,1660,1710,1770,1805,
//	1855,1878,1889,1902,1915, 1916,1917,1918,1920,1925,
//  1930,2050,2080,2110,2147, 2187,2227,2227,2300,2360,
//	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
};

const uint8_t GMAW_GAS_Inductor[]={

/*****CO2-1.0-SOLID****D*/
//	40,40,40,40,40, 45,50,60,70,80,
//	90,100,110,120,130, 140,150,150,150,150,
//	150,150,150,150,150, 150,150,150,150,150,
//	150,150,150,160,160, 160,160,160,160,160,
//	180,180,180,180,180, 180,180,180,180,180
/*****CO2-1.2-SOLID****D*/
	40,40,40,50,50, 50,55,60,65,70,
	80,90,110,130,140, 150,150,150,150,150,
	150,150,150,150,150, 150,150,150,150,150,
	150,150,150,160,160, 160,160,160,160,160,
	180,180,180,180,180, 180,180,180,180,180
/*****CO2-1.6-SOLID****D*/
//	40,40,40,50,50, 50,55,60,65,70,
//	80,90,100,110,120, 130,140,150,150,150,
//	150,150,150,150,150, 150,150,150,150,150,
//	150,150,150,150,150, 150,150,150,150,150,
//	150,150,150,150,150, 150,150,150,150,150

/*****MAG-1.0-SOLID****D*/
//	30,30,30,30,30, 30,30,30,30,30,
//	30,30,30,31,32, 33,35,36,37,38,
//	39,41,44,50,55, 60,65,70,75,80,
//	80,80,80,80,80, 80,80,80,80,80,
//	80,80,80,80,80, 80,80,80,80,80
/*****MAG-1.2-SOLID****D*/
//	30,30,30,30,30, 30,30,30,30,30,
//	30,30,30,30,30, 30,30,30,35,38,
//	41,45,50,55,60, 70,80,80,80,80,
//	80,80,80,80,80, 80,80,80,80,80,
//	80,80,80,80,80, 80,80,80,80,80
/*****MAG-1.6-SOLID****D*/
//	30,30,30,30,30, 30,30,30,30,30,
//	30,30,30,30,30, 30,33,36,38,40,
//	42,43,45,47,50, 55,60,65,75,80,
//	80,80,80,80,80, 80,80,80,80,80,
//	80,80,80,80,80, 80,80,80,80,80

/*****CO2-1.2-FLUX****D*/
//	20,20,20,20,20, 20,20,20,20,22,
//	25,28,30,31,32, 33,35,36,37,38,
//	39,41,44,50,55, 60,65,70,75,80,
//	80,80,80,80,80, 80,80,80,80,80,
//	80,80,80,80,80, 80,80,80,80,80	
/*****CO2-1.6-FLUX****W*/
//	20,20,20,20,20, 20,20,20,20,20,
//  20,20,20,20,20, 20,20,20,20,22,
//	25,28,30,31,33, 35,37,39,41,43,
//	45,50,55,60,65, 70,80,80,80,80,
//	80,80,80,80,80, 80,80,80,80,80
};

const uint16_t GMAW_GAS_Vc[]={			//Vol_Plus 电压变化幅限

/*****CO2-1.0-SOLID****D*/	
//	100,100,100,120,150, 180,200,200,200,200,
//	200,190,180,170,160, 160,150,150,140,130,
//	120,110,100,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0
/*****CO2-1.2-SOLID****D*/	
	100,100,100,100,100, 100,100,100,120,150,
	180,200,220,260,250, 200,200,200,200,200,
	100,100,100,100,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
/*****CO2-1.6-SOLID****D*/	
//	100,100,100,100,100, 100,100,100,120,150,
//	180,200,220,260,260, 250,240,230,220,210,
//	200,200,200,200,200, 100,100,100,100,100,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0	

/*****MAG-1.0-SOLID****D*/	
//	100,100,100,120,150, 180,180,180,180,160,
//	150,120,110,100,100, 100,100,100,90,90,
//	80,80,50,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0
/*****MAG-1.2-SOLID****D*/	
//	100,100,100,100,100, 100,100,100,120,150,
//	180,200,220,220,220, 220,200,180,160,150,
//	140,130,120,110,100, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0
/*****MAG-1.6-SOLID****D*/	
//	100,100,100,100,100, 100,100,100,100,100,
//	100,100,100,120,150, 170,180,190,200,210, 
//	220,210,200,190,180, 160,140,120,110,100,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0	

/*****CO2-1.2-FLUX****D*/	
//	80,80,80,80,80, 80,80,80,80,80,
//	90,100,110,100,90, 80,70,60,50,50,
//  50,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0
/*****CO2-1.6-FLUX****W*/	
//	80,80,80,80,80, 80,80,80,80,80,
//	80,80,80,80,80, 90,100,110,100,90,
//  80,70,60,50,50, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0
};


const uint8_t GMAW_GAS_VSLOPE[]={		//Vol_plus 电压变化斜率

/*****CO2-1.0-SOLID****D*/	
//	16,16,16,16,16, 16,16,16,16,18,
//	18,18,18,16,16, 15,14,13,12,11,
//	10,8,5,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0
/*****CO2-1.2-SOLID****D*/	
	16,16,16,16,16, 16,16,16,16,18,
	18,16,16,12,10, 8,8,8,8,6,
	6,6,5,5,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
/*****CO2-1.6-SOLID****D*/	
//	16,16,16,16,16, 16,16,16,16,18,
//	18,16,16,15,14, 13,12,11,10,9,
//	8,8,8,8,8, 6,6,5,5,5,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0

/*****MAG-1.0-SOLID****D*/	
//	16,16,16,16,17, 18,18,17,17,16,
//	15,12,9,8,7, 7,7,7,7,6,
//	5,5,3,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0
/*****MAG-1.2-SOLID****D*/	
//	16,16,16,16,16, 16,16,16,16,18,
//	18,16,16,12,10, 9,9,8,8,7,
//	7,6,6,5,3, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0
/*****MAG-1.6-SOLID****D*/	
//	16,16,16,16,16, 16,16,16,16,18,
//	16,16,16,16,18, 18,16,16,15,14,
//	13,12,12,11,11, 10,10,10,8,5,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0

/*****CO2-1.2-FLUX****D*/	
//	8,8,8,7,6, 5,5,4,4,3,
//	3,3,3,3,3, 3,3,2,1,1,
//	1,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0
/*****CO2-1.6-FLUX****W*/	
//	8,8,8,7,6, 5,5,4,4,3,
//	5,5,4,4,3, 3,3,3,2,2,
//	2,1,1,1,1, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0,
//	0,0,0,0,0, 0,0,0,0,0
};


/*  CO2  1.2   */
const uint16_t GMAW_Wave_CO2_10_Speed[]={
	481,520,540,590,628, 669,710,760,800,850,
	892,970,1020,1120,1160, 1216,1270,1330,1440,1500,
	1560,1670,1750,1820,1910, 1996,2090,2180,2270,2360,
	2448,2500,2550,2570,2595, 2628,2628,2628,2628,2628,
	2628,2628,2628,2628,2628, 2628,2628,2628,2628,2628
};
const uint16_t GMAW_Wave_CO2_10_Volt[]={
	1067,1087,1105,1115,1125, 1133,1142,1158,1175,1198,
	1228,1268,1318,1426,1555, 1637,1650,1680,1710,1730,
	1750,1760,1770,1780,1800, 1820,1800,1790,1780,1800,
	1857,1907,2000,2080,2150, 2187,2227,2267,2310,2360,
	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
};
const uint8_t  GMAW_Wave_CO2_10_Inductor[]={
	40,40,40,40,40, 45,50,60,70,80,
	90,100,110,120,130, 140,150,150,150,150,
	150,150,150,150,150, 150,150,150,150,150,
	150,150,150,160,160, 160,160,160,160,160,
	180,180,180,180,180, 180,180,180,180,180
};
const uint16_t GMAW_Wave_CO2_10_Vc[]={
	100,100,100,120,150, 180,200,200,200,200,
	200,190,180,170,160, 160,150,150,140,130,
	120,110,100,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};
const uint8_t  GMAW_Wave_CO2_10_VSLOPE[]={
	16,16,16,16,16, 16,16,16,16,18,
	18,18,18,16,16, 15,14,13,12,11,
	10,8,5,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};

/* CO2 1.4  */
const uint16_t GMAW_Wave_CO2_12_Speed[]={
	409,433,455,479,496, 515,543,580,614,641,
	691,745,794,848,905, 968,1000,1022,1050,1073,
	1154,1190,1250,1311,1367, 1421,1493,1573,1644,1719,
	1833,1901,1954,2035,2113, 2197,2197,2197,2197,2197,
	2197,2197,2197,2197,2197, 2197,2197,2197,2197,2197
};
const uint16_t GMAW_Wave_CO2_12_Volt[]={
	1120,1147,1173,1200,1233, 1250,1260,1270,1280,1300,
	1330,1370,1386,1426,1466, 1500,1580,1620,1680,1750,
	1800,1950,2100,2100,2050, 2020,1970,1970,1970,1970,
	1980,2000,2067,2120,2147, 2187,2227,2227,2300,2360,
	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
};
const uint8_t  GMAW_Wave_CO2_12_Inductor[]={
	40,40,40,50,50, 50,55,60,65,70,
	80,90,110,130,140, 150,150,150,150,150,
	150,150,150,150,150, 150,150,150,150,150,
	150,150,150,160,160, 160,160,160,160,160,
	180,180,180,180,180, 180,180,180,180,180
};
const uint16_t GMAW_Wave_CO2_12_Vc[]={
	100,100,100,100,100, 100,100,100,120,150,
	180,200,220,260,250, 200,200,200,200,200,
	100,100,100,100,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};
const uint8_t  GMAW_Wave_CO2_12_VSLOPE[]={
	16,16,16,16,16, 16,16,16,16,18,
	18,16,16,12,10, 8,8,8,8,6,
	6,6,5,5,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};

/* CO2 1.6 */
const uint16_t GMAW_Wave_CO2_16_Speed[]={
	409,413,417,421,425, 429,439,448,457,466,
	471,490,505,522,538, 563,583,605,625,648,
	667,690,712,733,755, 779,800,822,842,865,
	892,920,950,975,1000, 1020,1050,1080,1110,1140,
	1176,1220,1260,1300,1335, 1363,1390,1490,1550,1650
};
const uint16_t GMAW_Wave_CO2_16_Volt[]={
	1250,1255,1260,1265,1267, 1270,1273,1277,1280,1285,
	1290,1306,1326,1336,1350, 1370,1390,1410,1425,1440,
	1460,1490,1510,1550,1580, 1610,1650,1660,1650,1660,
	1670,1680,1767,1820,1857, 1957,2007,2067,2110,2160,
	2200,2260,2290,2310,2330, 2340,2350,2360,2370,2380
};
const uint8_t  GMAW_Wave_CO2_16_Inductor[]={
	40,40,40,50,50, 50,55,60,65,70,
	80,90,100,110,120, 130,140,150,150,150,
	150,150,150,150,150, 150,150,150,150,150,
	150,150,150,150,150, 150,150,150,150,150,
	150,150,150,150,150, 150,150,150,150,150
};
const uint16_t GMAW_Wave_CO2_16_Vc[]={
	100,100,100,100,100, 100,100,100,120,150,
	180,200,220,260,260, 250,240,230,220,210,
	200,200,200,200,200, 100,100,100,100,100,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0	
};
const uint8_t  GMAW_Wave_CO2_16_VSLOPE[]={
	16,16,16,16,16, 16,16,16,16,18,
	18,16,16,15,14, 13,12,11,10,9,
	8,8,8,8,8, 6,6,5,5,5,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};

/*	MAG 1.2	*/
const uint16_t GMAW_Wave_MAG_10_Speed[]={
	534,560,585,610,638, 675,720,760,800,850,
	892,950,1010,1060,1120, 1188,1270,1320,1400,1460,
	1518,1600,1680,1750,1820, 1906,1990,2080,2180,2260,
	2330,2500,2550,2570,2595, 2628,2628,2628,2628,2628,
	2628,2628,2628,2628,2628, 2628,2628,2628,2628,2628
};
const uint16_t GMAW_Wave_MAG_10_Volt[]={
	1000,1010,1138,1155,1160, 1070,1080,1090,1125,1165,
	1188,1220,1245,1268,1295, 1340,1370,1450,1490,1500,
	1510,1520,1530,1550,1590, 1640,1660,1680,1710,1750,
	1850,2017,2067,2110,2147, 2187,2227,2267,2310,2360,
	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
};
const uint8_t  GMAW_Wave_MAG_10_Inductor[]={
	30,30,30,30,30, 30,30,30,30,30,
	30,30,30,31,32, 33,35,36,37,38,
	39,41,44,50,55, 60,65,70,75,80,
	80,80,80,80,80, 80,80,80,80,80,
	80,80,80,80,80, 80,80,80,80,80
};
const uint16_t GMAW_Wave_MAG_10_Vc[]={
	100,100,100,120,150, 180,180,180,180,160,
	150,120,110,100,100, 100,100,100,90,90,
	80,80,50,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};
const uint8_t  GMAW_Wave_MAG_10_VSLOPE[]={
	16,16,16,16,17, 18,18,17,17,16,
	15,12,9,8,7, 7,7,7,7,6,
	5,5,3,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};

/* MAG	1.4 */
const uint16_t GMAW_Wave_MAG_12_Speed[]={
	390,409,433,455,479, 496,515,543,580,614,
	641,691,745,794,848, 880,920,960,1000,1030,
	1040,1060,1154,1190,1250, 1311,1357,1400,1425,1450,
	1493,1573,1630,1700,1750, 1810,1870,1950,2115,2197,
	2221,2257,2307,2357,2407, 2457,2507,2557,2607,2657
};
const uint16_t GMAW_Wave_MAG_12_Volt[]={
	927,945,953,980,1020, 1050,1060,1080,1100,1120,
	1130,1150,1170,1190,1210, 1230,1270,1290,1310,1330,
	1350,1420,1480,1500,1530, 1587,1670,1730,1790,1850,
	1920,2000,2067,2090,2150, 2200,2250,2290,2320,2360,
	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
};
const uint8_t  GMAW_Wave_MAG_12_Inductor[]={
	30,30,30,30,30, 30,30,30,30,30,
	30,30,30,30,30, 30,30,30,35,38,
	41,45,50,55,60, 70,80,80,80,80,
	80,80,80,80,80, 80,80,80,80,80,
	80,80,80,80,80, 80,80,80,80,80
};
const uint16_t GMAW_Wave_MAG_12_Vc[]={
	100,100,100,100,100, 100,100,100,120,150,
	180,200,220,220,220, 220,200,180,160,150,
	140,130,120,110,100, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};
const uint8_t  GMAW_Wave_MAG_12_VSLOPE[]={
	16,16,16,16,16, 16,16,16,16,18,
	18,16,16,12,10, 9,9,8,8,7,
	7,6,6,5,3, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};

/* MAG 1.6 */
const uint16_t GMAW_Flat_MAG_16_Speed[]={
	400,409,413,417,421, 425,429,439,448,457,
	466,471,490,505,522, 538,563,583,605,625,
	648,667,690,712,733, 755,779,800,822,842,
	865,892,920,940,950, 975,1000,1020,1050,1080,
	1110,1140,1176,1220,1260, 1300,1335,1363,1390,1490
};
const uint16_t GMAW_Wave_MAG_16_Volt[]={
	1000,1020,1040,1060,1080, 1150,1155,1160,1162,1165,
	1175,1178,1182,1188,1198, 1210,1225,1238,1265,1285,
	1300,1320,1350,1358,1366, 1380,1410,1430,1450,1470,
	1490,1510,1530,1575,1610, 1650,1720,1810,1910,1960,
	1973,2010,2060,2110,2150, 2187,2350,2360,2370,2380
};
const uint8_t  GMAW_Flat_MAG_16_Inductor[]={
	30,30,30,30,30, 30,30,30,30,30,
	30,30,30,30,30, 30,33,36,38,40,
	42,43,45,47,50, 55,60,65,75,80,
	80,80,80,80,80, 80,80,80,80,80,
	80,80,80,80,80, 80,80,80,80,80
};
const uint16_t GMAW_Flat_MAG_16_Vc[]={
	100,100,100,100,100, 100,100,100,100,100,
	100,100,100,120,150, 170,180,190,200,210, 
	220,210,200,190,180, 160,140,120,110,100,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};
const uint8_t  GMAW_Flat_MAG_16_VSLOPE[]={
	16,16,16,16,16, 16,16,16,16,18,
	16,16,16,16,18, 18,16,16,15,14,
	13,12,12,11,11, 10,10,10,8,5,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};

/*	FLUX CO2 1.2 */
const uint16_t GMAW_Flux_CO2_12_Speed[]={
	526,560,590,625,662, 698,736,772,800,840,
	885,940,1000,1055,1110, 1166,1230,1290,1350,1420,
	1476,1525,1600,1655,1738, 1813,1910,2010,2120,2230,
	2340,2400,2460,2520,2565, 2628,2628,2628,2628,2628,
	2628,2628,2628,2628,2628, 2628,2628,2628,2628,2628
};
const uint16_t GMAW_Flux_CO2_12_Volt[]={
	1213,1230,1250,1270,1285, 1290,1300,1310,1330,1350,
	1370,1420,1495,1542,1585, 1625,1660,1710,1770,1805,
	1855,1858,1860,1862,1865, 1870,1875,1878,1885,1895,
	1927,2007,2067,2110,2147, 2187,2227,2227,2300,2360,
	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
};
const uint8_t GMAW_Flux_CO2_12_Inductor[]={
	20,20,20,20,20, 20,20,20,20,22,
	25,28,30,31,32, 33,35,36,37,38,
	39,41,44,50,55, 60,65,70,75,80,
	80,80,80,80,80, 80,80,80,80,80,
	80,80,80,80,80, 80,80,80,80,80	
};
const uint16_t GMAW_Flux_CO2_12_Vc[]={
	80,80,80,80,80, 80,80,80,80,80,
	90,100,110,100,90, 80,70,60,50,50,
	50,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};
const uint8_t GMAW_Flux_CO2_12_VSLOPE[]={
	8,8,8,7,6, 5,5,4,4,3,
	3,3,3,3,3, 3,3,2,1,1,
	1,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};

/*	FLUX CO2 1.6 */
const uint16_t GMAW_Flux_CO2_16_Speed[]={
	412,423,435,449,466, 476,490,505,520,532,
	547,567,587,605,626, 646,675,702,730,758,
	787,815,845,875,905, 936,966,1000,1030,1070,
	1110,1150,1190,1230,1270, 1313,1360,1408,1455,1502,
	1553,1600,1650,1700,1755, 1819,1919,2019,2217,2327
};
const uint16_t GMAW_Flux_CO2_16_Volt[]={
	1213,1230,1250,1270,1285, 1290,1300,1310,1330,1350,
	1370,1420,1495,1542,1585, 1625,1660,1710,1770,1805,
	1855,1878,1889,1902,1915, 1916,1917,1918,1920,1925,
	1930,2050,2080,2110,2147, 2187,2227,2227,2300,2360,
	2360,2360,2360,2360,2360, 2360,2360,2360,2360,2360
};
const uint8_t GMAW_Flux_CO2_16_Inductor[]={
	20,20,20,20,20, 20,20,20,20,20,
	20,20,20,20,20, 20,20,20,20,22,
	25,28,30,31,33, 35,37,39,41,43,
	45,50,55,60,65, 70,80,80,80,80,
	80,80,80,80,80, 80,80,80,80,80
};
const uint16_t GMAW_Flux_CO2_16_Vc[]={
	80,80,80,80,80, 80,80,80,80,80,
	80,80,80,80,80, 90,100,110,100,90,
	80,70,60,50,50, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};
const uint8_t GMAW_Flux_CO2_16_VSLOPE[]={
	8,8,8,7,6, 5,5,4,4,3,
	5,5,4,4,3, 3,3,3,2,2,
	2,1,1,1,1, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0,
	0,0,0,0,0, 0,0,0,0,0
};



