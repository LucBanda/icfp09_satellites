#include "bin.h"

bin_4::bin_4(int instance):vm_state() {
	_instance = instance;
	min_out_port = 0;
	max_out_port = 101;
	min_global = 2039;
	max_global = 2128;
	output_ports =
		(double*)malloc(sizeof(double) * (max_out_port - min_out_port + 1));
	memory = (double*)malloc(sizeof(double) * (max_global - min_global + 1));
	input_ports = (double*)malloc(sizeof(double) * 4);
	delta_vx_addr = 0x2;
	delta_vy_addr = 0x3;
	instance_addr = 0x1;
	score_addr = 0;
	fuel_addr = 0x1;
	pos_x_addr = 0x2;
	pos_y_addr = 0x3;
	tank_x_addr = 0x4;
	tank_y_addr = 0x5;
	tank_fuel_addr = 0x6;
	nb_of_targets = 12;
	old_target_rel_pos_x.clear();
	old_target_rel_pos_y.clear();
	rel_speed_targets_x.clear();
	rel_speed_targets_y.clear();
	reset();
	step();
	_fuel_max = get_fuel();
	_tank_max = get_tank_fuel();
	step();
	reset();
	old_target_rel_pos_x.clear();
	old_target_rel_pos_y.clear();
}

bin_4::~bin_4() {
	free(memory);
	free(output_ports);
	free(input_ports);
}

double bin_4::get_relative_distance(int target) {
	return hypotl(output_ports[pos_target_x_addrs[target]], output_ports[pos_target_y_addrs[target]]);
}

double bin_4::get_relative_distance_to_tank() {
	return hypotl(output_ports[tank_x_addr], output_ports[tank_y_addr]);
}
double bin_4::get_tank_fuel() {
	return output_ports[tank_fuel_addr];
}

double bin_4::get_max_tank_fuel() {
	return _tank_max;
}

Complex bin_4::get_tank_absolute_position() {
	Complex relative_tank_pos(output_ports[tank_x_addr], output_ports[tank_y_addr]);
	Complex absolute_target_pos = get_absolute_position() + relative_tank_pos;

	return absolute_target_pos;
}

bool bin_4::is_target_validated(int target) {
	return output_ports[validated_target_addr[target]] == 1.0;
}

Complex bin_4::get_target_absolute_position(int target) {
	Complex relative_target_pos(old_target_rel_pos_x[target], old_target_rel_pos_y[target]);
	Complex absolute_target_pos = get_absolute_position() + relative_target_pos;

	return absolute_target_pos;
}

Complex bin_4::get_absolute_position() {
	return -Complex(output_ports[pos_x_addr], output_ports[pos_y_addr]);
}

double bin_4::get_relative_delta_speed(int target) {
	return hypotl(rel_speed_targets_x[target], rel_speed_targets_y[target]);
}

Complex bin_4::get_relative_speed(int target) {
	return Complex(rel_speed_targets_x[target], rel_speed_targets_y[target]);
}



void bin_4::step() {
	bool lstatus = status;
	double local_2, local_3, local_4, local_5, local_8, local_10, local_12,
		local_13, local_15, local_16, local_18, local_19, local_20, local_21,
		local_22, local_24, local_25, local_27, local_28, local_29, local_30,
		local_31, local_32, local_33, local_36, local_38, local_39, local_40,
		local_41, local_42, local_44, local_45, local_46, local_47, local_49,
		local_50, local_51, local_52, local_53, local_54, local_55, local_57,
		local_59, local_60, local_61, local_62, local_63, local_64, local_65,
		local_66, local_67, local_68, local_69, local_70, local_71, local_72,
		local_73, local_74, local_75, local_76, local_77, local_79, local_80,
		local_81, local_82, local_83, local_84, local_85, local_86, local_87,
		local_88, local_89, local_90, local_91, local_92, local_93, local_94,
		local_95, local_97, local_98, local_99, local_100, local_101, local_102,
		local_103, local_104, local_105, local_106, local_107, local_108,
		local_109, local_110, local_111, local_112, local_113, local_114,
		local_115, local_116, local_117, local_118, local_119, local_120,
		local_121, local_122, local_123, local_124, local_125, local_126,
		local_127, local_128, local_129, local_130, local_131, local_133,
		local_134, local_137, local_138, local_140, local_143, local_145,
		local_148, local_150, local_153, local_155, local_157, local_158,
		local_159, local_160, local_163, local_166, local_169, local_172,
		local_174, local_175, local_176, local_177, local_178, local_179,
		local_180, local_181, local_182, local_183, local_184, local_185,
		local_186, local_187, local_188, local_189, local_190, local_191,
		local_192, local_193, local_194, local_195, local_198, local_201,
		local_204, local_207, local_209, local_210, local_211, local_212,
		local_213, local_214, local_215, local_216, local_217, local_218,
		local_219, local_222, local_225, local_228, local_231, local_233,
		local_234, local_235, local_236, local_237, local_238, local_239,
		local_240, local_241, local_242, local_243, local_244, local_245,
		local_246, local_247, local_248, local_249, local_250, local_251,
		local_252, local_253, local_254, local_255, local_256, local_257,
		local_258, local_259, local_260, local_261, local_262, local_263,
		local_264, local_265, local_266, local_267, local_270, local_272,
		local_274, local_276, local_278, local_279, local_282, local_285,
		local_288, local_291, local_293, local_294, local_295, local_296,
		local_299, local_302, local_305, local_308, local_310, local_311,
		local_312, local_313, local_314, local_315, local_316, local_317,
		local_318, local_319, local_320, local_321, local_322, local_323,
		local_324, local_325, local_326, local_327, local_328, local_329,
		local_330, local_331, local_334, local_337, local_340, local_343,
		local_345, local_346, local_347, local_348, local_349, local_350,
		local_351, local_352, local_353, local_354, local_355, local_358,
		local_361, local_364, local_367, local_369, local_370, local_371,
		local_372, local_373, local_374, local_375, local_376, local_377,
		local_378, local_379, local_380, local_381, local_382, local_383,
		local_384, local_385, local_386, local_387, local_388, local_389,
		local_390, local_391, local_392, local_393, local_394, local_395,
		local_396, local_397, local_398, local_399, local_400, local_401,
		local_402, local_403, local_405, local_406, local_409, local_412,
		local_415, local_418, local_420, local_421, local_422, local_423,
		local_426, local_429, local_432, local_435, local_437, local_438,
		local_439, local_440, local_441, local_442, local_443, local_444,
		local_445, local_446, local_447, local_448, local_449, local_450,
		local_451, local_452, local_453, local_454, local_455, local_456,
		local_457, local_458, local_461, local_464, local_467, local_470,
		local_472, local_473, local_474, local_475, local_476, local_477,
		local_478, local_479, local_480, local_481, local_482, local_485,
		local_488, local_491, local_494, local_496, local_497, local_498,
		local_499, local_500, local_501, local_502, local_503, local_504,
		local_505, local_506, local_507, local_508, local_509, local_510,
		local_511, local_512, local_513, local_514, local_515, local_516,
		local_517, local_518, local_519, local_520, local_521, local_522,
		local_523, local_524, local_525, local_526, local_527, local_528,
		local_529, local_530, local_532, local_533, local_536, local_539,
		local_542, local_545, local_547, local_548, local_549, local_550,
		local_553, local_556, local_559, local_562, local_564, local_565,
		local_566, local_567, local_568, local_569, local_570, local_571,
		local_572, local_573, local_574, local_575, local_576, local_577,
		local_578, local_579, local_580, local_581, local_582, local_583,
		local_584, local_585, local_588, local_591, local_594, local_597,
		local_599, local_600, local_601, local_602, local_603, local_604,
		local_605, local_606, local_607, local_608, local_609, local_612,
		local_615, local_618, local_621, local_623, local_624, local_625,
		local_626, local_627, local_628, local_629, local_630, local_631,
		local_632, local_633, local_634, local_635, local_636, local_637,
		local_638, local_639, local_640, local_641, local_642, local_643,
		local_644, local_645, local_646, local_647, local_648, local_649,
		local_650, local_651, local_652, local_653, local_654, local_655,
		local_656, local_657, local_659, local_660, local_663, local_666,
		local_669, local_672, local_674, local_675, local_676, local_677,
		local_680, local_683, local_686, local_689, local_691, local_692,
		local_693, local_694, local_695, local_696, local_697, local_698,
		local_699, local_700, local_701, local_702, local_703, local_704,
		local_705, local_706, local_707, local_708, local_709, local_710,
		local_711, local_712, local_715, local_718, local_721, local_724,
		local_726, local_727, local_728, local_729, local_730, local_731,
		local_732, local_733, local_734, local_735, local_736, local_739,
		local_742, local_745, local_748, local_750, local_751, local_752,
		local_753, local_754, local_755, local_756, local_757, local_758,
		local_759, local_760, local_761, local_762, local_763, local_764,
		local_765, local_766, local_767, local_768, local_769, local_770,
		local_771, local_772, local_773, local_774, local_775, local_776,
		local_777, local_778, local_779, local_780, local_781, local_782,
		local_783, local_784, local_786, local_787, local_790, local_793,
		local_796, local_799, local_801, local_802, local_803, local_804,
		local_807, local_810, local_813, local_816, local_818, local_819,
		local_820, local_821, local_822, local_823, local_824, local_825,
		local_826, local_827, local_828, local_829, local_830, local_831,
		local_832, local_833, local_834, local_835, local_836, local_837,
		local_838, local_839, local_842, local_845, local_848, local_851,
		local_853, local_854, local_855, local_856, local_857, local_858,
		local_859, local_860, local_861, local_862, local_863, local_866,
		local_869, local_872, local_875, local_877, local_878, local_879,
		local_880, local_881, local_882, local_883, local_884, local_885,
		local_886, local_887, local_888, local_889, local_890, local_891,
		local_892, local_893, local_894, local_895, local_896, local_897,
		local_898, local_899, local_900, local_901, local_902, local_903,
		local_904, local_905, local_906, local_907, local_908, local_909,
		local_910, local_911, local_913, local_914, local_917, local_920,
		local_923, local_926, local_928, local_929, local_930, local_931,
		local_934, local_937, local_940, local_943, local_945, local_946,
		local_947, local_948, local_949, local_950, local_951, local_952,
		local_953, local_954, local_955, local_956, local_957, local_958,
		local_959, local_960, local_961, local_962, local_963, local_964,
		local_965, local_966, local_969, local_972, local_975, local_978,
		local_980, local_981, local_982, local_983, local_984, local_985,
		local_986, local_987, local_988, local_989, local_990, local_993,
		local_996, local_999, local_1002, local_1004, local_1005, local_1006,
		local_1007, local_1008, local_1009, local_1010, local_1011, local_1012,
		local_1013, local_1014, local_1015, local_1016, local_1017, local_1018,
		local_1019, local_1020, local_1021, local_1022, local_1023, local_1024,
		local_1025, local_1026, local_1027, local_1028, local_1029, local_1030,
		local_1031, local_1032, local_1033, local_1034, local_1035, local_1036,
		local_1037, local_1038, local_1040, local_1041, local_1044, local_1047,
		local_1050, local_1053, local_1055, local_1056, local_1057, local_1058,
		local_1061, local_1064, local_1067, local_1070, local_1072, local_1073,
		local_1074, local_1075, local_1076, local_1077, local_1078, local_1079,
		local_1080, local_1081, local_1082, local_1083, local_1084, local_1085,
		local_1086, local_1087, local_1088, local_1089, local_1090, local_1091,
		local_1092, local_1093, local_1096, local_1099, local_1102, local_1105,
		local_1107, local_1108, local_1109, local_1110, local_1111, local_1112,
		local_1113, local_1114, local_1115, local_1116, local_1117, local_1120,
		local_1123, local_1126, local_1129, local_1131, local_1132, local_1133,
		local_1134, local_1135, local_1136, local_1137, local_1138, local_1139,
		local_1140, local_1141, local_1142, local_1143, local_1144, local_1145,
		local_1146, local_1147, local_1148, local_1149, local_1150, local_1151,
		local_1152, local_1153, local_1154, local_1155, local_1156, local_1157,
		local_1158, local_1159, local_1160, local_1161, local_1162, local_1163,
		local_1164, local_1165, local_1167, local_1168, local_1171, local_1174,
		local_1177, local_1180, local_1182, local_1183, local_1184, local_1185,
		local_1188, local_1191, local_1194, local_1197, local_1199, local_1200,
		local_1201, local_1202, local_1203, local_1204, local_1205, local_1206,
		local_1207, local_1208, local_1209, local_1210, local_1211, local_1212,
		local_1213, local_1214, local_1215, local_1216, local_1217, local_1218,
		local_1219, local_1220, local_1223, local_1226, local_1229, local_1232,
		local_1234, local_1235, local_1236, local_1237, local_1238, local_1239,
		local_1240, local_1241, local_1242, local_1243, local_1244, local_1247,
		local_1250, local_1253, local_1256, local_1258, local_1259, local_1260,
		local_1261, local_1262, local_1263, local_1264, local_1265, local_1266,
		local_1267, local_1268, local_1269, local_1270, local_1271, local_1272,
		local_1273, local_1274, local_1275, local_1276, local_1277, local_1278,
		local_1279, local_1280, local_1281, local_1282, local_1283, local_1284,
		local_1285, local_1286, local_1287, local_1288, local_1289, local_1290,
		local_1291, local_1292, local_1294, local_1295, local_1298, local_1301,
		local_1304, local_1307, local_1309, local_1310, local_1311, local_1312,
		local_1315, local_1318, local_1321, local_1324, local_1326, local_1327,
		local_1328, local_1329, local_1330, local_1331, local_1332, local_1333,
		local_1334, local_1335, local_1336, local_1337, local_1338, local_1339,
		local_1340, local_1341, local_1342, local_1343, local_1344, local_1345,
		local_1346, local_1347, local_1350, local_1353, local_1356, local_1359,
		local_1361, local_1362, local_1363, local_1364, local_1365, local_1366,
		local_1367, local_1368, local_1369, local_1370, local_1371, local_1374,
		local_1377, local_1380, local_1383, local_1385, local_1386, local_1387,
		local_1388, local_1389, local_1390, local_1391, local_1392, local_1393,
		local_1394, local_1395, local_1396, local_1397, local_1398, local_1399,
		local_1400, local_1401, local_1402, local_1403, local_1404, local_1405,
		local_1406, local_1407, local_1408, local_1409, local_1410, local_1411,
		local_1412, local_1413, local_1414, local_1415, local_1416, local_1417,
		local_1418, local_1419, local_1421, local_1422, local_1424, local_1425,
		local_1426, local_1427, local_1430, local_1431, local_1432, local_1433,
		local_1434, local_1435, local_1436, local_1437, local_1438, local_1439,
		local_1440, local_1441, local_1442, local_1443, local_1444, local_1445,
		local_1446, local_1447, local_1448, local_1449, local_1450, local_1451,
		local_1454, local_1455, local_1456, local_1457, local_1458, local_1459,
		local_1460, local_1461, local_1462, local_1463, local_1464, local_1466,
		local_1467, local_1468, local_1469, local_1470, local_1471, local_1472,
		local_1473, local_1474, local_1475, local_1476, local_1477, local_1478,
		local_1479, local_1480, local_1481, local_1482, local_1483, local_1484,
		local_1485, local_1486, local_1487, local_1488, local_1489, local_1490,
		local_1491, local_1492, local_1493, local_1494, local_1495, local_1496,
		local_1497, local_1498, local_1499, local_1500, local_1502, local_1503,
		local_1504, local_1505, local_1506, local_1507, local_1508, local_1509,
		local_1510, local_1511, local_1512, local_1513, local_1514, local_1516,
		local_1517, local_1518, local_1519, local_1520, local_1521, local_1522,
		local_1523, local_1524, local_1526, local_1527, local_1528, local_1529,
		local_1530, local_1531, local_1532, local_1533, local_1534, local_1535,
		local_1536, local_1537, local_1538, local_1539, local_1540, local_1541,
		local_1542, local_1543, local_1544, local_1545, local_1546, local_1547,
		local_1550, local_1552, local_1555, local_1557, local_1559, local_1560,
		local_1561, local_1562, local_1564, local_1567, local_1570, local_1573,
		local_1575, local_1576, local_1577, local_1578, local_1579, local_1580,
		local_1581, local_1582, local_1583, local_1584, local_1585, local_1586,
		local_1587, local_1588, local_1589, local_1590, local_1591, local_1592,
		local_1593, local_1594, local_1595, local_1596, local_1597, local_1598,
		local_1599, local_1602, local_1605, local_1608, local_1610, local_1611,
		local_1612, local_1613, local_1614, local_1615, local_1616, local_1617,
		local_1618, local_1619, local_1620, local_1621, local_1622, local_1623,
		local_1626, local_1628, local_1630, local_1632, local_1634, local_1635,
		local_1636, local_1637, local_1638, local_1639, local_1640, local_1641,
		local_1642, local_1643, local_1644, local_1645, local_1646, local_1647,
		local_1648, local_1649, local_1650, local_1651, local_1652, local_1653,
		local_1654, local_1655, local_1656, local_1657, local_1658, local_1659,
		local_1660, local_1661, local_1662, local_1663, local_1664, local_1665,
		local_1666, local_1667, local_1668, local_1669, local_1670, local_1672,
		local_1673, local_1675, local_1676, local_1678, local_1679, local_1680,
		local_1681, local_1682, local_1683, local_1684, local_1685, local_1686,
		local_1687, local_1689, local_1690, local_1692, local_1694, local_1696,
		local_1697, local_1699, local_1700, local_1702, local_1703, local_1704,
		local_1705, local_1706, local_1707, local_1708, local_1709, local_1710,
		local_1711, local_1712, local_1714, local_1716, local_1717, local_1719,
		local_1720, local_1722, local_1723, local_1724, local_1725, local_1726,
		local_1727, local_1728, local_1729, local_1730, local_1731, local_1732,
		local_1734, local_1736, local_1737, local_1739, local_1740, local_1742,
		local_1743, local_1744, local_1745, local_1746, local_1747, local_1748,
		local_1749, local_1750, local_1751, local_1752, local_1754, local_1756,
		local_1757, local_1759, local_1760, local_1762, local_1763, local_1764,
		local_1765, local_1766, local_1767, local_1768, local_1769, local_1770,
		local_1771, local_1772, local_1774, local_1776, local_1777, local_1779,
		local_1780, local_1782, local_1783, local_1784, local_1785, local_1786,
		local_1787, local_1788, local_1789, local_1790, local_1791, local_1792,
		local_1794, local_1796, local_1797, local_1799, local_1800, local_1802,
		local_1803, local_1804, local_1805, local_1806, local_1807, local_1808,
		local_1809, local_1810, local_1811, local_1812, local_1814, local_1816,
		local_1817, local_1819, local_1820, local_1822, local_1823, local_1824,
		local_1825, local_1826, local_1827, local_1828, local_1829, local_1830,
		local_1831, local_1832, local_1834, local_1836, local_1837, local_1839,
		local_1840, local_1842, local_1843, local_1844, local_1845, local_1846,
		local_1847, local_1848, local_1849, local_1850, local_1851, local_1852,
		local_1854, local_1856, local_1857, local_1859, local_1860, local_1862,
		local_1863, local_1864, local_1865, local_1866, local_1867, local_1868,
		local_1869, local_1870, local_1871, local_1872, local_1874, local_1876,
		local_1877, local_1879, local_1880, local_1882, local_1883, local_1884,
		local_1885, local_1886, local_1887, local_1888, local_1889, local_1890,
		local_1891, local_1892, local_1894, local_1896, local_1897, local_1899,
		local_1900, local_1902, local_1903, local_1904, local_1905, local_1906,
		local_1907, local_1908, local_1909, local_1910, local_1911, local_1912,
		local_1913, local_1914, local_1916, local_1918, local_1919, local_1921,
		local_1922, local_1925, local_1926, local_1929, local_1930, local_1931,
		local_1932, local_1933, local_1934, local_1935, local_1936, local_1937,
		local_1939, local_1941, local_1942, local_1943, local_1946, local_1947,
		local_1948, local_1949, local_1950, local_1951, local_1952, local_1953,
		local_1954, local_1955, local_1956, local_1957, local_1959, local_1960,
		local_1961, local_1962, local_1964, local_1965, local_1966, local_1967,
		local_1969, local_1970, local_1971, local_1973, local_1974, local_1976,
		local_1977, local_1978, local_1979, local_1980, local_1981, local_1982,
		local_1983, local_1985, local_1987, local_1988, local_1989, local_1990,
		local_1991, local_1992, local_1993, local_1995;
	const double
		const_0 = 2,
		const_1 = 1, const_6 = 200000, const_7 = 1.738e+06,
		const_9 = 3.84399e+08, const_11 = 0, const_34 = 6.67428e-11,
		const_35 = 7.347e+22, const_56 = 6e+24, const_135 = 5.32785e+07,
		const_136 = 4004, const_141 = 4.36287e+07, const_142 = 4003,
		const_146 = -1.08232e+08, const_147 = 4002, const_151 = -9.38399e+07,
		const_152 = 4001, const_161 = 9.22811e+07, const_164 = 9.0281e+07,
		const_167 = -6.03229e+07, const_170 = 5.41785e+07, const_196 = -1678.87,
		const_199 = -1889.32, const_202 = 437.609, const_205 = -1057.33,
		const_220 = 969.297, const_223 = 913.021, const_226 = -785.161,
		const_229 = -1831.36, const_268 = 1, const_280 = -2.36489e-08,
		const_283 = -7.357e+06, const_286 = -6.357e+07, const_289 = -9.8357e+07,
		const_297 = 9.6557e+07, const_300 = 9e+07, const_303 = -9.6357e+07,
		const_306 = -1.80673e-08, const_332 = -2036.51, const_335 = -2193.33,
		const_338 = 777.34, const_341 = 2.69338e-13, const_356 = -3.74088e-13,
		const_359 = -179.292, const_362 = -512.837, const_365 = -2199.39,
		const_407 = -4.32785e+07, const_410 = -4.63713e+07,
		const_413 = -1.18747e+07, const_416 = -7.65194e+07,
		const_424 = 7.49606e+07, const_427 = 6.56035e+07,
		const_430 = -1.06572e+08, const_433 = -4.41785e+07,
		const_459 = -1862.76, const_462 = -1896.1, const_465 = 960.299,
		const_468 = 1149.61, const_483 = -1075.47, const_486 = -1340.24,
		const_489 = -107.001, const_492 = -1991.19, const_534 = -6.63003e+07,
		const_537 = -6.43003e+07, const_540 = 3.43421e+07,
		const_543 = -3.91785e+07, const_551 = 3.82785e+07,
		const_554 = 2.86287e+07, const_557 = -9.32317e+07,
		const_560 = -6.78592e+07, const_586 = -1143.55, const_589 = -1004.14,
		const_592 = 941.94, const_595 = 2094.85, const_610 = -1980.69,
		const_613 = -2255.31, const_616 = 346.965, const_619 = -1209.46,
		const_661 = -6.6557e+07, const_664 = -6e+07, const_667 = 6.6357e+07,
		const_670 = 8.37104e-09, const_678 = -1.22259e-08,
		const_681 = -7.357e+06, const_684 = -6.357e+07, const_687 = -6.8357e+07,
		const_713 = 3.00384e-13, const_716 = 322.647, const_719 = 722.061,
		const_722 = 2565.62, const_737 = -2452.91, const_740 = -2631.35,
		const_743 = 753.717, const_746 = 1.57094e-13, const_788 = -4.89798e+07,
		const_791 = -3.96228e+07, const_794 = 8.05916e+07,
		const_797 = 2.91785e+07, const_805 = -2.82785e+07,
		const_808 = -3.13713e+07, const_811 = -2.68747e+07,
		const_814 = -5.05386e+07, const_840 = 1330.47, const_843 = 1791.03,
		const_846 = 343.41, const_849 = 2382.05, const_864 = -2304.44,
		const_867 = -2262.12, const_870 = 1029.81, const_873 = 1375.28,
		const_915 = -2.32785e+07, const_918 = -1.36287e+07,
		const_921 = 7.82317e+07, const_924 = 4.18784e+07,
		const_932 = -4.03195e+07, const_935 = -3.83195e+07,
		const_938 = 8.36134e+06, const_941 = -2.41785e+07, const_967 = 2539.9,
		const_970 = 3015.59, const_973 = -119.881, const_976 = 1496.41,
		const_991 = -1466.41, const_994 = -1072.52, const_997 = 1121.65,
		const_1000 = 2591.86, const_1042 = 4.47679e-09, const_1045 = 7.357e+06,
		const_1048 = 6.357e+07, const_1051 = 3.8357e+07,
		const_1059 = -3.6557e+07, const_1062 = -3e+07, const_1065 = 3.6357e+07,
		const_1068 = 2.34861e-09, const_1094 = 3309.73, const_1097 = 3549.45,
		const_1100 = -580.472, const_1103 = 0, const_1118 = 2.02656e-13,
		const_1121 = 870.443, const_1124 = 1014.95, const_1127 = 3328.07,
		const_1169 = 1.32785e+07, const_1172 = 1.63713e+07,
		const_1175 = 4.18747e+07, const_1178 = 2.45579e+07,
		const_1186 = -2.2999e+07, const_1189 = -1.3642e+07,
		const_1192 = 5.46108e+07, const_1195 = 1.41785e+07,
		const_1221 = 3362.94, const_1224 = 2802.82, const_1227 = -957.148,
		const_1230 = -1916.54, const_1245 = 1941.59, const_1248 = 3363.58,
		const_1251 = 733.926, const_1254 = 3319.54, const_1296 = 1.43388e+07,
		const_1299 = 1.23388e+07, const_1302 = 1.76194e+07,
		const_1305 = 9.1785e+06, const_1313 = -8.2785e+06,
		const_1316 = 1.37135e+06, const_1319 = 6.32317e+07,
		const_1322 = 1.58976e+07, const_1348 = 2458.99, const_1351 = -630.501,
		const_1354 = -1189.66, const_1357 = -4085.35, const_1372 = 4259.1,
		const_1375 = 5672.96, const_1378 = 331.497, const_1381 = 2358.68,
		const_1428 = 6.657e+06, const_1452 = -7756.01, const_1548 = 6.457e+06,
		const_1553 = -6.357e+06, const_1565 = 8.357e+06, const_1568 = 6.357e+06,
		const_1571 = 6.557e+06, const_1600 = -6922.34, const_1603 = -4719.32,
		const_1606 = -7814.93, const_1624 = -7875.22, const_1688 = 1000,
		const_1693 = 2e+06, const_1923 = 75000, const_1927 = 10000,
		const_1944 = 8, const_1945 = 2.4e+07, const_1958 = 75, const_1963 = 25,
		const_1986 = 6.357e+06, const_1996 = 0;

	if (time_step == 0) {
		validated_target_addr.clear();
		pos_target_x_addrs.clear();
		pos_target_y_addrs.clear();
		for (int i = 0; i < nb_of_targets; i++) {
			int addr_x = 0x7 + 3 * i;
			int addr_y = 0x8 + 3 * i;
			int addr_validated = 0x9 + 3 * i;
			pos_target_x_addrs.push_back(addr_x);
			pos_target_y_addrs.push_back(addr_y);
			validated_target_addr.push_back(addr_validated);
		}
	}
	local_2 = (const_1 == 0) ? 0 : const_1 / const_1;
	local_3 = local_2 * local_2;
	local_4 = (const_0 == 0) ? 0 : local_3 / const_0;
	local_5 = memory[2111 - min_global];
	local_8 = const_7 + const_6;
	local_10 = const_9 + local_8;
	local_12 = memory[2039 - min_global];
	local_13 = local_12 - const_11;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_15 = local_10;
	else
		local_15 = local_5;
	local_16 = memory[2051 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_18 = const_9;
	else
		local_18 = local_16;
	local_19 = local_18 - local_15;
	local_20 = local_19 * local_19;
	local_21 = memory[2110 - min_global];
	local_22 = const_11 + const_11;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_24 = local_22;
	else
		local_24 = local_21;
	local_25 = memory[2050 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_27 = const_11;
	else
		local_27 = local_25;
	local_28 = local_27 - local_24;
	local_29 = local_28 * local_28;
	local_30 = local_29 + local_20;
	local_31 = sqrt(local_30);
	local_32 = local_31 * local_31;
	local_33 = local_32 * local_31;
	local_36 = memory[2052 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_38 = const_35;
	else
		local_38 = local_36;
	local_39 = const_34 * local_38;
	local_40 = (local_33 == 0) ? 0 : local_39 / local_33;
	local_41 = local_19 * local_40;
	local_42 = memory[2041 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_44 = const_11;
	else
		local_44 = local_42;
	local_45 = local_44 - local_15;
	local_46 = local_45 * local_45;
	local_47 = memory[2040 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_49 = const_11;
	else
		local_49 = local_47;
	local_50 = local_49 - local_24;
	local_51 = local_50 * local_50;
	local_52 = local_51 + local_46;
	local_53 = sqrt(local_52);
	local_54 = local_53 * local_53;
	local_55 = local_54 * local_53;
	local_57 = memory[2042 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_59 = const_56;
	else
		local_59 = local_57;
	local_60 = const_34 * local_59;
	local_61 = (local_55 == 0) ? 0 : local_60 / local_55;
	local_62 = local_45 * local_61;
	local_63 = local_62 + local_41;
	local_64 = local_63 * local_4;
	local_65 = memory[2114 - min_global];
	local_66 = const_9 * const_9;
	local_67 = const_11 * const_11;
	local_68 = local_67 + local_66;
	local_69 = sqrt(local_68);
	local_70 = (local_69 == 0) ? 0 : const_11 / local_69;
	local_71 = const_1996 - local_70;
	local_72 = const_56 * const_34;
	local_73 = (local_69 == 0) ? 0 : local_72 / local_69;
	local_74 = sqrt(local_73);
	local_75 = local_74 + const_11;
	local_76 = local_71 * local_75;
	local_77 = local_76 + const_11;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_79 = local_77;
	else
		local_79 = local_65;
	local_80 = local_79 * local_2;
	local_81 = local_15 + local_80;
	local_82 = local_81 + local_64;
	local_83 = local_18 - local_82;
	local_84 = local_83 * local_83;
	local_85 = local_28 * local_40;
	local_86 = local_50 * local_61;
	local_87 = local_86 + local_85;
	local_88 = local_87 * local_4;
	local_89 = memory[2113 - min_global];
	local_90 = (local_69 == 0) ? 0 : const_9 / local_69;
	local_91 = local_90 * local_75;
	local_92 = const_35 * const_34;
	local_93 = (local_8 == 0) ? 0 : local_92 / local_8;
	local_94 = sqrt(local_93);
	local_95 = local_91 + local_94;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_97 = local_95;
	else
		local_97 = local_89;
	local_98 = local_97 * local_2;
	local_99 = local_24 + local_98;
	local_100 = local_99 + local_88;
	local_101 = local_27 - local_100;
	local_102 = local_101 * local_101;
	local_103 = local_102 + local_84;
	local_104 = sqrt(local_103);
	local_105 = local_104 * local_104;
	local_106 = local_105 * local_104;
	local_107 = (local_106 == 0) ? 0 : local_39 / local_106;
	local_108 = local_83 * local_107;
	local_109 = local_44 - local_82;
	local_110 = local_109 * local_109;
	local_111 = local_49 - local_100;
	local_112 = local_111 * local_111;
	local_113 = local_112 + local_110;
	local_114 = sqrt(local_113);
	local_115 = local_114 * local_114;
	local_116 = local_115 * local_114;
	local_117 = (local_116 == 0) ? 0 : local_60 / local_116;
	local_118 = local_109 * local_117;
	local_119 = local_118 + local_108;
	local_120 = local_119 + local_63;
	local_121 = (const_0 == 0) ? 0 : local_120 / const_0;
	local_122 = local_121 * local_2;
	local_123 = local_79 + local_122;
	local_124 = local_101 * local_107;
	local_125 = local_111 * local_117;
	local_126 = local_125 + local_124;
	local_127 = local_126 + local_87;
	local_128 = (const_0 == 0) ? 0 : local_127 / const_0;
	local_129 = local_128 * local_2;
	local_130 = local_97 + local_129;
	local_131 = memory[2112 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_133 = const_1;
	else
		local_133 = local_131;
	local_134 = memory[2106 - min_global];
	local_137 = input_ports[1];
	local_138 = local_137 - const_136;
	lstatus = (local_138 == 0);
	if (lstatus)
		local_140 = const_135;
	else
		local_140 = const_11;
	local_143 = local_137 - const_142;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_145 = const_141;
	else
		local_145 = local_140;
	local_148 = local_137 - const_147;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_150 = const_146;
	else
		local_150 = local_145;
	local_153 = local_137 - const_152;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_155 = const_151;
	else
		local_155 = local_150;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_157 = local_155;
	else
		local_157 = local_134;
	local_158 = local_18 - local_157;
	local_159 = local_158 * local_158;
	local_160 = memory[2105 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_163 = const_161;
	else
		local_163 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_166 = const_164;
	else
		local_166 = local_163;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_169 = const_167;
	else
		local_169 = local_166;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_172 = const_170;
	else
		local_172 = local_169;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_174 = local_172;
	else
		local_174 = local_160;
	local_175 = local_27 - local_174;
	local_176 = local_175 * local_175;
	local_177 = local_176 + local_159;
	local_178 = sqrt(local_177);
	local_179 = local_178 * local_178;
	local_180 = local_179 * local_178;
	local_181 = (local_180 == 0) ? 0 : local_39 / local_180;
	local_182 = local_158 * local_181;
	local_183 = local_44 - local_157;
	local_184 = local_183 * local_183;
	local_185 = local_49 - local_174;
	local_186 = local_185 * local_185;
	local_187 = local_186 + local_184;
	local_188 = sqrt(local_187);
	local_189 = local_188 * local_188;
	local_190 = local_189 * local_188;
	local_191 = (local_190 == 0) ? 0 : local_60 / local_190;
	local_192 = local_183 * local_191;
	local_193 = local_192 + local_182;
	local_194 = local_193 * local_4;
	local_195 = memory[2109 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_198 = const_196;
	else
		local_198 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_201 = const_199;
	else
		local_201 = local_198;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_204 = const_202;
	else
		local_204 = local_201;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_207 = const_205;
	else
		local_207 = local_204;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_209 = local_207;
	else
		local_209 = local_195;
	local_210 = local_209 * local_2;
	local_211 = local_157 + local_210;
	local_212 = local_211 + local_194;
	local_213 = local_18 - local_212;
	local_214 = local_213 * local_213;
	local_215 = local_175 * local_181;
	local_216 = local_185 * local_191;
	local_217 = local_216 + local_215;
	local_218 = local_217 * local_4;
	local_219 = memory[2108 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_222 = const_220;
	else
		local_222 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_225 = const_223;
	else
		local_225 = local_222;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_228 = const_226;
	else
		local_228 = local_225;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_231 = const_229;
	else
		local_231 = local_228;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_233 = local_231;
	else
		local_233 = local_219;
	local_234 = local_233 * local_2;
	local_235 = local_174 + local_234;
	local_236 = local_235 + local_218;
	local_237 = local_27 - local_236;
	local_238 = local_237 * local_237;
	local_239 = local_238 + local_214;
	local_240 = sqrt(local_239);
	local_241 = local_240 * local_240;
	local_242 = local_241 * local_240;
	local_243 = (local_242 == 0) ? 0 : local_39 / local_242;
	local_244 = local_213 * local_243;
	local_245 = local_44 - local_212;
	local_246 = local_245 * local_245;
	local_247 = local_49 - local_236;
	local_248 = local_247 * local_247;
	local_249 = local_248 + local_246;
	local_250 = sqrt(local_249);
	local_251 = local_250 * local_250;
	local_252 = local_251 * local_250;
	local_253 = (local_252 == 0) ? 0 : local_60 / local_252;
	local_254 = local_245 * local_253;
	local_255 = local_254 + local_244;
	local_256 = local_255 + local_193;
	local_257 = (const_0 == 0) ? 0 : local_256 / const_0;
	local_258 = local_257 * local_2;
	local_259 = local_209 + local_258;
	local_260 = local_237 * local_243;
	local_261 = local_247 * local_253;
	local_262 = local_261 + local_260;
	local_263 = local_262 + local_217;
	local_264 = (const_0 == 0) ? 0 : local_263 / const_0;
	local_265 = local_264 * local_2;
	local_266 = local_233 + local_265;
	local_267 = memory[2107 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_270 = const_268;
	else
		local_270 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_272 = const_268;
	else
		local_272 = local_270;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_274 = const_268;
	else
		local_274 = local_272;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_276 = const_268;
	else
		local_276 = local_274;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_278 = local_276;
	else
		local_278 = local_267;
	local_279 = memory[2101 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_282 = const_280;
	else
		local_282 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_285 = const_283;
	else
		local_285 = local_282;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_288 = const_286;
	else
		local_288 = local_285;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_291 = const_289;
	else
		local_291 = local_288;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_293 = local_291;
	else
		local_293 = local_279;
	local_294 = local_18 - local_293;
	local_295 = local_294 * local_294;
	local_296 = memory[2100 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_299 = const_297;
	else
		local_299 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_302 = const_300;
	else
		local_302 = local_299;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_305 = const_303;
	else
		local_305 = local_302;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_308 = const_306;
	else
		local_308 = local_305;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_310 = local_308;
	else
		local_310 = local_296;
	local_311 = local_27 - local_310;
	local_312 = local_311 * local_311;
	local_313 = local_312 + local_295;
	local_314 = sqrt(local_313);
	local_315 = local_314 * local_314;
	local_316 = local_315 * local_314;
	local_317 = (local_316 == 0) ? 0 : local_39 / local_316;
	local_318 = local_294 * local_317;
	local_319 = local_44 - local_293;
	local_320 = local_319 * local_319;
	local_321 = local_49 - local_310;
	local_322 = local_321 * local_321;
	local_323 = local_322 + local_320;
	local_324 = sqrt(local_323);
	local_325 = local_324 * local_324;
	local_326 = local_325 * local_324;
	local_327 = (local_326 == 0) ? 0 : local_60 / local_326;
	local_328 = local_319 * local_327;
	local_329 = local_328 + local_318;
	local_330 = local_329 * local_4;
	local_331 = memory[2104 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_334 = const_332;
	else
		local_334 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_337 = const_335;
	else
		local_337 = local_334;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_340 = const_338;
	else
		local_340 = local_337;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_343 = const_341;
	else
		local_343 = local_340;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_345 = local_343;
	else
		local_345 = local_331;
	local_346 = local_345 * local_2;
	local_347 = local_293 + local_346;
	local_348 = local_347 + local_330;
	local_349 = local_18 - local_348;
	local_350 = local_349 * local_349;
	local_351 = local_311 * local_317;
	local_352 = local_321 * local_327;
	local_353 = local_352 + local_351;
	local_354 = local_353 * local_4;
	local_355 = memory[2103 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_358 = const_356;
	else
		local_358 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_361 = const_359;
	else
		local_361 = local_358;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_364 = const_362;
	else
		local_364 = local_361;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_367 = const_365;
	else
		local_367 = local_364;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_369 = local_367;
	else
		local_369 = local_355;
	local_370 = local_369 * local_2;
	local_371 = local_310 + local_370;
	local_372 = local_371 + local_354;
	local_373 = local_27 - local_372;
	local_374 = local_373 * local_373;
	local_375 = local_374 + local_350;
	local_376 = sqrt(local_375);
	local_377 = local_376 * local_376;
	local_378 = local_377 * local_376;
	local_379 = (local_378 == 0) ? 0 : local_39 / local_378;
	local_380 = local_349 * local_379;
	local_381 = local_44 - local_348;
	local_382 = local_381 * local_381;
	local_383 = local_49 - local_372;
	local_384 = local_383 * local_383;
	local_385 = local_384 + local_382;
	local_386 = sqrt(local_385);
	local_387 = local_386 * local_386;
	local_388 = local_387 * local_386;
	local_389 = (local_388 == 0) ? 0 : local_60 / local_388;
	local_390 = local_381 * local_389;
	local_391 = local_390 + local_380;
	local_392 = local_391 + local_329;
	local_393 = (const_0 == 0) ? 0 : local_392 / const_0;
	local_394 = local_393 * local_2;
	local_395 = local_345 + local_394;
	local_396 = local_373 * local_379;
	local_397 = local_383 * local_389;
	local_398 = local_397 + local_396;
	local_399 = local_398 + local_353;
	local_400 = (const_0 == 0) ? 0 : local_399 / const_0;
	local_401 = local_400 * local_2;
	local_402 = local_369 + local_401;
	local_403 = memory[2102 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_405 = local_276;
	else
		local_405 = local_403;
	local_406 = memory[2096 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_409 = const_407;
	else
		local_409 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_412 = const_410;
	else
		local_412 = local_409;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_415 = const_413;
	else
		local_415 = local_412;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_418 = const_416;
	else
		local_418 = local_415;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_420 = local_418;
	else
		local_420 = local_406;
	local_421 = local_18 - local_420;
	local_422 = local_421 * local_421;
	local_423 = memory[2095 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_426 = const_424;
	else
		local_426 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_429 = const_427;
	else
		local_429 = local_426;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_432 = const_430;
	else
		local_432 = local_429;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_435 = const_433;
	else
		local_435 = local_432;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_437 = local_435;
	else
		local_437 = local_423;
	local_438 = local_27 - local_437;
	local_439 = local_438 * local_438;
	local_440 = local_439 + local_422;
	local_441 = sqrt(local_440);
	local_442 = local_441 * local_441;
	local_443 = local_442 * local_441;
	local_444 = (local_443 == 0) ? 0 : local_39 / local_443;
	local_445 = local_421 * local_444;
	local_446 = local_44 - local_420;
	local_447 = local_446 * local_446;
	local_448 = local_49 - local_437;
	local_449 = local_448 * local_448;
	local_450 = local_449 + local_447;
	local_451 = sqrt(local_450);
	local_452 = local_451 * local_451;
	local_453 = local_452 * local_451;
	local_454 = (local_453 == 0) ? 0 : local_60 / local_453;
	local_455 = local_446 * local_454;
	local_456 = local_455 + local_445;
	local_457 = local_456 * local_4;
	local_458 = memory[2099 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_461 = const_459;
	else
		local_461 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_464 = const_462;
	else
		local_464 = local_461;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_467 = const_465;
	else
		local_467 = local_464;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_470 = const_468;
	else
		local_470 = local_467;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_472 = local_470;
	else
		local_472 = local_458;
	local_473 = local_472 * local_2;
	local_474 = local_420 + local_473;
	local_475 = local_474 + local_457;
	local_476 = local_18 - local_475;
	local_477 = local_476 * local_476;
	local_478 = local_438 * local_444;
	local_479 = local_448 * local_454;
	local_480 = local_479 + local_478;
	local_481 = local_480 * local_4;
	local_482 = memory[2098 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_485 = const_483;
	else
		local_485 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_488 = const_486;
	else
		local_488 = local_485;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_491 = const_489;
	else
		local_491 = local_488;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_494 = const_492;
	else
		local_494 = local_491;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_496 = local_494;
	else
		local_496 = local_482;
	local_497 = local_496 * local_2;
	local_498 = local_437 + local_497;
	local_499 = local_498 + local_481;
	local_500 = local_27 - local_499;
	local_501 = local_500 * local_500;
	local_502 = local_501 + local_477;
	local_503 = sqrt(local_502);
	local_504 = local_503 * local_503;
	local_505 = local_504 * local_503;
	local_506 = (local_505 == 0) ? 0 : local_39 / local_505;
	local_507 = local_476 * local_506;
	local_508 = local_44 - local_475;
	local_509 = local_508 * local_508;
	local_510 = local_49 - local_499;
	local_511 = local_510 * local_510;
	local_512 = local_511 + local_509;
	local_513 = sqrt(local_512);
	local_514 = local_513 * local_513;
	local_515 = local_514 * local_513;
	local_516 = (local_515 == 0) ? 0 : local_60 / local_515;
	local_517 = local_508 * local_516;
	local_518 = local_517 + local_507;
	local_519 = local_518 + local_456;
	local_520 = (const_0 == 0) ? 0 : local_519 / const_0;
	local_521 = local_520 * local_2;
	local_522 = local_472 + local_521;
	local_523 = local_500 * local_506;
	local_524 = local_510 * local_516;
	local_525 = local_524 + local_523;
	local_526 = local_525 + local_480;
	local_527 = (const_0 == 0) ? 0 : local_526 / const_0;
	local_528 = local_527 * local_2;
	local_529 = local_496 + local_528;
	local_530 = memory[2097 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_532 = local_276;
	else
		local_532 = local_530;
	local_533 = memory[2091 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_536 = const_534;
	else
		local_536 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_539 = const_537;
	else
		local_539 = local_536;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_542 = const_540;
	else
		local_542 = local_539;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_545 = const_543;
	else
		local_545 = local_542;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_547 = local_545;
	else
		local_547 = local_533;
	local_548 = local_18 - local_547;
	local_549 = local_548 * local_548;
	local_550 = memory[2090 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_553 = const_551;
	else
		local_553 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_556 = const_554;
	else
		local_556 = local_553;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_559 = const_557;
	else
		local_559 = local_556;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_562 = const_560;
	else
		local_562 = local_559;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_564 = local_562;
	else
		local_564 = local_550;
	local_565 = local_27 - local_564;
	local_566 = local_565 * local_565;
	local_567 = local_566 + local_549;
	local_568 = sqrt(local_567);
	local_569 = local_568 * local_568;
	local_570 = local_569 * local_568;
	local_571 = (local_570 == 0) ? 0 : local_39 / local_570;
	local_572 = local_548 * local_571;
	local_573 = local_44 - local_547;
	local_574 = local_573 * local_573;
	local_575 = local_49 - local_564;
	local_576 = local_575 * local_575;
	local_577 = local_576 + local_574;
	local_578 = sqrt(local_577);
	local_579 = local_578 * local_578;
	local_580 = local_579 * local_578;
	local_581 = (local_580 == 0) ? 0 : local_60 / local_580;
	local_582 = local_573 * local_581;
	local_583 = local_582 + local_572;
	local_584 = local_583 * local_4;
	local_585 = memory[2094 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_588 = const_586;
	else
		local_588 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_591 = const_589;
	else
		local_591 = local_588;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_594 = const_592;
	else
		local_594 = local_591;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_597 = const_595;
	else
		local_597 = local_594;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_599 = local_597;
	else
		local_599 = local_585;
	local_600 = local_599 * local_2;
	local_601 = local_547 + local_600;
	local_602 = local_601 + local_584;
	local_603 = local_18 - local_602;
	local_604 = local_603 * local_603;
	local_605 = local_565 * local_571;
	local_606 = local_575 * local_581;
	local_607 = local_606 + local_605;
	local_608 = local_607 * local_4;
	local_609 = memory[2093 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_612 = const_610;
	else
		local_612 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_615 = const_613;
	else
		local_615 = local_612;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_618 = const_616;
	else
		local_618 = local_615;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_621 = const_619;
	else
		local_621 = local_618;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_623 = local_621;
	else
		local_623 = local_609;
	local_624 = local_623 * local_2;
	local_625 = local_564 + local_624;
	local_626 = local_625 + local_608;
	local_627 = local_27 - local_626;
	local_628 = local_627 * local_627;
	local_629 = local_628 + local_604;
	local_630 = sqrt(local_629);
	local_631 = local_630 * local_630;
	local_632 = local_631 * local_630;
	local_633 = (local_632 == 0) ? 0 : local_39 / local_632;
	local_634 = local_603 * local_633;
	local_635 = local_44 - local_602;
	local_636 = local_635 * local_635;
	local_637 = local_49 - local_626;
	local_638 = local_637 * local_637;
	local_639 = local_638 + local_636;
	local_640 = sqrt(local_639);
	local_641 = local_640 * local_640;
	local_642 = local_641 * local_640;
	local_643 = (local_642 == 0) ? 0 : local_60 / local_642;
	local_644 = local_635 * local_643;
	local_645 = local_644 + local_634;
	local_646 = local_645 + local_583;
	local_647 = (const_0 == 0) ? 0 : local_646 / const_0;
	local_648 = local_647 * local_2;
	local_649 = local_599 + local_648;
	local_650 = local_627 * local_633;
	local_651 = local_637 * local_643;
	local_652 = local_651 + local_650;
	local_653 = local_652 + local_607;
	local_654 = (const_0 == 0) ? 0 : local_653 / const_0;
	local_655 = local_654 * local_2;
	local_656 = local_623 + local_655;
	local_657 = memory[2092 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_659 = local_276;
	else
		local_659 = local_657;
	local_660 = memory[2086 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_663 = const_661;
	else
		local_663 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_666 = const_664;
	else
		local_666 = local_663;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_669 = const_667;
	else
		local_669 = local_666;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_672 = const_670;
	else
		local_672 = local_669;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_674 = local_672;
	else
		local_674 = local_660;
	local_675 = local_18 - local_674;
	local_676 = local_675 * local_675;
	local_677 = memory[2085 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_680 = const_678;
	else
		local_680 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_683 = const_681;
	else
		local_683 = local_680;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_686 = const_684;
	else
		local_686 = local_683;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_689 = const_687;
	else
		local_689 = local_686;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_691 = local_689;
	else
		local_691 = local_677;
	local_692 = local_27 - local_691;
	local_693 = local_692 * local_692;
	local_694 = local_693 + local_676;
	local_695 = sqrt(local_694);
	local_696 = local_695 * local_695;
	local_697 = local_696 * local_695;
	local_698 = (local_697 == 0) ? 0 : local_39 / local_697;
	local_699 = local_675 * local_698;
	local_700 = local_44 - local_674;
	local_701 = local_700 * local_700;
	local_702 = local_49 - local_691;
	local_703 = local_702 * local_702;
	local_704 = local_703 + local_701;
	local_705 = sqrt(local_704);
	local_706 = local_705 * local_705;
	local_707 = local_706 * local_705;
	local_708 = (local_707 == 0) ? 0 : local_60 / local_707;
	local_709 = local_700 * local_708;
	local_710 = local_709 + local_699;
	local_711 = local_710 * local_4;
	local_712 = memory[2089 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_715 = const_713;
	else
		local_715 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_718 = const_716;
	else
		local_718 = local_715;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_721 = const_719;
	else
		local_721 = local_718;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_724 = const_722;
	else
		local_724 = local_721;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_726 = local_724;
	else
		local_726 = local_712;
	local_727 = local_726 * local_2;
	local_728 = local_674 + local_727;
	local_729 = local_728 + local_711;
	local_730 = local_18 - local_729;
	local_731 = local_730 * local_730;
	local_732 = local_692 * local_698;
	local_733 = local_702 * local_708;
	local_734 = local_733 + local_732;
	local_735 = local_734 * local_4;
	local_736 = memory[2088 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_739 = const_737;
	else
		local_739 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_742 = const_740;
	else
		local_742 = local_739;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_745 = const_743;
	else
		local_745 = local_742;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_748 = const_746;
	else
		local_748 = local_745;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_750 = local_748;
	else
		local_750 = local_736;
	local_751 = local_750 * local_2;
	local_752 = local_691 + local_751;
	local_753 = local_752 + local_735;
	local_754 = local_27 - local_753;
	local_755 = local_754 * local_754;
	local_756 = local_755 + local_731;
	local_757 = sqrt(local_756);
	local_758 = local_757 * local_757;
	local_759 = local_758 * local_757;
	local_760 = (local_759 == 0) ? 0 : local_39 / local_759;
	local_761 = local_730 * local_760;
	local_762 = local_44 - local_729;
	local_763 = local_762 * local_762;
	local_764 = local_49 - local_753;
	local_765 = local_764 * local_764;
	local_766 = local_765 + local_763;
	local_767 = sqrt(local_766);
	local_768 = local_767 * local_767;
	local_769 = local_768 * local_767;
	local_770 = (local_769 == 0) ? 0 : local_60 / local_769;
	local_771 = local_762 * local_770;
	local_772 = local_771 + local_761;
	local_773 = local_772 + local_710;
	local_774 = (const_0 == 0) ? 0 : local_773 / const_0;
	local_775 = local_774 * local_2;
	local_776 = local_726 + local_775;
	local_777 = local_754 * local_760;
	local_778 = local_764 * local_770;
	local_779 = local_778 + local_777;
	local_780 = local_779 + local_734;
	local_781 = (const_0 == 0) ? 0 : local_780 / const_0;
	local_782 = local_781 * local_2;
	local_783 = local_750 + local_782;
	local_784 = memory[2087 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_786 = local_276;
	else
		local_786 = local_784;
	local_787 = memory[2081 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_790 = const_788;
	else
		local_790 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_793 = const_791;
	else
		local_793 = local_790;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_796 = const_794;
	else
		local_796 = local_793;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_799 = const_797;
	else
		local_799 = local_796;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_801 = local_799;
	else
		local_801 = local_787;
	local_802 = local_18 - local_801;
	local_803 = local_802 * local_802;
	local_804 = memory[2080 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_807 = const_805;
	else
		local_807 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_810 = const_808;
	else
		local_810 = local_807;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_813 = const_811;
	else
		local_813 = local_810;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_816 = const_814;
	else
		local_816 = local_813;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_818 = local_816;
	else
		local_818 = local_804;
	local_819 = local_27 - local_818;
	local_820 = local_819 * local_819;
	local_821 = local_820 + local_803;
	local_822 = sqrt(local_821);
	local_823 = local_822 * local_822;
	local_824 = local_823 * local_822;
	local_825 = (local_824 == 0) ? 0 : local_39 / local_824;
	local_826 = local_802 * local_825;
	local_827 = local_44 - local_801;
	local_828 = local_827 * local_827;
	local_829 = local_49 - local_818;
	local_830 = local_829 * local_829;
	local_831 = local_830 + local_828;
	local_832 = sqrt(local_831);
	local_833 = local_832 * local_832;
	local_834 = local_833 * local_832;
	local_835 = (local_834 == 0) ? 0 : local_60 / local_834;
	local_836 = local_827 * local_835;
	local_837 = local_836 + local_826;
	local_838 = local_837 * local_4;
	local_839 = memory[2084 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_842 = const_840;
	else
		local_842 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_845 = const_843;
	else
		local_845 = local_842;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_848 = const_846;
	else
		local_848 = local_845;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_851 = const_849;
	else
		local_851 = local_848;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_853 = local_851;
	else
		local_853 = local_839;
	local_854 = local_853 * local_2;
	local_855 = local_801 + local_854;
	local_856 = local_855 + local_838;
	local_857 = local_18 - local_856;
	local_858 = local_857 * local_857;
	local_859 = local_819 * local_825;
	local_860 = local_829 * local_835;
	local_861 = local_860 + local_859;
	local_862 = local_861 * local_4;
	local_863 = memory[2083 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_866 = const_864;
	else
		local_866 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_869 = const_867;
	else
		local_869 = local_866;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_872 = const_870;
	else
		local_872 = local_869;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_875 = const_873;
	else
		local_875 = local_872;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_877 = local_875;
	else
		local_877 = local_863;
	local_878 = local_877 * local_2;
	local_879 = local_818 + local_878;
	local_880 = local_879 + local_862;
	local_881 = local_27 - local_880;
	local_882 = local_881 * local_881;
	local_883 = local_882 + local_858;
	local_884 = sqrt(local_883);
	local_885 = local_884 * local_884;
	local_886 = local_885 * local_884;
	local_887 = (local_886 == 0) ? 0 : local_39 / local_886;
	local_888 = local_857 * local_887;
	local_889 = local_44 - local_856;
	local_890 = local_889 * local_889;
	local_891 = local_49 - local_880;
	local_892 = local_891 * local_891;
	local_893 = local_892 + local_890;
	local_894 = sqrt(local_893);
	local_895 = local_894 * local_894;
	local_896 = local_895 * local_894;
	local_897 = (local_896 == 0) ? 0 : local_60 / local_896;
	local_898 = local_889 * local_897;
	local_899 = local_898 + local_888;
	local_900 = local_899 + local_837;
	local_901 = (const_0 == 0) ? 0 : local_900 / const_0;
	local_902 = local_901 * local_2;
	local_903 = local_853 + local_902;
	local_904 = local_881 * local_887;
	local_905 = local_891 * local_897;
	local_906 = local_905 + local_904;
	local_907 = local_906 + local_861;
	local_908 = (const_0 == 0) ? 0 : local_907 / const_0;
	local_909 = local_908 * local_2;
	local_910 = local_877 + local_909;
	local_911 = memory[2082 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_913 = local_276;
	else
		local_913 = local_911;
	local_914 = memory[2076 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_917 = const_915;
	else
		local_917 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_920 = const_918;
	else
		local_920 = local_917;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_923 = const_921;
	else
		local_923 = local_920;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_926 = const_924;
	else
		local_926 = local_923;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_928 = local_926;
	else
		local_928 = local_914;
	local_929 = local_18 - local_928;
	local_930 = local_929 * local_929;
	local_931 = memory[2075 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_934 = const_932;
	else
		local_934 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_937 = const_935;
	else
		local_937 = local_934;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_940 = const_938;
	else
		local_940 = local_937;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_943 = const_941;
	else
		local_943 = local_940;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_945 = local_943;
	else
		local_945 = local_931;
	local_946 = local_27 - local_945;
	local_947 = local_946 * local_946;
	local_948 = local_947 + local_930;
	local_949 = sqrt(local_948);
	local_950 = local_949 * local_949;
	local_951 = local_950 * local_949;
	local_952 = (local_951 == 0) ? 0 : local_39 / local_951;
	local_953 = local_929 * local_952;
	local_954 = local_44 - local_928;
	local_955 = local_954 * local_954;
	local_956 = local_49 - local_945;
	local_957 = local_956 * local_956;
	local_958 = local_957 + local_955;
	local_959 = sqrt(local_958);
	local_960 = local_959 * local_959;
	local_961 = local_960 * local_959;
	local_962 = (local_961 == 0) ? 0 : local_60 / local_961;
	local_963 = local_954 * local_962;
	local_964 = local_963 + local_953;
	local_965 = local_964 * local_4;
	local_966 = memory[2079 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_969 = const_967;
	else
		local_969 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_972 = const_970;
	else
		local_972 = local_969;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_975 = const_973;
	else
		local_975 = local_972;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_978 = const_976;
	else
		local_978 = local_975;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_980 = local_978;
	else
		local_980 = local_966;
	local_981 = local_980 * local_2;
	local_982 = local_928 + local_981;
	local_983 = local_982 + local_965;
	local_984 = local_18 - local_983;
	local_985 = local_984 * local_984;
	local_986 = local_946 * local_952;
	local_987 = local_956 * local_962;
	local_988 = local_987 + local_986;
	local_989 = local_988 * local_4;
	local_990 = memory[2078 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_993 = const_991;
	else
		local_993 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_996 = const_994;
	else
		local_996 = local_993;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_999 = const_997;
	else
		local_999 = local_996;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1002 = const_1000;
	else
		local_1002 = local_999;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1004 = local_1002;
	else
		local_1004 = local_990;
	local_1005 = local_1004 * local_2;
	local_1006 = local_945 + local_1005;
	local_1007 = local_1006 + local_989;
	local_1008 = local_27 - local_1007;
	local_1009 = local_1008 * local_1008;
	local_1010 = local_1009 + local_985;
	local_1011 = sqrt(local_1010);
	local_1012 = local_1011 * local_1011;
	local_1013 = local_1012 * local_1011;
	local_1014 = (local_1013 == 0) ? 0 : local_39 / local_1013;
	local_1015 = local_984 * local_1014;
	local_1016 = local_44 - local_983;
	local_1017 = local_1016 * local_1016;
	local_1018 = local_49 - local_1007;
	local_1019 = local_1018 * local_1018;
	local_1020 = local_1019 + local_1017;
	local_1021 = sqrt(local_1020);
	local_1022 = local_1021 * local_1021;
	local_1023 = local_1022 * local_1021;
	local_1024 = (local_1023 == 0) ? 0 : local_60 / local_1023;
	local_1025 = local_1016 * local_1024;
	local_1026 = local_1025 + local_1015;
	local_1027 = local_1026 + local_964;
	local_1028 = (const_0 == 0) ? 0 : local_1027 / const_0;
	local_1029 = local_1028 * local_2;
	local_1030 = local_980 + local_1029;
	local_1031 = local_1008 * local_1014;
	local_1032 = local_1018 * local_1024;
	local_1033 = local_1032 + local_1031;
	local_1034 = local_1033 + local_988;
	local_1035 = (const_0 == 0) ? 0 : local_1034 / const_0;
	local_1036 = local_1035 * local_2;
	local_1037 = local_1004 + local_1036;
	local_1038 = memory[2077 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1040 = local_276;
	else
		local_1040 = local_1038;
	local_1041 = memory[2071 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1044 = const_1042;
	else
		local_1044 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1047 = const_1045;
	else
		local_1047 = local_1044;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1050 = const_1048;
	else
		local_1050 = local_1047;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1053 = const_1051;
	else
		local_1053 = local_1050;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1055 = local_1053;
	else
		local_1055 = local_1041;
	local_1056 = local_18 - local_1055;
	local_1057 = local_1056 * local_1056;
	local_1058 = memory[2070 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1061 = const_1059;
	else
		local_1061 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1064 = const_1062;
	else
		local_1064 = local_1061;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1067 = const_1065;
	else
		local_1067 = local_1064;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1070 = const_1068;
	else
		local_1070 = local_1067;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1072 = local_1070;
	else
		local_1072 = local_1058;
	local_1073 = local_27 - local_1072;
	local_1074 = local_1073 * local_1073;
	local_1075 = local_1074 + local_1057;
	local_1076 = sqrt(local_1075);
	local_1077 = local_1076 * local_1076;
	local_1078 = local_1077 * local_1076;
	local_1079 = (local_1078 == 0) ? 0 : local_39 / local_1078;
	local_1080 = local_1056 * local_1079;
	local_1081 = local_44 - local_1055;
	local_1082 = local_1081 * local_1081;
	local_1083 = local_49 - local_1072;
	local_1084 = local_1083 * local_1083;
	local_1085 = local_1084 + local_1082;
	local_1086 = sqrt(local_1085);
	local_1087 = local_1086 * local_1086;
	local_1088 = local_1087 * local_1086;
	local_1089 = (local_1088 == 0) ? 0 : local_60 / local_1088;
	local_1090 = local_1081 * local_1089;
	local_1091 = local_1090 + local_1080;
	local_1092 = local_1091 * local_4;
	local_1093 = memory[2074 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1096 = const_1094;
	else
		local_1096 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1099 = const_1097;
	else
		local_1099 = local_1096;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1102 = const_1100;
	else
		local_1102 = local_1099;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1105 = const_1103;
	else
		local_1105 = local_1102;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1107 = local_1105;
	else
		local_1107 = local_1093;
	local_1108 = local_1107 * local_2;
	local_1109 = local_1055 + local_1108;
	local_1110 = local_1109 + local_1092;
	local_1111 = local_18 - local_1110;
	local_1112 = local_1111 * local_1111;
	local_1113 = local_1073 * local_1079;
	local_1114 = local_1083 * local_1089;
	local_1115 = local_1114 + local_1113;
	local_1116 = local_1115 * local_4;
	local_1117 = memory[2073 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1120 = const_1118;
	else
		local_1120 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1123 = const_1121;
	else
		local_1123 = local_1120;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1126 = const_1124;
	else
		local_1126 = local_1123;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1129 = const_1127;
	else
		local_1129 = local_1126;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1131 = local_1129;
	else
		local_1131 = local_1117;
	local_1132 = local_1131 * local_2;
	local_1133 = local_1072 + local_1132;
	local_1134 = local_1133 + local_1116;
	local_1135 = local_27 - local_1134;
	local_1136 = local_1135 * local_1135;
	local_1137 = local_1136 + local_1112;
	local_1138 = sqrt(local_1137);
	local_1139 = local_1138 * local_1138;
	local_1140 = local_1139 * local_1138;
	local_1141 = (local_1140 == 0) ? 0 : local_39 / local_1140;
	local_1142 = local_1111 * local_1141;
	local_1143 = local_44 - local_1110;
	local_1144 = local_1143 * local_1143;
	local_1145 = local_49 - local_1134;
	local_1146 = local_1145 * local_1145;
	local_1147 = local_1146 + local_1144;
	local_1148 = sqrt(local_1147);
	local_1149 = local_1148 * local_1148;
	local_1150 = local_1149 * local_1148;
	local_1151 = (local_1150 == 0) ? 0 : local_60 / local_1150;
	local_1152 = local_1143 * local_1151;
	local_1153 = local_1152 + local_1142;
	local_1154 = local_1153 + local_1091;
	local_1155 = (const_0 == 0) ? 0 : local_1154 / const_0;
	local_1156 = local_1155 * local_2;
	local_1157 = local_1107 + local_1156;
	local_1158 = local_1135 * local_1141;
	local_1159 = local_1145 * local_1151;
	local_1160 = local_1159 + local_1158;
	local_1161 = local_1160 + local_1115;
	local_1162 = (const_0 == 0) ? 0 : local_1161 / const_0;
	local_1163 = local_1162 * local_2;
	local_1164 = local_1131 + local_1163;
	local_1165 = memory[2072 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1167 = local_276;
	else
		local_1167 = local_1165;
	local_1168 = memory[2066 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1171 = const_1169;
	else
		local_1171 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1174 = const_1172;
	else
		local_1174 = local_1171;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1177 = const_1175;
	else
		local_1177 = local_1174;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1180 = const_1178;
	else
		local_1180 = local_1177;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1182 = local_1180;
	else
		local_1182 = local_1168;
	local_1183 = local_18 - local_1182;
	local_1184 = local_1183 * local_1183;
	local_1185 = memory[2065 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1188 = const_1186;
	else
		local_1188 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1191 = const_1189;
	else
		local_1191 = local_1188;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1194 = const_1192;
	else
		local_1194 = local_1191;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1197 = const_1195;
	else
		local_1197 = local_1194;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1199 = local_1197;
	else
		local_1199 = local_1185;
	local_1200 = local_27 - local_1199;
	local_1201 = local_1200 * local_1200;
	local_1202 = local_1201 + local_1184;
	local_1203 = sqrt(local_1202);
	local_1204 = local_1203 * local_1203;
	local_1205 = local_1204 * local_1203;
	local_1206 = (local_1205 == 0) ? 0 : local_39 / local_1205;
	local_1207 = local_1183 * local_1206;
	local_1208 = local_44 - local_1182;
	local_1209 = local_1208 * local_1208;
	local_1210 = local_49 - local_1199;
	local_1211 = local_1210 * local_1210;
	local_1212 = local_1211 + local_1209;
	local_1213 = sqrt(local_1212);
	local_1214 = local_1213 * local_1213;
	local_1215 = local_1214 * local_1213;
	local_1216 = (local_1215 == 0) ? 0 : local_60 / local_1215;
	local_1217 = local_1208 * local_1216;
	local_1218 = local_1217 + local_1207;
	local_1219 = local_1218 * local_4;
	local_1220 = memory[2069 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1223 = const_1221;
	else
		local_1223 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1226 = const_1224;
	else
		local_1226 = local_1223;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1229 = const_1227;
	else
		local_1229 = local_1226;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1232 = const_1230;
	else
		local_1232 = local_1229;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1234 = local_1232;
	else
		local_1234 = local_1220;
	local_1235 = local_1234 * local_2;
	local_1236 = local_1182 + local_1235;
	local_1237 = local_1236 + local_1219;
	local_1238 = local_18 - local_1237;
	local_1239 = local_1238 * local_1238;
	local_1240 = local_1200 * local_1206;
	local_1241 = local_1210 * local_1216;
	local_1242 = local_1241 + local_1240;
	local_1243 = local_1242 * local_4;
	local_1244 = memory[2068 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1247 = const_1245;
	else
		local_1247 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1250 = const_1248;
	else
		local_1250 = local_1247;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1253 = const_1251;
	else
		local_1253 = local_1250;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1256 = const_1254;
	else
		local_1256 = local_1253;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1258 = local_1256;
	else
		local_1258 = local_1244;
	local_1259 = local_1258 * local_2;
	local_1260 = local_1199 + local_1259;
	local_1261 = local_1260 + local_1243;
	local_1262 = local_27 - local_1261;
	local_1263 = local_1262 * local_1262;
	local_1264 = local_1263 + local_1239;
	local_1265 = sqrt(local_1264);
	local_1266 = local_1265 * local_1265;
	local_1267 = local_1266 * local_1265;
	local_1268 = (local_1267 == 0) ? 0 : local_39 / local_1267;
	local_1269 = local_1238 * local_1268;
	local_1270 = local_44 - local_1237;
	local_1271 = local_1270 * local_1270;
	local_1272 = local_49 - local_1261;
	local_1273 = local_1272 * local_1272;
	local_1274 = local_1273 + local_1271;
	local_1275 = sqrt(local_1274);
	local_1276 = local_1275 * local_1275;
	local_1277 = local_1276 * local_1275;
	local_1278 = (local_1277 == 0) ? 0 : local_60 / local_1277;
	local_1279 = local_1270 * local_1278;
	local_1280 = local_1279 + local_1269;
	local_1281 = local_1280 + local_1218;
	local_1282 = (const_0 == 0) ? 0 : local_1281 / const_0;
	local_1283 = local_1282 * local_2;
	local_1284 = local_1234 + local_1283;
	local_1285 = local_1262 * local_1268;
	local_1286 = local_1272 * local_1278;
	local_1287 = local_1286 + local_1285;
	local_1288 = local_1287 + local_1242;
	local_1289 = (const_0 == 0) ? 0 : local_1288 / const_0;
	local_1290 = local_1289 * local_2;
	local_1291 = local_1258 + local_1290;
	local_1292 = memory[2067 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1294 = local_276;
	else
		local_1294 = local_1292;
	local_1295 = memory[2061 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1298 = const_1296;
	else
		local_1298 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1301 = const_1299;
	else
		local_1301 = local_1298;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1304 = const_1302;
	else
		local_1304 = local_1301;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1307 = const_1305;
	else
		local_1307 = local_1304;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1309 = local_1307;
	else
		local_1309 = local_1295;
	local_1310 = local_18 - local_1309;
	local_1311 = local_1310 * local_1310;
	local_1312 = memory[2060 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1315 = const_1313;
	else
		local_1315 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1318 = const_1316;
	else
		local_1318 = local_1315;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1321 = const_1319;
	else
		local_1321 = local_1318;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1324 = const_1322;
	else
		local_1324 = local_1321;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1326 = local_1324;
	else
		local_1326 = local_1312;
	local_1327 = local_27 - local_1326;
	local_1328 = local_1327 * local_1327;
	local_1329 = local_1328 + local_1311;
	local_1330 = sqrt(local_1329);
	local_1331 = local_1330 * local_1330;
	local_1332 = local_1331 * local_1330;
	local_1333 = (local_1332 == 0) ? 0 : local_39 / local_1332;
	local_1334 = local_1310 * local_1333;
	local_1335 = local_44 - local_1309;
	local_1336 = local_1335 * local_1335;
	local_1337 = local_49 - local_1326;
	local_1338 = local_1337 * local_1337;
	local_1339 = local_1338 + local_1336;
	local_1340 = sqrt(local_1339);
	local_1341 = local_1340 * local_1340;
	local_1342 = local_1341 * local_1340;
	local_1343 = (local_1342 == 0) ? 0 : local_60 / local_1342;
	local_1344 = local_1335 * local_1343;
	local_1345 = local_1344 + local_1334;
	local_1346 = local_1345 * local_4;
	local_1347 = memory[2064 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1350 = const_1348;
	else
		local_1350 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1353 = const_1351;
	else
		local_1353 = local_1350;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1356 = const_1354;
	else
		local_1356 = local_1353;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1359 = const_1357;
	else
		local_1359 = local_1356;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1361 = local_1359;
	else
		local_1361 = local_1347;
	local_1362 = local_1361 * local_2;
	local_1363 = local_1309 + local_1362;
	local_1364 = local_1363 + local_1346;
	local_1365 = local_18 - local_1364;
	local_1366 = local_1365 * local_1365;
	local_1367 = local_1327 * local_1333;
	local_1368 = local_1337 * local_1343;
	local_1369 = local_1368 + local_1367;
	local_1370 = local_1369 * local_4;
	local_1371 = memory[2063 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1374 = const_1372;
	else
		local_1374 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1377 = const_1375;
	else
		local_1377 = local_1374;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1380 = const_1378;
	else
		local_1380 = local_1377;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1383 = const_1381;
	else
		local_1383 = local_1380;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1385 = local_1383;
	else
		local_1385 = local_1371;
	local_1386 = local_1385 * local_2;
	local_1387 = local_1326 + local_1386;
	local_1388 = local_1387 + local_1370;
	local_1389 = local_27 - local_1388;
	local_1390 = local_1389 * local_1389;
	local_1391 = local_1390 + local_1366;
	local_1392 = sqrt(local_1391);
	local_1393 = local_1392 * local_1392;
	local_1394 = local_1393 * local_1392;
	local_1395 = (local_1394 == 0) ? 0 : local_39 / local_1394;
	local_1396 = local_1365 * local_1395;
	local_1397 = local_44 - local_1364;
	local_1398 = local_1397 * local_1397;
	local_1399 = local_49 - local_1388;
	local_1400 = local_1399 * local_1399;
	local_1401 = local_1400 + local_1398;
	local_1402 = sqrt(local_1401);
	local_1403 = local_1402 * local_1402;
	local_1404 = local_1403 * local_1402;
	local_1405 = (local_1404 == 0) ? 0 : local_60 / local_1404;
	local_1406 = local_1397 * local_1405;
	local_1407 = local_1406 + local_1396;
	local_1408 = local_1407 + local_1345;
	local_1409 = (const_0 == 0) ? 0 : local_1408 / const_0;
	local_1410 = local_1409 * local_2;
	local_1411 = local_1361 + local_1410;
	local_1412 = local_1389 * local_1395;
	local_1413 = local_1399 * local_1405;
	local_1414 = local_1413 + local_1412;
	local_1415 = local_1414 + local_1369;
	local_1416 = (const_0 == 0) ? 0 : local_1415 / const_0;
	local_1417 = local_1416 * local_2;
	local_1418 = local_1385 + local_1417;
	local_1419 = memory[2062 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1421 = local_276;
	else
		local_1421 = local_1419;
	local_1422 = memory[2056 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1424 = const_1103;
	else
		local_1424 = local_1422;
	local_1425 = local_18 - local_1424;
	local_1426 = local_1425 * local_1425;
	local_1427 = memory[2055 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1430 = const_1428;
	else
		local_1430 = local_1427;
	local_1431 = local_27 - local_1430;
	local_1432 = local_1431 * local_1431;
	local_1433 = local_1432 + local_1426;
	local_1434 = sqrt(local_1433);
	local_1435 = local_1434 * local_1434;
	local_1436 = local_1435 * local_1434;
	local_1437 = (local_1436 == 0) ? 0 : local_39 / local_1436;
	local_1438 = local_1425 * local_1437;
	local_1439 = local_44 - local_1424;
	local_1440 = local_1439 * local_1439;
	local_1441 = local_49 - local_1430;
	local_1442 = local_1441 * local_1441;
	local_1443 = local_1442 + local_1440;
	local_1444 = sqrt(local_1443);
	local_1445 = local_1444 * local_1444;
	local_1446 = local_1445 * local_1444;
	local_1447 = (local_1446 == 0) ? 0 : local_60 / local_1446;
	local_1448 = local_1439 * local_1447;
	local_1449 = local_1448 + local_1438;
	local_1450 = local_1449 * local_4;
	local_1451 = memory[2059 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1454 = const_1452;
	else
		local_1454 = local_1451;
	local_1455 = local_1454 * local_2;
	local_1456 = local_1424 + local_1455;
	local_1457 = local_1456 + local_1450;
	local_1458 = local_18 - local_1457;
	local_1459 = local_1458 * local_1458;
	local_1460 = local_1431 * local_1437;
	local_1461 = local_1441 * local_1447;
	local_1462 = local_1461 + local_1460;
	local_1463 = local_1462 * local_4;
	local_1464 = memory[2058 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1466 = const_1103;
	else
		local_1466 = local_1464;
	local_1467 = local_1466 * local_2;
	local_1468 = local_1430 + local_1467;
	local_1469 = local_1468 + local_1463;
	local_1470 = local_27 - local_1469;
	local_1471 = local_1470 * local_1470;
	local_1472 = local_1471 + local_1459;
	local_1473 = sqrt(local_1472);
	local_1474 = local_1473 * local_1473;
	local_1475 = local_1474 * local_1473;
	local_1476 = (local_1475 == 0) ? 0 : local_39 / local_1475;
	local_1477 = local_1458 * local_1476;
	local_1478 = local_44 - local_1457;
	local_1479 = local_1478 * local_1478;
	local_1480 = local_49 - local_1469;
	local_1481 = local_1480 * local_1480;
	local_1482 = local_1481 + local_1479;
	local_1483 = sqrt(local_1482);
	local_1484 = local_1483 * local_1483;
	local_1485 = local_1484 * local_1483;
	local_1486 = (local_1485 == 0) ? 0 : local_60 / local_1485;
	local_1487 = local_1478 * local_1486;
	local_1488 = local_1487 + local_1477;
	local_1489 = local_1488 + local_1449;
	local_1490 = (const_0 == 0) ? 0 : local_1489 / const_0;
	local_1491 = local_1490 * local_2;
	local_1492 = local_1454 + local_1491;
	local_1493 = local_1470 * local_1476;
	local_1494 = local_1480 * local_1486;
	local_1495 = local_1494 + local_1493;
	local_1496 = local_1495 + local_1462;
	local_1497 = (const_0 == 0) ? 0 : local_1496 / const_0;
	local_1498 = local_1497 * local_2;
	local_1499 = local_1466 + local_1498;
	local_1500 = memory[2057 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1502 = const_268;
	else
		local_1502 = local_1500;
	local_1503 = local_44 - local_18;
	local_1504 = local_1503 * local_1503;
	local_1505 = local_49 - local_27;
	local_1506 = local_1505 * local_1505;
	local_1507 = local_1506 + local_1504;
	local_1508 = sqrt(local_1507);
	local_1509 = local_1508 * local_1508;
	local_1510 = local_1509 * local_1508;
	local_1511 = (local_1510 == 0) ? 0 : local_60 / local_1510;
	local_1512 = local_1503 * local_1511;
	local_1513 = local_1512 * local_4;
	local_1514 = memory[2054 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1516 = local_76;
	else
		local_1516 = local_1514;
	local_1517 = local_1516 * local_2;
	local_1518 = local_18 + local_1517;
	local_1519 = local_1518 + local_1513;
	local_1520 = local_44 - local_1519;
	local_1521 = local_1520 * local_1520;
	local_1522 = local_1505 * local_1511;
	local_1523 = local_1522 * local_4;
	local_1524 = memory[2053 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1526 = local_91;
	else
		local_1526 = local_1524;
	local_1527 = local_1526 * local_2;
	local_1528 = local_27 + local_1527;
	local_1529 = local_1528 + local_1523;
	local_1530 = local_49 - local_1529;
	local_1531 = local_1530 * local_1530;
	local_1532 = local_1531 + local_1521;
	local_1533 = sqrt(local_1532);
	local_1534 = local_1533 * local_1533;
	local_1535 = local_1534 * local_1533;
	local_1536 = (local_1535 == 0) ? 0 : local_60 / local_1535;
	local_1537 = local_1520 * local_1536;
	local_1538 = local_1537 + local_1512;
	local_1539 = (const_0 == 0) ? 0 : local_1538 / const_0;
	local_1540 = local_1539 * local_2;
	local_1541 = local_1516 + local_1540;
	local_1542 = local_1530 * local_1536;
	local_1543 = local_1542 + local_1522;
	local_1544 = (const_0 == 0) ? 0 : local_1543 / const_0;
	local_1545 = local_1544 * local_2;
	local_1546 = local_1526 + local_1545;
	local_1547 = memory[2046 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1550 = const_1548;
	else
		local_1550 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1552 = const_1103;
	else
		local_1552 = local_1550;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1555 = const_1553;
	else
		local_1555 = local_1552;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1557 = const_1103;
	else
		local_1557 = local_1555;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1559 = local_1557;
	else
		local_1559 = local_1547;
	local_1560 = local_18 - local_1559;
	local_1561 = local_1560 * local_1560;
	local_1562 = memory[2045 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1564 = const_1103;
	else
		local_1564 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1567 = const_1565;
	else
		local_1567 = local_1564;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1570 = const_1568;
	else
		local_1570 = local_1567;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1573 = const_1571;
	else
		local_1573 = local_1570;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1575 = local_1573;
	else
		local_1575 = local_1562;
	local_1576 = local_27 - local_1575;
	local_1577 = local_1576 * local_1576;
	local_1578 = local_1577 + local_1561;
	local_1579 = sqrt(local_1578);
	local_1580 = local_1579 * local_1579;
	local_1581 = local_1580 * local_1579;
	local_1582 = (local_1581 == 0) ? 0 : local_39 / local_1581;
	local_1583 = local_1560 * local_1582;
	local_1584 = local_44 - local_1559;
	local_1585 = local_1584 * local_1584;
	local_1586 = local_49 - local_1575;
	local_1587 = local_1586 * local_1586;
	local_1588 = local_1587 + local_1585;
	local_1589 = sqrt(local_1588);
	local_1590 = local_1589 * local_1589;
	local_1591 = local_1590 * local_1589;
	local_1592 = (local_1591 == 0) ? 0 : local_60 / local_1591;
	local_1593 = local_1584 * local_1592;
	local_1594 = local_1593 + local_1583;
	local_1595 = input_ports[3];
	local_1596 = (local_2 == 0) ? 0 : local_1595 / local_2;
	local_1597 = local_1596 + local_1594;
	local_1598 = local_1597 * local_4;
	local_1599 = memory[2049 - min_global];
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1602 = const_1600;
	else
		local_1602 = local_1564;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1605 = const_1603;
	else
		local_1605 = local_1602;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1608 = const_1606;
	else
		local_1608 = local_1605;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1610 = local_1608;
	else
		local_1610 = local_1599;
	local_1611 = local_1610 * local_2;
	local_1612 = local_1559 + local_1611;
	local_1613 = local_1612 + local_1598;
	local_1614 = local_18 - local_1613;
	local_1615 = local_1614 * local_1614;
	local_1616 = local_1576 * local_1582;
	local_1617 = local_1586 * local_1592;
	local_1618 = local_1617 + local_1616;
	local_1619 = input_ports[2];
	local_1620 = (local_2 == 0) ? 0 : local_1619 / local_2;
	local_1621 = local_1620 + local_1618;
	local_1622 = local_1621 * local_4;
	local_1623 = memory[2048 - min_global];
	lstatus = (local_138 == 0);
	if (lstatus)
		local_1626 = const_1624;
	else
		local_1626 = const_11;
	lstatus = (local_143 == 0);
	if (lstatus)
		local_1628 = const_1103;
	else
		local_1628 = local_1626;
	lstatus = (local_148 == 0);
	if (lstatus)
		local_1630 = const_1603;
	else
		local_1630 = local_1628;
	lstatus = (local_153 == 0);
	if (lstatus)
		local_1632 = const_1103;
	else
		local_1632 = local_1630;
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1634 = local_1632;
	else
		local_1634 = local_1623;
	local_1635 = local_1634 * local_2;
	local_1636 = local_1575 + local_1635;
	local_1637 = local_1636 + local_1622;
	local_1638 = local_27 - local_1637;
	local_1639 = local_1638 * local_1638;
	local_1640 = local_1639 + local_1615;
	local_1641 = sqrt(local_1640);
	local_1642 = local_1641 * local_1641;
	local_1643 = local_1642 * local_1641;
	local_1644 = (local_1643 == 0) ? 0 : local_39 / local_1643;
	local_1645 = local_1614 * local_1644;
	local_1646 = local_44 - local_1613;
	local_1647 = local_1646 * local_1646;
	local_1648 = local_49 - local_1637;
	local_1649 = local_1648 * local_1648;
	local_1650 = local_1649 + local_1647;
	local_1651 = sqrt(local_1650);
	local_1652 = local_1651 * local_1651;
	local_1653 = local_1652 * local_1651;
	local_1654 = (local_1653 == 0) ? 0 : local_60 / local_1653;
	local_1655 = local_1646 * local_1654;
	local_1656 = local_1655 + local_1645;
	local_1657 = local_1656 + local_1594;
	local_1658 = (const_0 == 0) ? 0 : local_1657 / const_0;
	local_1659 = local_1596 + local_1658;
	local_1660 = local_1659 * local_2;
	local_1661 = local_1610 + local_1660;
	local_1662 = local_1638 * local_1644;
	local_1663 = local_1648 * local_1654;
	local_1664 = local_1663 + local_1662;
	local_1665 = local_1664 + local_1618;
	local_1666 = (const_0 == 0) ? 0 : local_1665 / const_0;
	local_1667 = local_1620 + local_1666;
	local_1668 = local_1667 * local_2;
	local_1669 = local_1634 + local_1668;
	local_1670 = memory[2047 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1672 = local_276;
	else
		local_1672 = local_1670;
	local_1673 = memory[2044 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1675 = const_11;
	else
		local_1675 = local_1673;
	local_1676 = memory[2043 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1678 = const_11;
	else
		local_1678 = local_1676;
	local_1679 = local_12 + const_1;
	local_1680 = local_1519 - local_1613;
	local_1681 = local_1529 - local_1637;
	local_1682 = local_1613 - local_82;
	local_1683 = local_1682 * local_1682;
	local_1684 = local_1637 - local_100;
	local_1685 = local_1684 * local_1684;
	local_1686 = local_1685 + local_1683;
	local_1687 = sqrt(local_1686);
	local_1689 = local_1687 - const_1688;
	local_1690 = memory[2127 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1692 = const_11;
	else
		local_1692 = local_1690;
	local_1694 = const_1693 - local_12;
	lstatus = (local_1689 < 0);
	if (lstatus)
		local_1696 = local_1694;
	else
		local_1696 = local_1692;
	local_1697 = local_1692 - const_11;
	lstatus = (local_1697 == 0);
	if (lstatus)
		local_1699 = local_1696;
	else
		local_1699 = local_1692;
	local_1700 = local_1699 - const_11;
	lstatus = (local_1700 == 0);
	if (lstatus)
		local_1702 = const_11;
	else
		local_1702 = const_1;
	local_1703 = local_82 - local_1613;
	local_1704 = local_100 - local_1637;
	local_1705 = local_1613 - local_212;
	local_1706 = local_1705 * local_1705;
	local_1707 = local_1637 - local_236;
	local_1708 = local_1707 * local_1707;
	local_1709 = local_1708 + local_1706;
	local_1710 = sqrt(local_1709);
	local_1711 = local_1710 - const_1688;
	local_1712 = memory[2126 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1714 = const_11;
	else
		local_1714 = local_1712;
	lstatus = (local_1711 < 0);
	if (lstatus)
		local_1716 = local_1694;
	else
		local_1716 = local_1714;
	local_1717 = local_1714 - const_11;
	lstatus = (local_1717 == 0);
	if (lstatus)
		local_1719 = local_1716;
	else
		local_1719 = local_1714;
	local_1720 = local_1719 - const_11;
	lstatus = (local_1720 == 0);
	if (lstatus)
		local_1722 = const_11;
	else
		local_1722 = const_1;
	local_1723 = local_212 - local_1613;
	local_1724 = local_236 - local_1637;
	local_1725 = local_1613 - local_348;
	local_1726 = local_1725 * local_1725;
	local_1727 = local_1637 - local_372;
	local_1728 = local_1727 * local_1727;
	local_1729 = local_1728 + local_1726;
	local_1730 = sqrt(local_1729);
	local_1731 = local_1730 - const_1688;
	local_1732 = memory[2125 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1734 = const_11;
	else
		local_1734 = local_1732;
	lstatus = (local_1731 < 0);
	if (lstatus)
		local_1736 = local_1694;
	else
		local_1736 = local_1734;
	local_1737 = local_1734 - const_11;
	lstatus = (local_1737 == 0);
	if (lstatus)
		local_1739 = local_1736;
	else
		local_1739 = local_1734;
	local_1740 = local_1739 - const_11;
	lstatus = (local_1740 == 0);
	if (lstatus)
		local_1742 = const_11;
	else
		local_1742 = const_1;
	local_1743 = local_348 - local_1613;
	local_1744 = local_372 - local_1637;
	local_1745 = local_1613 - local_475;
	local_1746 = local_1745 * local_1745;
	local_1747 = local_1637 - local_499;
	local_1748 = local_1747 * local_1747;
	local_1749 = local_1748 + local_1746;
	local_1750 = sqrt(local_1749);
	local_1751 = local_1750 - const_1688;
	local_1752 = memory[2124 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1754 = const_11;
	else
		local_1754 = local_1752;
	lstatus = (local_1751 < 0);
	if (lstatus)
		local_1756 = local_1694;
	else
		local_1756 = local_1754;
	local_1757 = local_1754 - const_11;
	lstatus = (local_1757 == 0);
	if (lstatus)
		local_1759 = local_1756;
	else
		local_1759 = local_1754;
	local_1760 = local_1759 - const_11;
	lstatus = (local_1760 == 0);
	if (lstatus)
		local_1762 = const_11;
	else
		local_1762 = const_1;
	local_1763 = local_475 - local_1613;
	local_1764 = local_499 - local_1637;
	local_1765 = local_1613 - local_602;
	local_1766 = local_1765 * local_1765;
	local_1767 = local_1637 - local_626;
	local_1768 = local_1767 * local_1767;
	local_1769 = local_1768 + local_1766;
	local_1770 = sqrt(local_1769);
	local_1771 = local_1770 - const_1688;
	local_1772 = memory[2123 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1774 = const_11;
	else
		local_1774 = local_1772;
	lstatus = (local_1771 < 0);
	if (lstatus)
		local_1776 = local_1694;
	else
		local_1776 = local_1774;
	local_1777 = local_1774 - const_11;
	lstatus = (local_1777 == 0);
	if (lstatus)
		local_1779 = local_1776;
	else
		local_1779 = local_1774;
	local_1780 = local_1779 - const_11;
	lstatus = (local_1780 == 0);
	if (lstatus)
		local_1782 = const_11;
	else
		local_1782 = const_1;
	local_1783 = local_602 - local_1613;
	local_1784 = local_626 - local_1637;
	local_1785 = local_1613 - local_729;
	local_1786 = local_1785 * local_1785;
	local_1787 = local_1637 - local_753;
	local_1788 = local_1787 * local_1787;
	local_1789 = local_1788 + local_1786;
	local_1790 = sqrt(local_1789);
	local_1791 = local_1790 - const_1688;
	local_1792 = memory[2122 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1794 = const_11;
	else
		local_1794 = local_1792;
	lstatus = (local_1791 < 0);
	if (lstatus)
		local_1796 = local_1694;
	else
		local_1796 = local_1794;
	local_1797 = local_1794 - const_11;
	lstatus = (local_1797 == 0);
	if (lstatus)
		local_1799 = local_1796;
	else
		local_1799 = local_1794;
	local_1800 = local_1799 - const_11;
	lstatus = (local_1800 == 0);
	if (lstatus)
		local_1802 = const_11;
	else
		local_1802 = const_1;
	local_1803 = local_729 - local_1613;
	local_1804 = local_753 - local_1637;
	local_1805 = local_1613 - local_856;
	local_1806 = local_1805 * local_1805;
	local_1807 = local_1637 - local_880;
	local_1808 = local_1807 * local_1807;
	local_1809 = local_1808 + local_1806;
	local_1810 = sqrt(local_1809);
	local_1811 = local_1810 - const_1688;
	local_1812 = memory[2121 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1814 = const_11;
	else
		local_1814 = local_1812;
	lstatus = (local_1811 < 0);
	if (lstatus)
		local_1816 = local_1694;
	else
		local_1816 = local_1814;
	local_1817 = local_1814 - const_11;
	lstatus = (local_1817 == 0);
	if (lstatus)
		local_1819 = local_1816;
	else
		local_1819 = local_1814;
	local_1820 = local_1819 - const_11;
	lstatus = (local_1820 == 0);
	if (lstatus)
		local_1822 = const_11;
	else
		local_1822 = const_1;
	local_1823 = local_856 - local_1613;
	local_1824 = local_880 - local_1637;
	local_1825 = local_1613 - local_983;
	local_1826 = local_1825 * local_1825;
	local_1827 = local_1637 - local_1007;
	local_1828 = local_1827 * local_1827;
	local_1829 = local_1828 + local_1826;
	local_1830 = sqrt(local_1829);
	local_1831 = local_1830 - const_1688;
	local_1832 = memory[2120 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1834 = const_11;
	else
		local_1834 = local_1832;
	lstatus = (local_1831 < 0);
	if (lstatus)
		local_1836 = local_1694;
	else
		local_1836 = local_1834;
	local_1837 = local_1834 - const_11;
	lstatus = (local_1837 == 0);
	if (lstatus)
		local_1839 = local_1836;
	else
		local_1839 = local_1834;
	local_1840 = local_1839 - const_11;
	lstatus = (local_1840 == 0);
	if (lstatus)
		local_1842 = const_11;
	else
		local_1842 = const_1;
	local_1843 = local_983 - local_1613;
	local_1844 = local_1007 - local_1637;
	local_1845 = local_1613 - local_1110;
	local_1846 = local_1845 * local_1845;
	local_1847 = local_1637 - local_1134;
	local_1848 = local_1847 * local_1847;
	local_1849 = local_1848 + local_1846;
	local_1850 = sqrt(local_1849);
	local_1851 = local_1850 - const_1688;
	local_1852 = memory[2119 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1854 = const_11;
	else
		local_1854 = local_1852;
	lstatus = (local_1851 < 0);
	if (lstatus)
		local_1856 = local_1694;
	else
		local_1856 = local_1854;
	local_1857 = local_1854 - const_11;
	lstatus = (local_1857 == 0);
	if (lstatus)
		local_1859 = local_1856;
	else
		local_1859 = local_1854;
	local_1860 = local_1859 - const_11;
	lstatus = (local_1860 == 0);
	if (lstatus)
		local_1862 = const_11;
	else
		local_1862 = const_1;
	local_1863 = local_1110 - local_1613;
	local_1864 = local_1134 - local_1637;
	local_1865 = local_1613 - local_1237;
	local_1866 = local_1865 * local_1865;
	local_1867 = local_1637 - local_1261;
	local_1868 = local_1867 * local_1867;
	local_1869 = local_1868 + local_1866;
	local_1870 = sqrt(local_1869);
	local_1871 = local_1870 - const_1688;
	local_1872 = memory[2118 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1874 = const_11;
	else
		local_1874 = local_1872;
	lstatus = (local_1871 < 0);
	if (lstatus)
		local_1876 = local_1694;
	else
		local_1876 = local_1874;
	local_1877 = local_1874 - const_11;
	lstatus = (local_1877 == 0);
	if (lstatus)
		local_1879 = local_1876;
	else
		local_1879 = local_1874;
	local_1880 = local_1879 - const_11;
	lstatus = (local_1880 == 0);
	if (lstatus)
		local_1882 = const_11;
	else
		local_1882 = const_1;
	local_1883 = local_1237 - local_1613;
	local_1884 = local_1261 - local_1637;
	local_1885 = local_1613 - local_1364;
	local_1886 = local_1885 * local_1885;
	local_1887 = local_1637 - local_1388;
	local_1888 = local_1887 * local_1887;
	local_1889 = local_1888 + local_1886;
	local_1890 = sqrt(local_1889);
	local_1891 = local_1890 - const_1688;
	local_1892 = memory[2117 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1894 = const_11;
	else
		local_1894 = local_1892;
	lstatus = (local_1891 < 0);
	if (lstatus)
		local_1896 = local_1694;
	else
		local_1896 = local_1894;
	local_1897 = local_1894 - const_11;
	lstatus = (local_1897 == 0);
	if (lstatus)
		local_1899 = local_1896;
	else
		local_1899 = local_1894;
	local_1900 = local_1899 - const_11;
	lstatus = (local_1900 == 0);
	if (lstatus)
		local_1902 = const_11;
	else
		local_1902 = const_1;
	local_1903 = local_1364 - local_1613;
	local_1904 = local_1388 - local_1637;
	local_1905 = local_1457 - local_1613;
	local_1906 = local_1469 - local_1637;
	local_1907 = local_1613 - local_1457;
	local_1908 = local_1907 * local_1907;
	local_1909 = local_1637 - local_1469;
	local_1910 = local_1909 * local_1909;
	local_1911 = local_1910 + local_1908;
	local_1912 = sqrt(local_1911);
	local_1913 = local_1912 - const_1688;
	local_1914 = memory[2116 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1916 = const_11;
	else
		local_1916 = local_1914;
	lstatus = (local_1913 < 0);
	if (lstatus)
		local_1918 = local_1694;
	else
		local_1918 = local_1916;
	local_1919 = local_1916 - const_11;
	lstatus = (local_1919 == 0);
	if (lstatus)
		local_1921 = local_1918;
	else
		local_1921 = local_1916;
	local_1922 = memory[2128 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1925 = const_1923;
	else
		local_1925 = local_1922;
	local_1926 = memory[2115 - min_global];
	lstatus = (local_13 == 0);
	if (lstatus)
		local_1929 = const_1927;
	else
		local_1929 = local_1926;
	local_1930 = local_1596 * local_1596;
	local_1931 = local_1620 * local_1620;
	local_1932 = local_1931 + local_1930;
	local_1933 = sqrt(local_1932);
	local_1934 = local_1933 * local_2;
	local_1935 = local_1929 - local_1934;
	local_1936 = const_1927 - local_1935;
	local_1937 = local_1936 - local_1925;
	lstatus = (local_1937 < 0);
	if (lstatus)
		local_1939 = local_1936;
	else
		local_1939 = local_1925;
	lstatus = (local_1913 < 0);
	if (lstatus)
		local_1941 = local_1939;
	else
		local_1941 = const_11;
	local_1942 = local_1925 - local_1941;
	local_1943 = local_1935 + local_1941;
	local_1946 = local_1719 + local_1699;
	local_1947 = local_1739 + local_1946;
	local_1948 = local_1759 + local_1947;
	local_1949 = local_1779 + local_1948;
	local_1950 = local_1799 + local_1949;
	local_1951 = local_1819 + local_1950;
	local_1952 = local_1839 + local_1951;
	local_1953 = local_1859 + local_1952;
	local_1954 = local_1879 + local_1953;
	local_1955 = local_1899 + local_1954;
	local_1956 = local_1921 + local_1955;
	local_1957 = (const_1945 == 0) ? 0 : local_1956 / const_1945;
	local_1959 = const_1958 * local_1957;
	local_1960 = const_1927 + const_1923;
	local_1961 = local_1942 + local_1943;
	local_1962 = (local_1960 == 0) ? 0 : local_1961 / local_1960;
	local_1964 = const_1963 * local_1962;
	local_1965 = local_1964 + local_1959;
	local_1966 = local_1965 * const_1944;
	local_1967 = local_1694 - const_11;
	lstatus = (local_1967 == 0);
	if (lstatus)
		local_1969 = local_1966;
	else
		local_1969 = const_11;
	local_1970 = local_1943 - const_11;
	local_1971 = const_1996 - const_1;
	lstatus = (local_1970 < 0);
	if (lstatus)
		local_1973 = local_1971;
	else
		local_1973 = local_1969;
	local_1974 = const_1927 - local_1934;
	lstatus = (local_1974 < 0);
	if (lstatus)
		local_1976 = local_1971;
	else
		local_1976 = local_1973;
	local_1977 = local_1613 - local_18;
	local_1978 = local_1977 * local_1977;
	local_1979 = local_1637 - local_27;
	local_1980 = local_1979 * local_1979;
	local_1981 = local_1980 + local_1978;
	local_1982 = sqrt(local_1981);
	local_1983 = local_1982 - const_7;
	lstatus = (local_1983 < 0);
	if (lstatus)
		local_1985 = local_1971;
	else
		local_1985 = local_1976;
	local_1987 = local_1613 - local_44;
	local_1988 = local_1987 * local_1987;
	local_1989 = local_1637 - local_49;
	local_1990 = local_1989 * local_1989;
	local_1991 = local_1990 + local_1988;
	local_1992 = sqrt(local_1991);
	local_1993 = local_1992 - const_1986;
	lstatus = (local_1993 < 0);
	if (lstatus)
		local_1995 = local_1971;
	else
		local_1995 = local_1985;
	output_ports[0] = local_1995;
	output_ports[1] = local_1943;
	output_ports[2] = local_1648;
	output_ports[3] = local_1646;
	output_ports[4] = local_1906;
	output_ports[5] = local_1905;
	output_ports[6] = local_1942;
	output_ports[7] = local_1904;
	output_ports[8] = local_1903;
	output_ports[9] = local_1902;
	output_ports[10] = local_1884;
	output_ports[11] = local_1883;
	output_ports[12] = local_1882;
	output_ports[13] = local_1864;
	output_ports[14] = local_1863;
	output_ports[15] = local_1862;
	output_ports[16] = local_1844;
	output_ports[17] = local_1843;
	output_ports[18] = local_1842;
	output_ports[19] = local_1824;
	output_ports[20] = local_1823;
	output_ports[21] = local_1822;
	output_ports[22] = local_1804;
	output_ports[23] = local_1803;
	output_ports[24] = local_1802;
	output_ports[25] = local_1784;
	output_ports[26] = local_1783;
	output_ports[27] = local_1782;
	output_ports[28] = local_1764;
	output_ports[29] = local_1763;
	output_ports[30] = local_1762;
	output_ports[31] = local_1744;
	output_ports[32] = local_1743;
	output_ports[33] = local_1742;
	output_ports[34] = local_1724;
	output_ports[35] = local_1723;
	output_ports[36] = local_1722;
	output_ports[37] = local_1704;
	output_ports[38] = local_1703;
	output_ports[39] = local_1702;
	output_ports[100] = local_1681;
	output_ports[101] = local_1680;
	memory[2039 - min_global] = local_1679;
	memory[2040 - min_global] = local_49;
	memory[2041 - min_global] = local_44;
	memory[2042 - min_global] = local_59;
	memory[2043 - min_global] = local_1678;
	memory[2044 - min_global] = local_1675;
	memory[2045 - min_global] = local_1637;
	memory[2046 - min_global] = local_1613;
	memory[2047 - min_global] = local_1672;
	memory[2048 - min_global] = local_1669;
	memory[2049 - min_global] = local_1661;
	memory[2050 - min_global] = local_1529;
	memory[2051 - min_global] = local_1519;
	memory[2052 - min_global] = local_38;
	memory[2053 - min_global] = local_1546;
	memory[2054 - min_global] = local_1541;
	memory[2055 - min_global] = local_1469;
	memory[2056 - min_global] = local_1457;
	memory[2057 - min_global] = local_1502;
	memory[2058 - min_global] = local_1499;
	memory[2059 - min_global] = local_1492;
	memory[2060 - min_global] = local_1388;
	memory[2061 - min_global] = local_1364;
	memory[2062 - min_global] = local_1421;
	memory[2063 - min_global] = local_1418;
	memory[2064 - min_global] = local_1411;
	memory[2065 - min_global] = local_1261;
	memory[2066 - min_global] = local_1237;
	memory[2067 - min_global] = local_1294;
	memory[2068 - min_global] = local_1291;
	memory[2069 - min_global] = local_1284;
	memory[2070 - min_global] = local_1134;
	memory[2071 - min_global] = local_1110;
	memory[2072 - min_global] = local_1167;
	memory[2073 - min_global] = local_1164;
	memory[2074 - min_global] = local_1157;
	memory[2075 - min_global] = local_1007;
	memory[2076 - min_global] = local_983;
	memory[2077 - min_global] = local_1040;
	memory[2078 - min_global] = local_1037;
	memory[2079 - min_global] = local_1030;
	memory[2080 - min_global] = local_880;
	memory[2081 - min_global] = local_856;
	memory[2082 - min_global] = local_913;
	memory[2083 - min_global] = local_910;
	memory[2084 - min_global] = local_903;
	memory[2085 - min_global] = local_753;
	memory[2086 - min_global] = local_729;
	memory[2087 - min_global] = local_786;
	memory[2088 - min_global] = local_783;
	memory[2089 - min_global] = local_776;
	memory[2090 - min_global] = local_626;
	memory[2091 - min_global] = local_602;
	memory[2092 - min_global] = local_659;
	memory[2093 - min_global] = local_656;
	memory[2094 - min_global] = local_649;
	memory[2095 - min_global] = local_499;
	memory[2096 - min_global] = local_475;
	memory[2097 - min_global] = local_532;
	memory[2098 - min_global] = local_529;
	memory[2099 - min_global] = local_522;
	memory[2100 - min_global] = local_372;
	memory[2101 - min_global] = local_348;
	memory[2102 - min_global] = local_405;
	memory[2103 - min_global] = local_402;
	memory[2104 - min_global] = local_395;
	memory[2105 - min_global] = local_236;
	memory[2106 - min_global] = local_212;
	memory[2107 - min_global] = local_278;
	memory[2108 - min_global] = local_266;
	memory[2109 - min_global] = local_259;
	memory[2110 - min_global] = local_100;
	memory[2111 - min_global] = local_82;
	memory[2112 - min_global] = local_133;
	memory[2113 - min_global] = local_130;
	memory[2114 - min_global] = local_123;
	memory[2115 - min_global] = local_1943;
	memory[2116 - min_global] = local_1921;
	memory[2117 - min_global] = local_1899;
	memory[2118 - min_global] = local_1879;
	memory[2119 - min_global] = local_1859;
	memory[2120 - min_global] = local_1839;
	memory[2121 - min_global] = local_1819;
	memory[2122 - min_global] = local_1799;
	memory[2123 - min_global] = local_1779;
	memory[2124 - min_global] = local_1759;
	memory[2125 - min_global] = local_1739;
	memory[2126 - min_global] = local_1719;
	memory[2127 - min_global] = local_1699;
	memory[2128 - min_global] = local_1942;
	status = lstatus;

	if (time_step == 0){
		int new_number_of_targets = 0;
		for (int i = 0; i < nb_of_targets; i++) {
			if (get_relative_distance(i) != 0. || is_target_validated(i))
				new_number_of_targets++;
		}
	 	nb_of_targets = new_number_of_targets;
	}
	if (!old_target_rel_pos_x.empty()) {
		rel_speed_targets_x.clear();
		rel_speed_targets_y.clear();
		for (int i = 0; i < nb_of_targets; i++) {
			rel_speed_targets_x.push_back(-(output_ports[pos_target_x_addrs[i]] - old_target_rel_pos_x[i]));
			rel_speed_targets_y.push_back(-(output_ports[pos_target_y_addrs[i]] - old_target_rel_pos_y[i]));
		}
		old_target_rel_pos_x.clear();
		old_target_rel_pos_y.clear();
	}
	for (int i = 0; i < nb_of_targets; i++) {
		old_target_rel_pos_x.push_back(output_ports[pos_target_x_addrs[i]]);
		old_target_rel_pos_y.push_back(output_ports[pos_target_y_addrs[i]]);
	}
	_fuel = output_ports[fuel_addr];
	time_step++;
}
