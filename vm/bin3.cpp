#include "bin.h"
#ifndef GENERATE 
void bin_3::step() {
bool lstatus = status;
memory[1] = memory[410];
memory[4] = memory[389];
memory[5] = memory[4] - memory[3];
lstatus = (memory[5] ==  0);
if (lstatus) memory[7]= memory[2]; else memory[7] = memory[1];
memory[8] = memory[7] - memory[0];
memory[9] = memory[408];
lstatus = (memory[5] ==  0);
if (lstatus) memory[11]= memory[0]; else memory[11] = memory[9];
memory[12] = memory[11] - memory[0];
lstatus = (memory[12] ==  0);
if (lstatus) memory[14]= memory[8]; else memory[14] = memory[7];
memory[16] = memory[409];
lstatus = (memory[5] ==  0);
if (lstatus) memory[19]= memory[17]; else memory[19] = memory[16];
memory[20] = memory[19] * memory[15];
lstatus = (memory[12] ==  0);
if (lstatus) memory[22]= memory[20]; else memory[22] = memory[19];
lstatus = (memory[12] ==  0);
if (lstatus) memory[24]= memory[19]; else memory[24] = memory[12];
memory[25] = memory[406];
memory[26] = memory[401];
memory[30] = input_ports[16000];
memory[31] = memory[30] - memory[29];
lstatus = (memory[31] ==  0);
if (lstatus) memory[33]= memory[28]; else memory[33] = memory[3];
memory[35] = memory[30] - memory[34];
lstatus = (memory[35] ==  0);
if (lstatus) memory[37]= memory[27]; else memory[37] = memory[33];
memory[40] = memory[30] - memory[39];
lstatus = (memory[40] ==  0);
if (lstatus) memory[42]= memory[38]; else memory[42] = memory[37];
memory[44] = memory[30] - memory[43];
lstatus = (memory[44] ==  0);
if (lstatus) memory[46]= memory[27]; else memory[46] = memory[42];
lstatus = (memory[5] ==  0);
if (lstatus) memory[48]= memory[46]; else memory[48] = memory[26];
memory[49] = memory[391];
lstatus = (memory[5] ==  0);
if (lstatus) memory[51]= memory[3]; else memory[51] = memory[49];
memory[52] = memory[51] - memory[48];
memory[53] = memory[52] * memory[52];
memory[54] = memory[400];
lstatus = (memory[31] ==  0);
if (lstatus) memory[56]= memory[27]; else memory[56] = memory[3];
lstatus = (memory[35] ==  0);
if (lstatus) memory[59]= memory[57]; else memory[59] = memory[56];
lstatus = (memory[40] ==  0);
if (lstatus) memory[62]= memory[60]; else memory[62] = memory[59];
lstatus = (memory[44] ==  0);
if (lstatus) memory[64]= memory[28]; else memory[64] = memory[62];
lstatus = (memory[5] ==  0);
if (lstatus) memory[66]= memory[64]; else memory[66] = memory[54];
memory[67] = memory[390];
lstatus = (memory[5] ==  0);
if (lstatus) memory[69]= memory[3]; else memory[69] = memory[67];
memory[70] = memory[69] - memory[66];
memory[71] = memory[70] * memory[70];
memory[72] = memory[71] + memory[53];
memory[73] = sqrt(memory[72]);
memory[74] = memory[73] * memory[73];
memory[75] = memory[74] * memory[73];
memory[77] = memory[392];
lstatus = (memory[5] ==  0);
if (lstatus) memory[80]= memory[78]; else memory[80] = memory[77];
memory[81] = memory[76] * memory[80];
memory[82] = (memory[75] == 0)? 0 : memory[81] / memory[75];
memory[83] = memory[52] * memory[82];
memory[84] = (memory[0] == 0)? 0 : memory[0] / memory[0];
memory[85] = memory[84] * memory[84];
memory[86] = (memory[15] == 0)? 0 : memory[85] / memory[15];
memory[87] = memory[83] * memory[86];
memory[88] = memory[404];
lstatus = (memory[35] ==  0);
if (lstatus) memory[91]= memory[89]; else memory[91] = memory[56];
lstatus = (memory[40] ==  0);
if (lstatus) memory[94]= memory[92]; else memory[94] = memory[91];
lstatus = (memory[44] ==  0);
if (lstatus) memory[97]= memory[95]; else memory[97] = memory[94];
lstatus = (memory[5] ==  0);
if (lstatus) memory[99]= memory[97]; else memory[99] = memory[88];
memory[100] = memory[99] * memory[84];
memory[101] = memory[48] + memory[100];
memory[102] = memory[101] + memory[87];
memory[103] = memory[396];
memory[106] = memory[105] + memory[17];
memory[107] = memory[30] - memory[106];
lstatus = (memory[107] ==  0);
if (lstatus) memory[109]= memory[104]; else memory[109] = memory[3];
memory[111] = memory[110] + memory[17];
memory[112] = memory[30] - memory[111];
lstatus = (memory[112] ==  0);
if (lstatus) memory[114]= memory[27]; else memory[114] = memory[109];
memory[116] = memory[115] + memory[17];
memory[117] = memory[30] - memory[116];
lstatus = (memory[117] ==  0);
if (lstatus) memory[119]= memory[38]; else memory[119] = memory[114];
memory[121] = memory[120] + memory[17];
memory[122] = memory[30] - memory[121];
lstatus = (memory[122] ==  0);
if (lstatus) memory[124]= memory[27]; else memory[124] = memory[119];
memory[125] = memory[30] - memory[105];
lstatus = (memory[125] ==  0);
if (lstatus) memory[127]= memory[104]; else memory[127] = memory[124];
memory[128] = memory[30] - memory[110];
lstatus = (memory[128] ==  0);
if (lstatus) memory[130]= memory[27]; else memory[130] = memory[127];
memory[131] = memory[30] - memory[115];
lstatus = (memory[131] ==  0);
if (lstatus) memory[133]= memory[38]; else memory[133] = memory[130];
memory[134] = memory[30] - memory[120];
lstatus = (memory[134] ==  0);
if (lstatus) memory[136]= memory[27]; else memory[136] = memory[133];
lstatus = (memory[5] ==  0);
if (lstatus) memory[138]= memory[136]; else memory[138] = memory[103];
memory[139] = memory[51] - memory[138];
memory[140] = memory[139] * memory[139];
memory[141] = memory[395];
lstatus = (memory[107] ==  0);
if (lstatus) memory[143]= memory[27]; else memory[143] = memory[3];
lstatus = (memory[112] ==  0);
if (lstatus) memory[145]= memory[28]; else memory[145] = memory[143];
lstatus = (memory[117] ==  0);
if (lstatus) memory[148]= memory[146]; else memory[148] = memory[145];
lstatus = (memory[122] ==  0);
if (lstatus) memory[151]= memory[149]; else memory[151] = memory[148];
lstatus = (memory[125] ==  0);
if (lstatus) memory[153]= memory[27]; else memory[153] = memory[151];
lstatus = (memory[128] ==  0);
if (lstatus) memory[155]= memory[28]; else memory[155] = memory[153];
lstatus = (memory[131] ==  0);
if (lstatus) memory[157]= memory[146]; else memory[157] = memory[155];
lstatus = (memory[134] ==  0);
if (lstatus) memory[159]= memory[149]; else memory[159] = memory[157];
lstatus = (memory[5] ==  0);
if (lstatus) memory[161]= memory[159]; else memory[161] = memory[141];
memory[162] = memory[69] - memory[161];
memory[163] = memory[162] * memory[162];
memory[164] = memory[163] + memory[140];
memory[165] = sqrt(memory[164]);
memory[166] = memory[165] * memory[165];
memory[167] = memory[166] * memory[165];
memory[168] = (memory[167] == 0)? 0 : memory[81] / memory[167];
memory[169] = memory[139] * memory[168];
memory[170] = input_ports[3];
memory[171] = (memory[84] == 0)? 0 : memory[170] / memory[84];
memory[172] = memory[171] + memory[169];
memory[173] = memory[172] * memory[86];
memory[174] = memory[399];
lstatus = (memory[112] ==  0);
if (lstatus) memory[177]= memory[175]; else memory[177] = memory[143];
lstatus = (memory[117] ==  0);
if (lstatus) memory[180]= memory[178]; else memory[180] = memory[177];
lstatus = (memory[122] ==  0);
if (lstatus) memory[183]= memory[181]; else memory[183] = memory[180];
lstatus = (memory[125] ==  0);
if (lstatus) memory[185]= memory[27]; else memory[185] = memory[183];
lstatus = (memory[128] ==  0);
if (lstatus) memory[187]= memory[175]; else memory[187] = memory[185];
lstatus = (memory[131] ==  0);
if (lstatus) memory[189]= memory[178]; else memory[189] = memory[187];
lstatus = (memory[134] ==  0);
if (lstatus) memory[191]= memory[181]; else memory[191] = memory[189];
lstatus = (memory[5] ==  0);
if (lstatus) memory[193]= memory[191]; else memory[193] = memory[174];
memory[194] = memory[193] * memory[84];
memory[195] = memory[138] + memory[194];
memory[196] = memory[195] + memory[173];
memory[197] = memory[196] - memory[102];
memory[198] = memory[197] * memory[197];
memory[199] = memory[70] * memory[82];
memory[200] = memory[199] * memory[86];
memory[201] = memory[403];
lstatus = (memory[31] ==  0);
if (lstatus) memory[204]= memory[202]; else memory[204] = memory[3];
lstatus = (memory[35] ==  0);
if (lstatus) memory[206]= memory[27]; else memory[206] = memory[204];
lstatus = (memory[40] ==  0);
if (lstatus) memory[209]= memory[207]; else memory[209] = memory[206];
lstatus = (memory[44] ==  0);
if (lstatus) memory[211]= memory[27]; else memory[211] = memory[209];
lstatus = (memory[5] ==  0);
if (lstatus) memory[213]= memory[211]; else memory[213] = memory[201];
memory[214] = memory[213] * memory[84];
memory[215] = memory[66] + memory[214];
memory[216] = memory[215] + memory[200];
memory[217] = memory[162] * memory[168];
memory[218] = input_ports[2];
memory[219] = (memory[84] == 0)? 0 : memory[218] / memory[84];
memory[220] = memory[219] + memory[217];
memory[221] = memory[220] * memory[86];
memory[222] = memory[398];
lstatus = (memory[107] ==  0);
if (lstatus) memory[225]= memory[223]; else memory[225] = memory[3];
lstatus = (memory[112] ==  0);
if (lstatus) memory[227]= memory[27]; else memory[227] = memory[225];
lstatus = (memory[117] ==  0);
if (lstatus) memory[229]= memory[178]; else memory[229] = memory[227];
lstatus = (memory[122] ==  0);
if (lstatus) memory[231]= memory[27]; else memory[231] = memory[229];
lstatus = (memory[125] ==  0);
if (lstatus) memory[233]= memory[223]; else memory[233] = memory[231];
lstatus = (memory[128] ==  0);
if (lstatus) memory[235]= memory[27]; else memory[235] = memory[233];
lstatus = (memory[131] ==  0);
if (lstatus) memory[237]= memory[178]; else memory[237] = memory[235];
lstatus = (memory[134] ==  0);
if (lstatus) memory[239]= memory[27]; else memory[239] = memory[237];
lstatus = (memory[5] ==  0);
if (lstatus) memory[241]= memory[239]; else memory[241] = memory[222];
memory[242] = memory[241] * memory[84];
memory[243] = memory[161] + memory[242];
memory[244] = memory[243] + memory[221];
memory[245] = memory[244] - memory[216];
memory[246] = memory[245] * memory[245];
memory[247] = memory[246] + memory[198];
memory[248] = sqrt(memory[247]);
memory[249] = memory[25] + memory[248];
memory[250] = memory[171] * memory[171];
memory[251] = memory[219] * memory[219];
memory[252] = memory[251] + memory[250];
memory[253] = sqrt(memory[252]);
memory[254] = memory[253] - memory[3];
lstatus = (memory[254] ==  0);
if (lstatus) memory[256]= memory[249]; else memory[256] = memory[3];
memory[257] = memory[248] - memory[17];
lstatus = (memory[257] <  0);
if (lstatus) memory[259]= memory[256]; else memory[259] = memory[3];
memory[260] = memory[51] - memory[102];
memory[261] = memory[260] * memory[260];
memory[262] = memory[69] - memory[216];
memory[263] = memory[262] * memory[262];
memory[264] = memory[263] + memory[261];
memory[265] = sqrt(memory[264]);
memory[266] = memory[265] * memory[265];
memory[267] = memory[266] * memory[265];
memory[268] = (memory[267] == 0)? 0 : memory[81] / memory[267];
memory[269] = memory[260] * memory[268];
memory[270] = memory[269] + memory[83];
memory[271] = (memory[15] == 0)? 0 : memory[270] / memory[15];
memory[272] = memory[271] * memory[84];
memory[273] = memory[99] + memory[272];
memory[274] = memory[262] * memory[268];
memory[275] = memory[274] + memory[199];
memory[276] = (memory[15] == 0)? 0 : memory[275] / memory[15];
memory[277] = memory[276] * memory[84];
memory[278] = memory[213] + memory[277];
memory[279] = memory[402];
lstatus = (memory[31] ==  0);
if (lstatus) memory[282]= memory[280]; else memory[282] = memory[3];
lstatus = (memory[35] ==  0);
if (lstatus) memory[284]= memory[280]; else memory[284] = memory[282];
lstatus = (memory[40] ==  0);
if (lstatus) memory[286]= memory[280]; else memory[286] = memory[284];
lstatus = (memory[44] ==  0);
if (lstatus) memory[288]= memory[280]; else memory[288] = memory[286];
lstatus = (memory[5] ==  0);
if (lstatus) memory[290]= memory[288]; else memory[290] = memory[279];
memory[291] = memory[51] - memory[196];
memory[292] = memory[291] * memory[291];
memory[293] = memory[69] - memory[244];
memory[294] = memory[293] * memory[293];
memory[295] = memory[294] + memory[292];
memory[296] = sqrt(memory[295]);
memory[297] = memory[296] * memory[296];
memory[298] = memory[297] * memory[296];
memory[299] = (memory[298] == 0)? 0 : memory[81] / memory[298];
memory[300] = memory[291] * memory[299];
memory[301] = memory[300] + memory[169];
memory[302] = (memory[15] == 0)? 0 : memory[301] / memory[15];
memory[303] = memory[171] + memory[302];
memory[304] = memory[303] * memory[84];
memory[305] = memory[193] + memory[304];
memory[306] = memory[293] * memory[299];
memory[307] = memory[306] + memory[217];
memory[308] = (memory[15] == 0)? 0 : memory[307] / memory[15];
memory[309] = memory[219] + memory[308];
memory[310] = memory[309] * memory[84];
memory[311] = memory[241] + memory[310];
memory[312] = memory[397];
lstatus = (memory[107] ==  0);
if (lstatus) memory[314]= memory[280]; else memory[314] = memory[3];
lstatus = (memory[112] ==  0);
if (lstatus) memory[316]= memory[280]; else memory[316] = memory[314];
lstatus = (memory[117] ==  0);
if (lstatus) memory[318]= memory[280]; else memory[318] = memory[316];
lstatus = (memory[122] ==  0);
if (lstatus) memory[320]= memory[280]; else memory[320] = memory[318];
lstatus = (memory[125] ==  0);
if (lstatus) memory[322]= memory[280]; else memory[322] = memory[320];
lstatus = (memory[128] ==  0);
if (lstatus) memory[324]= memory[280]; else memory[324] = memory[322];
lstatus = (memory[131] ==  0);
if (lstatus) memory[326]= memory[280]; else memory[326] = memory[324];
lstatus = (memory[134] ==  0);
if (lstatus) memory[328]= memory[280]; else memory[328] = memory[326];
lstatus = (memory[5] ==  0);
if (lstatus) memory[330]= memory[328]; else memory[330] = memory[312];
memory[331] = memory[394];
lstatus = (memory[5] ==  0);
if (lstatus) memory[333]= memory[3]; else memory[333] = memory[331];
memory[334] = memory[393];
lstatus = (memory[5] ==  0);
if (lstatus) memory[336]= memory[3]; else memory[336] = memory[334];
memory[337] = memory[4] + memory[0];
memory[338] = memory[102] - memory[196];
memory[339] = memory[216] - memory[244];
memory[340] = memory[405];
memory[341] = memory[340] + memory[0];
lstatus = (memory[254] ==  0);
if (lstatus) memory[343]= memory[341]; else memory[343] = memory[3];
lstatus = (memory[257] <  0);
if (lstatus) memory[345]= memory[343]; else memory[345] = memory[3];
memory[346] = memory[407];
lstatus = (memory[5] ==  0);
if (lstatus) memory[349]= memory[347]; else memory[349] = memory[346];
memory[350] = memory[253] * memory[84];
memory[351] = memory[349] - memory[350];
memory[355] = (memory[347] == 0)? 0 : memory[351] / memory[347];
memory[356] = memory[355] * memory[354];
memory[357] = memory[7] + memory[356];
memory[358] = memory[357] + memory[353];
memory[359] = memory[358] * memory[352];
memory[361] = (memory[84] == 0)? 0 : memory[360] / memory[84];
memory[362] = memory[361] - memory[345];
lstatus = (memory[362] <  0);
if (lstatus) memory[364]= memory[359]; else memory[364] = memory[3];
memory[365] = memory[351] - memory[3];
memory[366] = memory[382] - memory[0];
lstatus = (memory[365] <  0);
if (lstatus) memory[368]= memory[366]; else memory[368] = memory[364];
memory[369] = memory[347] - memory[350];
lstatus = (memory[369] <  0);
if (lstatus) memory[371]= memory[366]; else memory[371] = memory[368];
memory[373] = memory[196] - memory[51];
memory[374] = memory[373] * memory[373];
memory[375] = memory[244] - memory[69];
memory[376] = memory[375] * memory[375];
memory[377] = memory[376] + memory[374];
memory[378] = sqrt(memory[377]);
memory[379] = memory[378] - memory[372];
lstatus = (memory[379] <  0);
if (lstatus) memory[381]= memory[366]; else memory[381] = memory[371];
output_ports[0] = memory[381];
output_ports[1] = memory[351];
output_ports[2] = memory[293];
output_ports[3] = memory[291];
output_ports[4] = memory[339];
output_ports[5] = memory[338];
memory[389] = memory[337];
memory[390] = memory[69];
memory[391] = memory[51];
memory[392] = memory[80];
memory[393] = memory[336];
memory[394] = memory[333];
memory[395] = memory[244];
memory[396] = memory[196];
memory[397] = memory[330];
memory[398] = memory[311];
memory[399] = memory[305];
memory[400] = memory[216];
memory[401] = memory[102];
memory[402] = memory[290];
memory[403] = memory[278];
memory[404] = memory[273];
memory[405] = memory[345];
memory[406] = memory[259];
memory[407] = memory[351];
memory[408] = memory[24];
memory[409] = memory[22];
memory[410] = memory[14];
status = lstatus;
}
#endif
