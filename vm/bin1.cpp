#include "bin.h"
#ifndef GENERATE 
void bin_1::step() {
memory[1] = memory[265];
memory[4] = memory[248];
memory[5] = memory[4] - memory[3];
status = (memory[5] ==  0);
if (status) memory[7]= memory[2]; else memory[7] = memory[1];
memory[8] = memory[7] - memory[0];
memory[9] = memory[263];
status = (memory[5] ==  0);
if (status) memory[11]= memory[0]; else memory[11] = memory[9];
memory[12] = memory[11] - memory[0];
status = (memory[12] ==  0);
if (status) memory[14]= memory[8]; else memory[14] = memory[7];
memory[15] = memory[264];
status = (memory[5] ==  0);
if (status) memory[18]= memory[16]; else memory[18] = memory[15];
memory[20] = memory[18] * memory[19];
status = (memory[12] ==  0);
if (status) memory[22]= memory[20]; else memory[22] = memory[18];
status = (memory[12] ==  0);
if (status) memory[24]= memory[18]; else memory[24] = memory[12];
memory[25] = memory[260];
memory[26] = memory[262];
memory[29] = memory[28] * memory[16];
memory[30] = (memory[27] == 0)? 0 : memory[29] / memory[27];
memory[32] = input_ports[16000];
memory[33] = memory[32] - memory[31];
status = (memory[33] ==  0);
if (status) memory[35]= memory[30]; else memory[35] = memory[3];
memory[37] = (memory[36] == 0)? 0 : memory[29] / memory[36];
memory[39] = memory[32] - memory[38];
status = (memory[39] ==  0);
if (status) memory[41]= memory[37]; else memory[41] = memory[35];
memory[42] = (memory[19] == 0)? 0 : memory[29] / memory[19];
memory[44] = memory[32] - memory[43];
status = (memory[44] ==  0);
if (status) memory[46]= memory[42]; else memory[46] = memory[41];
memory[48] = memory[32] - memory[47];
status = (memory[48] ==  0);
if (status) memory[50]= memory[29]; else memory[50] = memory[46];
status = (memory[5] ==  0);
if (status) memory[52]= memory[50]; else memory[52] = memory[26];
memory[53] = memory[255];
status = (memory[33] ==  0);
if (status) memory[57]= memory[55]; else memory[57] = memory[3];
status = (memory[39] ==  0);
if (status) memory[59]= memory[54]; else memory[59] = memory[57];
status = (memory[44] ==  0);
if (status) memory[62]= memory[60]; else memory[62] = memory[59];
status = (memory[48] ==  0);
if (status) memory[64]= memory[54]; else memory[64] = memory[62];
status = (memory[5] ==  0);
if (status) memory[66]= memory[64]; else memory[66] = memory[53];
memory[67] = memory[250];
status = (memory[5] ==  0);
if (status) memory[69]= memory[3]; else memory[69] = memory[67];
memory[70] = memory[69] - memory[66];
memory[71] = memory[70] * memory[70];
memory[72] = memory[249];
status = (memory[5] ==  0);
if (status) memory[74]= memory[3]; else memory[74] = memory[72];
memory[75] = memory[254];
status = (memory[33] ==  0);
if (status) memory[77]= memory[54]; else memory[77] = memory[3];
status = (memory[39] ==  0);
if (status) memory[80]= memory[78]; else memory[80] = memory[77];
status = (memory[44] ==  0);
if (status) memory[83]= memory[81]; else memory[83] = memory[80];
status = (memory[48] ==  0);
if (status) memory[86]= memory[84]; else memory[86] = memory[83];
status = (memory[5] ==  0);
if (status) memory[88]= memory[86]; else memory[88] = memory[75];
memory[89] = memory[74] - memory[88];
memory[90] = memory[89] * memory[89];
memory[91] = memory[90] + memory[71];
memory[92] = sqrt(memory[91]);
memory[93] = memory[92] * memory[92];
memory[94] = memory[93] * memory[92];
memory[95] = memory[251];
status = (memory[5] ==  0);
if (status) memory[98]= memory[96]; else memory[98] = memory[95];
memory[100] = memory[99] * memory[98];
memory[101] = (memory[94] == 0)? 0 : memory[100] / memory[94];
memory[102] = memory[70] * memory[101];
memory[103] = input_ports[3];
memory[104] = (memory[0] == 0)? 0 : memory[0] / memory[0];
memory[105] = (memory[104] == 0)? 0 : memory[103] / memory[104];
memory[106] = memory[105] + memory[102];
memory[107] = memory[104] * memory[104];
memory[108] = (memory[19] == 0)? 0 : memory[107] / memory[19];
memory[109] = memory[106] * memory[108];
memory[110] = memory[258];
status = (memory[39] ==  0);
if (status) memory[113]= memory[111]; else memory[113] = memory[77];
status = (memory[44] ==  0);
if (status) memory[116]= memory[114]; else memory[116] = memory[113];
status = (memory[48] ==  0);
if (status) memory[119]= memory[117]; else memory[119] = memory[116];
status = (memory[5] ==  0);
if (status) memory[121]= memory[119]; else memory[121] = memory[110];
memory[122] = memory[121] * memory[104];
memory[123] = memory[66] + memory[122];
memory[124] = memory[123] + memory[109];
memory[125] = memory[124] - memory[69];
memory[126] = memory[125] * memory[125];
memory[127] = memory[89] * memory[101];
memory[128] = input_ports[2];
memory[129] = (memory[104] == 0)? 0 : memory[128] / memory[104];
memory[130] = memory[129] + memory[127];
memory[131] = memory[130] * memory[108];
memory[132] = memory[257];
status = (memory[33] ==  0);
if (status) memory[135]= memory[133]; else memory[135] = memory[3];
status = (memory[39] ==  0);
if (status) memory[137]= memory[54]; else memory[137] = memory[135];
status = (memory[44] ==  0);
if (status) memory[139]= memory[114]; else memory[139] = memory[137];
status = (memory[48] ==  0);
if (status) memory[141]= memory[54]; else memory[141] = memory[139];
status = (memory[5] ==  0);
if (status) memory[143]= memory[141]; else memory[143] = memory[132];
memory[144] = memory[143] * memory[104];
memory[145] = memory[88] + memory[144];
memory[146] = memory[145] + memory[131];
memory[147] = memory[146] - memory[74];
memory[148] = memory[147] * memory[147];
memory[149] = memory[148] + memory[126];
memory[150] = sqrt(memory[149]);
memory[151] = memory[150] - memory[52];
memory[152] = memory[3] - memory[151];
memory[153] = memory[151] - memory[3];
status = (memory[153] <  0);
if (status) memory[155]= memory[152]; else memory[155] = memory[151];
memory[156] = memory[25] + memory[155];
memory[157] = memory[105] * memory[105];
memory[158] = memory[129] * memory[129];
memory[159] = memory[158] + memory[157];
memory[160] = sqrt(memory[159]);
memory[161] = memory[160] - memory[3];
status = (memory[161] ==  0);
if (status) memory[163]= memory[156]; else memory[163] = memory[3];
memory[164] = memory[155] - memory[16];
status = (memory[164] <  0);
if (status) memory[166]= memory[163]; else memory[166] = memory[3];
memory[167] = memory[69] - memory[124];
memory[168] = memory[167] * memory[167];
memory[169] = memory[74] - memory[146];
memory[170] = memory[169] * memory[169];
memory[171] = memory[170] + memory[168];
memory[172] = sqrt(memory[171]);
memory[173] = memory[172] * memory[172];
memory[174] = memory[173] * memory[172];
memory[175] = (memory[174] == 0)? 0 : memory[100] / memory[174];
memory[176] = memory[167] * memory[175];
memory[177] = memory[176] + memory[102];
memory[178] = (memory[19] == 0)? 0 : memory[177] / memory[19];
memory[179] = memory[105] + memory[178];
memory[180] = memory[179] * memory[104];
memory[181] = memory[121] + memory[180];
memory[182] = memory[169] * memory[175];
memory[183] = memory[182] + memory[127];
memory[184] = (memory[19] == 0)? 0 : memory[183] / memory[19];
memory[185] = memory[129] + memory[184];
memory[186] = memory[185] * memory[104];
memory[187] = memory[143] + memory[186];
memory[188] = memory[256];
status = (memory[33] ==  0);
if (status) memory[191]= memory[189]; else memory[191] = memory[3];
status = (memory[39] ==  0);
if (status) memory[193]= memory[189]; else memory[193] = memory[191];
status = (memory[44] ==  0);
if (status) memory[195]= memory[189]; else memory[195] = memory[193];
status = (memory[48] ==  0);
if (status) memory[197]= memory[189]; else memory[197] = memory[195];
status = (memory[5] ==  0);
if (status) memory[199]= memory[197]; else memory[199] = memory[188];
memory[200] = memory[253];
status = (memory[5] ==  0);
if (status) memory[202]= memory[3]; else memory[202] = memory[200];
memory[203] = memory[252];
status = (memory[5] ==  0);
if (status) memory[205]= memory[3]; else memory[205] = memory[203];
memory[206] = memory[4] + memory[0];
memory[207] = memory[259];
memory[208] = memory[207] + memory[0];
status = (memory[161] ==  0);
if (status) memory[210]= memory[208]; else memory[210] = memory[3];
status = (memory[164] <  0);
if (status) memory[212]= memory[210]; else memory[212] = memory[3];
memory[213] = memory[261];
status = (memory[5] ==  0);
if (status) memory[216]= memory[214]; else memory[216] = memory[213];
memory[217] = memory[160] * memory[104];
memory[218] = memory[216] - memory[217];
memory[221] = memory[214] - memory[218];
memory[222] = (memory[214] == 0)? 0 : memory[221] / memory[214];
memory[223] = memory[222] * memory[220];
memory[224] = memory[7] + memory[223];
memory[225] = memory[224] + memory[219];
memory[227] = (memory[104] == 0)? 0 : memory[226] / memory[104];
memory[228] = memory[227] - memory[212];
status = (memory[228] <  0);
if (status) memory[230]= memory[225]; else memory[230] = memory[3];
memory[231] = memory[218] - memory[3];
memory[232] = memory[242] - memory[0];
status = (memory[231] <  0);
if (status) memory[234]= memory[232]; else memory[234] = memory[230];
memory[235] = memory[214] - memory[217];
status = (memory[235] <  0);
if (status) memory[237]= memory[232]; else memory[237] = memory[234];
memory[239] = memory[150] - memory[238];
status = (memory[239] <  0);
if (status) memory[241]= memory[232]; else memory[241] = memory[237];
output_ports[0] = memory[241];
output_ports[1] = memory[218];
output_ports[2] = memory[169];
output_ports[3] = memory[167];
output_ports[4] = memory[52];
memory[248] = memory[206];
memory[249] = memory[74];
memory[250] = memory[69];
memory[251] = memory[98];
memory[252] = memory[205];
memory[253] = memory[202];
memory[254] = memory[146];
memory[255] = memory[124];
memory[256] = memory[199];
memory[257] = memory[187];
memory[258] = memory[181];
memory[259] = memory[212];
memory[260] = memory[166];
memory[261] = memory[218];
memory[262] = memory[52];
memory[263] = memory[24];
memory[264] = memory[22];
memory[265] = memory[14];
}
#endif
