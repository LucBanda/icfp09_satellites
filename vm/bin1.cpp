#include "bin.h"
#ifndef GENERATE
bin_1::bin_1(int instance) {
	_instance = instance;
	min_out_port = 0;
	max_out_port = 4;
	min_global = 248;
	max_global = 265;
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
	radius_addr = 0x4;
	vm_state::reset();
}

bin_1::~bin_1() {
	free(memory);
	free(output_ports);
	free(input_ports);
}

double bin_1::get_radius() { return output_ports[radius_addr]; }

void bin_1::step() {
	bool lstatus = status;
	double local_1, local_4, local_5, local_7, local_8, local_9, local_11,
		local_12, local_14, local_15, local_18, local_20, local_22, local_24,
		local_25, local_26, local_29, local_30, local_32, local_33, local_35,
		local_37, local_39, local_41, local_42, local_44, local_46, local_48,
		local_50, local_52, local_53, local_57, local_59, local_62, local_64,
		local_66, local_67, local_69, local_70, local_71, local_72, local_74,
		local_75, local_77, local_80, local_83, local_86, local_88, local_89,
		local_90, local_91, local_92, local_93, local_94, local_95, local_98,
		local_100, local_101, local_102, local_103, local_104, local_105,
		local_106, local_107, local_108, local_109, local_110, local_113,
		local_116, local_119, local_121, local_122, local_123, local_124,
		local_125, local_126, local_127, local_128, local_129, local_130,
		local_131, local_132, local_135, local_137, local_139, local_141,
		local_143, local_144, local_145, local_146, local_147, local_148,
		local_149, local_150, local_151, local_152, local_153, local_155,
		local_156, local_157, local_158, local_159, local_160, local_161,
		local_163, local_164, local_166, local_167, local_168, local_169,
		local_170, local_171, local_172, local_173, local_174, local_175,
		local_176, local_177, local_178, local_179, local_180, local_181,
		local_182, local_183, local_184, local_185, local_186, local_187,
		local_188, local_191, local_193, local_195, local_197, local_199,
		local_200, local_202, local_203, local_205, local_206, local_207,
		local_208, local_210, local_212, local_213, local_216, local_217,
		local_218, local_221, local_222, local_223, local_224, local_225,
		local_227, local_228, local_230, local_231, local_232, local_234,
		local_235, local_237, local_239, local_241;
	const double const_0 = 1, const_2 = 30, const_3 = 0, const_16 = 1000,
				 const_19 = 2, const_27 = 1.1, const_28 = 42164,
				 const_31 = 1004, const_36 = 1.5, const_38 = 1003,
				 const_43 = 1002, const_47 = 1001, const_54 = 0,
				 const_55 = 6.457e+06, const_60 = -6.357e+06,
				 const_78 = 8.357e+06, const_81 = 6.357e+06,
				 const_84 = 6.557e+06, const_96 = 6e+24, const_99 = 6.67428e-11,
				 const_111 = -6922.34, const_114 = -4719.32,
				 const_117 = -7814.93, const_133 = -7875.22, const_189 = 1,
				 const_214 = 10000, const_219 = 25, const_220 = 45,
				 const_226 = 900, const_238 = 6.357e+06, const_242 = 0;
	local_1 = memory[265 - min_global];
	local_4 = memory[248 - min_global];
	local_5 = local_4 - const_3;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_7 = const_2;
	else
		local_7 = local_1;
	local_8 = local_7 - const_0;
	local_9 = memory[263 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_11 = const_0;
	else
		local_11 = local_9;
	local_12 = local_11 - const_0;
	lstatus = (local_12 == 0);
	if (lstatus)
		local_14 = local_8;
	else
		local_14 = local_7;
	local_15 = memory[264 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_18 = const_16;
	else
		local_18 = local_15;
	local_20 = local_18 * const_19;
	lstatus = (local_12 == 0);
	if (lstatus)
		local_22 = local_20;
	else
		local_22 = local_18;
	lstatus = (local_12 == 0);
	if (lstatus)
		local_24 = local_18;
	else
		local_24 = local_12;
	local_25 = memory[260 - min_global];
	local_26 = memory[262 - min_global];
	local_29 = const_28 * const_16;
	local_30 = (const_27 == 0) ? 0 : local_29 / const_27;
	local_32 = input_ports[1];
	local_33 = local_32 - const_31;
	lstatus = (local_33 == 0);
	if (lstatus)
		local_35 = local_30;
	else
		local_35 = const_3;
	local_37 = (const_36 == 0) ? 0 : local_29 / const_36;
	local_39 = local_32 - const_38;
	lstatus = (local_39 == 0);
	if (lstatus)
		local_41 = local_37;
	else
		local_41 = local_35;
	local_42 = (const_19 == 0) ? 0 : local_29 / const_19;
	local_44 = local_32 - const_43;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_46 = local_42;
	else
		local_46 = local_41;
	local_48 = local_32 - const_47;
	lstatus = (local_48 == 0);
	if (lstatus)
		local_50 = local_29;
	else
		local_50 = local_46;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_52 = local_50;
	else
		local_52 = local_26;
	local_53 = memory[255 - min_global];
	lstatus = (local_33 == 0);
	if (lstatus)
		local_57 = const_55;
	else
		local_57 = const_3;
	lstatus = (local_39 == 0);
	if (lstatus)
		local_59 = const_54;
	else
		local_59 = local_57;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_62 = const_60;
	else
		local_62 = local_59;
	lstatus = (local_48 == 0);
	if (lstatus)
		local_64 = const_54;
	else
		local_64 = local_62;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_66 = local_64;
	else
		local_66 = local_53;
	local_67 = memory[250 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_69 = const_3;
	else
		local_69 = local_67;
	local_70 = local_69 - local_66;
	local_71 = local_70 * local_70;
	local_72 = memory[249 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_74 = const_3;
	else
		local_74 = local_72;
	local_75 = memory[254 - min_global];
	lstatus = (local_33 == 0);
	if (lstatus)
		local_77 = const_54;
	else
		local_77 = const_3;
	lstatus = (local_39 == 0);
	if (lstatus)
		local_80 = const_78;
	else
		local_80 = local_77;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_83 = const_81;
	else
		local_83 = local_80;
	lstatus = (local_48 == 0);
	if (lstatus)
		local_86 = const_84;
	else
		local_86 = local_83;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_88 = local_86;
	else
		local_88 = local_75;
	local_89 = local_74 - local_88;
	local_90 = local_89 * local_89;
	local_91 = local_90 + local_71;
	local_92 = sqrt(local_91);
	local_93 = local_92 * local_92;
	local_94 = local_93 * local_92;
	local_95 = memory[251 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_98 = const_96;
	else
		local_98 = local_95;
	local_100 = const_99 * local_98;
	local_101 = (local_94 == 0) ? 0 : local_100 / local_94;
	local_102 = local_70 * local_101;
	local_103 = input_ports[3];
	local_104 = (const_0 == 0) ? 0 : const_0 / const_0;
	local_105 = (local_104 == 0) ? 0 : local_103 / local_104;
	local_106 = local_105 + local_102;
	local_107 = local_104 * local_104;
	local_108 = (const_19 == 0) ? 0 : local_107 / const_19;
	local_109 = local_106 * local_108;
	local_110 = memory[258 - min_global];
	lstatus = (local_39 == 0);
	if (lstatus)
		local_113 = const_111;
	else
		local_113 = local_77;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_116 = const_114;
	else
		local_116 = local_113;
	lstatus = (local_48 == 0);
	if (lstatus)
		local_119 = const_117;
	else
		local_119 = local_116;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_121 = local_119;
	else
		local_121 = local_110;
	local_122 = local_121 * local_104;
	local_123 = local_66 + local_122;
	local_124 = local_123 + local_109;
	local_125 = local_124 - local_69;
	local_126 = local_125 * local_125;
	local_127 = local_89 * local_101;
	local_128 = input_ports[2];
	local_129 = (local_104 == 0) ? 0 : local_128 / local_104;
	local_130 = local_129 + local_127;
	local_131 = local_130 * local_108;
	local_132 = memory[257 - min_global];
	lstatus = (local_33 == 0);
	if (lstatus)
		local_135 = const_133;
	else
		local_135 = const_3;
	lstatus = (local_39 == 0);
	if (lstatus)
		local_137 = const_54;
	else
		local_137 = local_135;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_139 = const_114;
	else
		local_139 = local_137;
	lstatus = (local_48 == 0);
	if (lstatus)
		local_141 = const_54;
	else
		local_141 = local_139;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_143 = local_141;
	else
		local_143 = local_132;
	local_144 = local_143 * local_104;
	local_145 = local_88 + local_144;
	local_146 = local_145 + local_131;
	local_147 = local_146 - local_74;
	local_148 = local_147 * local_147;
	local_149 = local_148 + local_126;
	local_150 = sqrt(local_149);
	local_151 = local_150 - local_52;
	local_152 = const_3 - local_151;
	local_153 = local_151 - const_3;
	lstatus = (local_153 < 0);
	if (lstatus)
		local_155 = local_152;
	else
		local_155 = local_151;
	local_156 = local_25 + local_155;
	local_157 = local_105 * local_105;
	local_158 = local_129 * local_129;
	local_159 = local_158 + local_157;
	local_160 = sqrt(local_159);
	local_161 = local_160 - const_3;
	lstatus = (local_161 == 0);
	if (lstatus)
		local_163 = local_156;
	else
		local_163 = const_3;
	local_164 = local_155 - const_16;
	lstatus = (local_164 < 0);
	if (lstatus)
		local_166 = local_163;
	else
		local_166 = const_3;
	local_167 = local_69 - local_124;
	local_168 = local_167 * local_167;
	local_169 = local_74 - local_146;
	local_170 = local_169 * local_169;
	local_171 = local_170 + local_168;
	local_172 = sqrt(local_171);
	local_173 = local_172 * local_172;
	local_174 = local_173 * local_172;
	local_175 = (local_174 == 0) ? 0 : local_100 / local_174;
	local_176 = local_167 * local_175;
	local_177 = local_176 + local_102;
	local_178 = (const_19 == 0) ? 0 : local_177 / const_19;
	local_179 = local_105 + local_178;
	local_180 = local_179 * local_104;
	local_181 = local_121 + local_180;
	local_182 = local_169 * local_175;
	local_183 = local_182 + local_127;
	local_184 = (const_19 == 0) ? 0 : local_183 / const_19;
	local_185 = local_129 + local_184;
	local_186 = local_185 * local_104;
	local_187 = local_143 + local_186;
	local_188 = memory[256 - min_global];
	lstatus = (local_33 == 0);
	if (lstatus)
		local_191 = const_189;
	else
		local_191 = const_3;
	lstatus = (local_39 == 0);
	if (lstatus)
		local_193 = const_189;
	else
		local_193 = local_191;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_195 = const_189;
	else
		local_195 = local_193;
	lstatus = (local_48 == 0);
	if (lstatus)
		local_197 = const_189;
	else
		local_197 = local_195;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_199 = local_197;
	else
		local_199 = local_188;
	local_200 = memory[253 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_202 = const_3;
	else
		local_202 = local_200;
	local_203 = memory[252 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_205 = const_3;
	else
		local_205 = local_203;
	local_206 = local_4 + const_0;
	local_207 = memory[259 - min_global];
	local_208 = local_207 + const_0;
	lstatus = (local_161 == 0);
	if (lstatus)
		local_210 = local_208;
	else
		local_210 = const_3;
	lstatus = (local_164 < 0);
	if (lstatus)
		local_212 = local_210;
	else
		local_212 = const_3;
	local_213 = memory[261 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_216 = const_214;
	else
		local_216 = local_213;
	local_217 = local_160 * local_104;
	local_218 = local_216 - local_217;
	local_221 = const_214 - local_218;
	local_222 = (const_214 == 0) ? 0 : local_221 / const_214;
	local_223 = local_222 * const_220;
	local_224 = local_7 + local_223;
	local_225 = local_224 + const_219;
	local_227 = (local_104 == 0) ? 0 : const_226 / local_104;
	local_228 = local_227 - local_212;
	lstatus = (local_228 < 0);
	if (lstatus)
		local_230 = local_225;
	else
		local_230 = const_3;
	local_231 = local_218 - const_3;
	local_232 = const_242 - const_0;
	lstatus = (local_231 < 0);
	if (lstatus)
		local_234 = local_232;
	else
		local_234 = local_230;
	local_235 = const_214 - local_217;
	lstatus = (local_235 < 0);
	if (lstatus)
		local_237 = local_232;
	else
		local_237 = local_234;
	local_239 = local_150 - const_238;
	lstatus = (local_239 < 0);
	if (lstatus)
		local_241 = local_232;
	else
		local_241 = local_237;
	output_ports[0] = local_241;
	output_ports[1] = local_218;
	output_ports[2] = local_169;
	output_ports[3] = local_167;
	output_ports[4] = local_52;
	memory[248 - min_global] = local_206;
	memory[249 - min_global] = local_74;
	memory[250 - min_global] = local_69;
	memory[251 - min_global] = local_98;
	memory[252 - min_global] = local_205;
	memory[253 - min_global] = local_202;
	memory[254 - min_global] = local_146;
	memory[255 - min_global] = local_124;
	memory[256 - min_global] = local_199;
	memory[257 - min_global] = local_187;
	memory[258 - min_global] = local_181;
	memory[259 - min_global] = local_212;
	memory[260 - min_global] = local_166;
	memory[261 - min_global] = local_218;
	memory[262 - min_global] = local_52;
	memory[263 - min_global] = local_24;
	memory[264 - min_global] = local_22;
	memory[265 - min_global] = local_14;
	status = lstatus;

	vm_state::step();
}
#endif
