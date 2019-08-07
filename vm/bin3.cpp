#include "bin.h"
#ifndef GENERATE
bin_3::bin_3(int instance):vm_state() {
	_instance = instance;
	min_out_port = 0;
	max_out_port = 5;
	min_global = 389;
	max_global = 410;
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
	pos_target_x_addr = 0x4;
	pos_target_y_addr = 0x5;
	old_target_rel_pos_x = 0.;
	old_target_rel_pos_y = 0.;
	rel_speed_targets_x = 0.;
	rel_speed_targets_y = 0.;
	nb_of_targets = 1;
	reset();
	step();
	_fuel_max = get_fuel();
	step();
	reset();
	old_target_rel_pos_x = 0;
	old_target_rel_pos_y = 0;
}

bin_3::~bin_3() {
	free(memory);
	free(output_ports);
	free(input_ports);
}

double bin_3::get_relative_distance(int target) {
	return hypotl(output_ports[pos_target_x_addr], output_ports[pos_target_y_addr]);
}

Complex bin_3::get_target_absolute_position(int target) {
	Complex relative_target_pos(output_ports[pos_target_x_addr], output_ports[pos_target_y_addr]);
	Complex absolute_target_pos = get_absolute_position() + relative_target_pos;

	return absolute_target_pos;
}

Complex bin_3::get_absolute_position() {
	return -Complex(output_ports[pos_x_addr], output_ports[pos_y_addr]);
}

double bin_3::get_relative_delta_speed(int target) {
	return hypotl(rel_speed_targets_x, rel_speed_targets_y);
}

Complex bin_3::get_relative_speed(int target) {
	return Complex(rel_speed_targets_x, rel_speed_targets_y);
}

void bin_3::step() {
	bool lstatus = status;
	double local_1, local_4, local_5, local_7, local_8, local_9, local_11,
		local_12, local_14, local_16, local_19, local_20, local_22, local_24,
		local_25, local_26, local_30, local_31, local_33, local_35, local_37,
		local_40, local_42, local_44, local_46, local_48, local_49, local_51,
		local_52, local_53, local_54, local_56, local_59, local_62, local_64,
		local_66, local_67, local_69, local_70, local_71, local_72, local_73,
		local_74, local_75, local_77, local_80, local_81, local_82, local_83,
		local_84, local_85, local_86, local_87, local_88, local_91, local_94,
		local_97, local_99, local_100, local_101, local_102, local_103,
		local_106, local_107, local_109, local_111, local_112, local_114,
		local_116, local_117, local_119, local_121, local_122, local_124,
		local_125, local_127, local_128, local_130, local_131, local_133,
		local_134, local_136, local_138, local_139, local_140, local_141,
		local_143, local_145, local_148, local_151, local_153, local_155,
		local_157, local_159, local_161, local_162, local_163, local_164,
		local_165, local_166, local_167, local_168, local_169, local_170,
		local_171, local_172, local_173, local_174, local_177, local_180,
		local_183, local_185, local_187, local_189, local_191, local_193,
		local_194, local_195, local_196, local_197, local_198, local_199,
		local_200, local_201, local_204, local_206, local_209, local_211,
		local_213, local_214, local_215, local_216, local_217, local_218,
		local_219, local_220, local_221, local_222, local_225, local_227,
		local_229, local_231, local_233, local_235, local_237, local_239,
		local_241, local_242, local_243, local_244, local_245, local_246,
		local_247, local_248, local_249, local_250, local_251, local_252,
		local_253, local_254, local_256, local_257, local_259, local_260,
		local_261, local_262, local_263, local_264, local_265, local_266,
		local_267, local_268, local_269, local_270, local_271, local_272,
		local_273, local_274, local_275, local_276, local_277, local_278,
		local_279, local_282, local_284, local_286, local_288, local_290,
		local_291, local_292, local_293, local_294, local_295, local_296,
		local_297, local_298, local_299, local_300, local_301, local_302,
		local_303, local_304, local_305, local_306, local_307, local_308,
		local_309, local_310, local_311, local_312, local_314, local_316,
		local_318, local_320, local_322, local_324, local_326, local_328,
		local_330, local_331, local_333, local_334, local_336, local_337,
		local_338, local_339, local_340, local_341, local_343, local_345,
		local_346, local_349, local_350, local_351, local_355, local_356,
		local_357, local_358, local_359, local_361, local_362, local_364,
		local_365, local_366, local_368, local_369, local_371, local_373,
		local_374, local_375, local_376, local_377, local_378, local_379,
		local_381;
	const double const_0 = 1, const_2 = 30, const_3 = 0, const_15 = 2,
				 const_17 = 1000, const_27 = 0, const_28 = 8.357e+06,
				 const_29 = 3004, const_34 = 3003, const_38 = -6.357e+06,
				 const_39 = 3002, const_43 = 3001, const_57 = 7.357e+06,
				 const_60 = 6.357e+07, const_76 = 6.67428e-11, const_78 = 6e+24,
				 const_89 = -10328.9, const_92 = -2242.09, const_95 = -7614.57,
				 const_104 = 6.457e+06, const_105 = 2004, const_110 = 2003,
				 const_115 = 2002, const_120 = 2001, const_146 = 6.357e+06,
				 const_149 = 6.557e+06, const_175 = -6922.34,
				 const_178 = -4719.32, const_181 = -7814.93,
				 const_202 = 7614.57, const_207 = -224.209, const_223 = 7875.22,
				 const_280 = 1, const_347 = 50000, const_352 = 4,
				 const_353 = 25, const_354 = 45, const_360 = 900,
				 const_372 = 6.357e+06, const_382 = 0;
	local_1 = memory[410 - min_global];
	local_4 = memory[389 - min_global];
	local_5 = local_4 - const_3;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_7 = const_2;
	else
		local_7 = local_1;
	local_8 = local_7 - const_0;
	local_9 = memory[408 - min_global];
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
	local_16 = memory[409 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_19 = const_17;
	else
		local_19 = local_16;
	local_20 = local_19 * const_15;
	lstatus = (local_12 == 0);
	if (lstatus)
		local_22 = local_20;
	else
		local_22 = local_19;
	lstatus = (local_12 == 0);
	if (lstatus)
		local_24 = local_19;
	else
		local_24 = local_12;
	local_25 = memory[406 - min_global];
	local_26 = memory[401 - min_global];
	local_30 = input_ports[1];
	local_31 = local_30 - const_29;
	lstatus = (local_31 == 0);
	if (lstatus)
		local_33 = const_28;
	else
		local_33 = const_3;
	local_35 = local_30 - const_34;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_37 = const_27;
	else
		local_37 = local_33;
	local_40 = local_30 - const_39;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_42 = const_38;
	else
		local_42 = local_37;
	local_44 = local_30 - const_43;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_46 = const_27;
	else
		local_46 = local_42;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_48 = local_46;
	else
		local_48 = local_26;
	local_49 = memory[391 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_51 = const_3;
	else
		local_51 = local_49;
	local_52 = local_51 - local_48;
	local_53 = local_52 * local_52;
	local_54 = memory[400 - min_global];
	lstatus = (local_31 == 0);
	if (lstatus)
		local_56 = const_27;
	else
		local_56 = const_3;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_59 = const_57;
	else
		local_59 = local_56;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_62 = const_60;
	else
		local_62 = local_59;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_64 = const_28;
	else
		local_64 = local_62;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_66 = local_64;
	else
		local_66 = local_54;
	local_67 = memory[390 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_69 = const_3;
	else
		local_69 = local_67;
	local_70 = local_69 - local_66;
	local_71 = local_70 * local_70;
	local_72 = local_71 + local_53;
	local_73 = sqrt(local_72);
	local_74 = local_73 * local_73;
	local_75 = local_74 * local_73;
	local_77 = memory[392 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_80 = const_78;
	else
		local_80 = local_77;
	local_81 = const_76 * local_80;
	local_82 = (local_75 == 0) ? 0 : local_81 / local_75;
	local_83 = local_52 * local_82;
	local_84 = (const_0 == 0) ? 0 : const_0 / const_0;
	local_85 = local_84 * local_84;
	local_86 = (const_15 == 0) ? 0 : local_85 / const_15;
	local_87 = local_83 * local_86;
	local_88 = memory[404 - min_global];
	lstatus = (local_35 == 0);
	if (lstatus)
		local_91 = const_89;
	else
		local_91 = local_56;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_94 = const_92;
	else
		local_94 = local_91;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_97 = const_95;
	else
		local_97 = local_94;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_99 = local_97;
	else
		local_99 = local_88;
	local_100 = local_99 * local_84;
	local_101 = local_48 + local_100;
	local_102 = local_101 + local_87;
	local_103 = memory[396 - min_global];
	local_106 = const_105 + const_17;
	local_107 = local_30 - local_106;
	lstatus = (local_107 == 0);
	if (lstatus)
		local_109 = const_104;
	else
		local_109 = const_3;
	local_111 = const_110 + const_17;
	local_112 = local_30 - local_111;
	lstatus = (local_112 == 0);
	if (lstatus)
		local_114 = const_27;
	else
		local_114 = local_109;
	local_116 = const_115 + const_17;
	local_117 = local_30 - local_116;
	lstatus = (local_117 == 0);
	if (lstatus)
		local_119 = const_38;
	else
		local_119 = local_114;
	local_121 = const_120 + const_17;
	local_122 = local_30 - local_121;
	lstatus = (local_122 == 0);
	if (lstatus)
		local_124 = const_27;
	else
		local_124 = local_119;
	local_125 = local_30 - const_105;
	lstatus = (local_125 == 0);
	if (lstatus)
		local_127 = const_104;
	else
		local_127 = local_124;
	local_128 = local_30 - const_110;
	lstatus = (local_128 == 0);
	if (lstatus)
		local_130 = const_27;
	else
		local_130 = local_127;
	local_131 = local_30 - const_115;
	lstatus = (local_131 == 0);
	if (lstatus)
		local_133 = const_38;
	else
		local_133 = local_130;
	local_134 = local_30 - const_120;
	lstatus = (local_134 == 0);
	if (lstatus)
		local_136 = const_27;
	else
		local_136 = local_133;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_138 = local_136;
	else
		local_138 = local_103;
	local_139 = local_51 - local_138;
	local_140 = local_139 * local_139;
	local_141 = memory[395 - min_global];
	lstatus = (local_107 == 0);
	if (lstatus)
		local_143 = const_27;
	else
		local_143 = const_3;
	lstatus = (local_112 == 0);
	if (lstatus)
		local_145 = const_28;
	else
		local_145 = local_143;
	lstatus = (local_117 == 0);
	if (lstatus)
		local_148 = const_146;
	else
		local_148 = local_145;
	lstatus = (local_122 == 0);
	if (lstatus)
		local_151 = const_149;
	else
		local_151 = local_148;
	lstatus = (local_125 == 0);
	if (lstatus)
		local_153 = const_27;
	else
		local_153 = local_151;
	lstatus = (local_128 == 0);
	if (lstatus)
		local_155 = const_28;
	else
		local_155 = local_153;
	lstatus = (local_131 == 0);
	if (lstatus)
		local_157 = const_146;
	else
		local_157 = local_155;
	lstatus = (local_134 == 0);
	if (lstatus)
		local_159 = const_149;
	else
		local_159 = local_157;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_161 = local_159;
	else
		local_161 = local_141;
	local_162 = local_69 - local_161;
	local_163 = local_162 * local_162;
	local_164 = local_163 + local_140;
	local_165 = sqrt(local_164);
	local_166 = local_165 * local_165;
	local_167 = local_166 * local_165;
	local_168 = (local_167 == 0) ? 0 : local_81 / local_167;
	local_169 = local_139 * local_168;
	local_170 = input_ports[3];
	local_171 = (local_84 == 0) ? 0 : local_170 / local_84;
	local_172 = local_171 + local_169;
	local_173 = local_172 * local_86;
	local_174 = memory[399 - min_global];
	lstatus = (local_112 == 0);
	if (lstatus)
		local_177 = const_175;
	else
		local_177 = local_143;
	lstatus = (local_117 == 0);
	if (lstatus)
		local_180 = const_178;
	else
		local_180 = local_177;
	lstatus = (local_122 == 0);
	if (lstatus)
		local_183 = const_181;
	else
		local_183 = local_180;
	lstatus = (local_125 == 0);
	if (lstatus)
		local_185 = const_27;
	else
		local_185 = local_183;
	lstatus = (local_128 == 0);
	if (lstatus)
		local_187 = const_175;
	else
		local_187 = local_185;
	lstatus = (local_131 == 0);
	if (lstatus)
		local_189 = const_178;
	else
		local_189 = local_187;
	lstatus = (local_134 == 0);
	if (lstatus)
		local_191 = const_181;
	else
		local_191 = local_189;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_193 = local_191;
	else
		local_193 = local_174;
	local_194 = local_193 * local_84;
	local_195 = local_138 + local_194;
	local_196 = local_195 + local_173;
	local_197 = local_196 - local_102;
	local_198 = local_197 * local_197;
	local_199 = local_70 * local_82;
	local_200 = local_199 * local_86;
	local_201 = memory[403 - min_global];
	lstatus = (local_31 == 0);
	if (lstatus)
		local_204 = const_202;
	else
		local_204 = const_3;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_206 = const_27;
	else
		local_206 = local_204;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_209 = const_207;
	else
		local_209 = local_206;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_211 = const_27;
	else
		local_211 = local_209;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_213 = local_211;
	else
		local_213 = local_201;
	local_214 = local_213 * local_84;
	local_215 = local_66 + local_214;
	local_216 = local_215 + local_200;
	local_217 = local_162 * local_168;
	local_218 = input_ports[2];
	local_219 = (local_84 == 0) ? 0 : local_218 / local_84;
	local_220 = local_219 + local_217;
	local_221 = local_220 * local_86;
	local_222 = memory[398 - min_global];
	lstatus = (local_107 == 0);
	if (lstatus)
		local_225 = const_223;
	else
		local_225 = const_3;
	lstatus = (local_112 == 0);
	if (lstatus)
		local_227 = const_27;
	else
		local_227 = local_225;
	lstatus = (local_117 == 0);
	if (lstatus)
		local_229 = const_178;
	else
		local_229 = local_227;
	lstatus = (local_122 == 0);
	if (lstatus)
		local_231 = const_27;
	else
		local_231 = local_229;
	lstatus = (local_125 == 0);
	if (lstatus)
		local_233 = const_223;
	else
		local_233 = local_231;
	lstatus = (local_128 == 0);
	if (lstatus)
		local_235 = const_27;
	else
		local_235 = local_233;
	lstatus = (local_131 == 0);
	if (lstatus)
		local_237 = const_178;
	else
		local_237 = local_235;
	lstatus = (local_134 == 0);
	if (lstatus)
		local_239 = const_27;
	else
		local_239 = local_237;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_241 = local_239;
	else
		local_241 = local_222;
	local_242 = local_241 * local_84;
	local_243 = local_161 + local_242;
	local_244 = local_243 + local_221;
	local_245 = local_244 - local_216;
	local_246 = local_245 * local_245;
	local_247 = local_246 + local_198;
	local_248 = sqrt(local_247);
	local_249 = local_25 + local_248;
	local_250 = local_171 * local_171;
	local_251 = local_219 * local_219;
	local_252 = local_251 + local_250;
	local_253 = sqrt(local_252);
	local_254 = local_253 - const_3;
	lstatus = (local_254 == 0);
	if (lstatus)
		local_256 = local_249;
	else
		local_256 = const_3;
	local_257 = local_248 - const_17;
	lstatus = (local_257 < 0);
	if (lstatus)
		local_259 = local_256;
	else
		local_259 = const_3;
	local_260 = local_51 - local_102;
	local_261 = local_260 * local_260;
	local_262 = local_69 - local_216;
	local_263 = local_262 * local_262;
	local_264 = local_263 + local_261;
	local_265 = sqrt(local_264);
	local_266 = local_265 * local_265;
	local_267 = local_266 * local_265;
	local_268 = (local_267 == 0) ? 0 : local_81 / local_267;
	local_269 = local_260 * local_268;
	local_270 = local_269 + local_83;
	local_271 = (const_15 == 0) ? 0 : local_270 / const_15;
	local_272 = local_271 * local_84;
	local_273 = local_99 + local_272;
	local_274 = local_262 * local_268;
	local_275 = local_274 + local_199;
	local_276 = (const_15 == 0) ? 0 : local_275 / const_15;
	local_277 = local_276 * local_84;
	local_278 = local_213 + local_277;
	local_279 = memory[402 - min_global];
	lstatus = (local_31 == 0);
	if (lstatus)
		local_282 = const_280;
	else
		local_282 = const_3;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_284 = const_280;
	else
		local_284 = local_282;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_286 = const_280;
	else
		local_286 = local_284;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_288 = const_280;
	else
		local_288 = local_286;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_290 = local_288;
	else
		local_290 = local_279;
	local_291 = local_51 - local_196;
	local_292 = local_291 * local_291;
	local_293 = local_69 - local_244;
	local_294 = local_293 * local_293;
	local_295 = local_294 + local_292;
	local_296 = sqrt(local_295);
	local_297 = local_296 * local_296;
	local_298 = local_297 * local_296;
	local_299 = (local_298 == 0) ? 0 : local_81 / local_298;
	local_300 = local_291 * local_299;
	local_301 = local_300 + local_169;
	local_302 = (const_15 == 0) ? 0 : local_301 / const_15;
	local_303 = local_171 + local_302;
	local_304 = local_303 * local_84;
	local_305 = local_193 + local_304;
	local_306 = local_293 * local_299;
	local_307 = local_306 + local_217;
	local_308 = (const_15 == 0) ? 0 : local_307 / const_15;
	local_309 = local_219 + local_308;
	local_310 = local_309 * local_84;
	local_311 = local_241 + local_310;
	local_312 = memory[397 - min_global];
	lstatus = (local_107 == 0);
	if (lstatus)
		local_314 = const_280;
	else
		local_314 = const_3;
	lstatus = (local_112 == 0);
	if (lstatus)
		local_316 = const_280;
	else
		local_316 = local_314;
	lstatus = (local_117 == 0);
	if (lstatus)
		local_318 = const_280;
	else
		local_318 = local_316;
	lstatus = (local_122 == 0);
	if (lstatus)
		local_320 = const_280;
	else
		local_320 = local_318;
	lstatus = (local_125 == 0);
	if (lstatus)
		local_322 = const_280;
	else
		local_322 = local_320;
	lstatus = (local_128 == 0);
	if (lstatus)
		local_324 = const_280;
	else
		local_324 = local_322;
	lstatus = (local_131 == 0);
	if (lstatus)
		local_326 = const_280;
	else
		local_326 = local_324;
	lstatus = (local_134 == 0);
	if (lstatus)
		local_328 = const_280;
	else
		local_328 = local_326;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_330 = local_328;
	else
		local_330 = local_312;
	local_331 = memory[394 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_333 = const_3;
	else
		local_333 = local_331;
	local_334 = memory[393 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_336 = const_3;
	else
		local_336 = local_334;
	local_337 = local_4 + const_0;
	local_338 = local_102 - local_196;
	local_339 = local_216 - local_244;
	local_340 = memory[405 - min_global];
	local_341 = local_340 + const_0;
	lstatus = (local_254 == 0);
	if (lstatus)
		local_343 = local_341;
	else
		local_343 = const_3;
	lstatus = (local_257 < 0);
	if (lstatus)
		local_345 = local_343;
	else
		local_345 = const_3;
	local_346 = memory[407 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_349 = const_347;
	else
		local_349 = local_346;
	local_350 = local_253 * local_84;
	local_351 = local_349 - local_350;
	local_355 = (const_347 == 0) ? 0 : local_351 / const_347;
	local_356 = local_355 * const_354;
	local_357 = local_7 + local_356;
	local_358 = local_357 + const_353;
	local_359 = local_358 * const_352;
	local_361 = (local_84 == 0) ? 0 : const_360 / local_84;
	local_362 = local_361 - local_345;
	lstatus = (local_362 < 0);
	if (lstatus)
		local_364 = local_359;
	else
		local_364 = const_3;
	local_365 = local_351 - const_3;
	local_366 = const_382 - const_0;
	lstatus = (local_365 < 0);
	if (lstatus)
		local_368 = local_366;
	else
		local_368 = local_364;
	local_369 = const_347 - local_350;
	lstatus = (local_369 < 0);
	if (lstatus)
		local_371 = local_366;
	else
		local_371 = local_368;
	local_373 = local_196 - local_51;
	local_374 = local_373 * local_373;
	local_375 = local_244 - local_69;
	local_376 = local_375 * local_375;
	local_377 = local_376 + local_374;
	local_378 = sqrt(local_377);
	local_379 = local_378 - const_372;
	lstatus = (local_379 < 0);
	if (lstatus)
		local_381 = local_366;
	else
		local_381 = local_371;
	output_ports[0] = local_381;
	output_ports[1] = local_351;
	output_ports[2] = local_293;
	output_ports[3] = local_291;
	output_ports[4] = local_339;
	output_ports[5] = local_338;
	memory[389 - min_global] = local_337;
	memory[390 - min_global] = local_69;
	memory[391 - min_global] = local_51;
	memory[392 - min_global] = local_80;
	memory[393 - min_global] = local_336;
	memory[394 - min_global] = local_333;
	memory[395 - min_global] = local_244;
	memory[396 - min_global] = local_196;
	memory[397 - min_global] = local_330;
	memory[398 - min_global] = local_311;
	memory[399 - min_global] = local_305;
	memory[400 - min_global] = local_216;
	memory[401 - min_global] = local_102;
	memory[402 - min_global] = local_290;
	memory[403 - min_global] = local_278;
	memory[404 - min_global] = local_273;
	memory[405 - min_global] = local_345;
	memory[406 - min_global] = local_259;
	memory[407 - min_global] = local_351;
	memory[408 - min_global] = local_24;
	memory[409 - min_global] = local_22;
	memory[410 - min_global] = local_14;
	status = lstatus;

	if (old_target_rel_pos_x != 0) {
			rel_speed_targets_x = -(output_ports[pos_target_x_addr] - old_target_rel_pos_x);
			rel_speed_targets_y = -(output_ports[pos_target_y_addr] - old_target_rel_pos_y);
	}
	old_target_rel_pos_x = output_ports[pos_target_x_addr];
	old_target_rel_pos_y = output_ports[pos_target_y_addr];
	_fuel = output_ports[fuel_addr];
	time_step++;
}
#endif
