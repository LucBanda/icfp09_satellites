#include "bin.h"
#ifndef GENERATE
bin_2::bin_2(int instance) {
	_instance = instance;
	min_out_port = 0;
	max_out_port = 5;
	min_global = 379;
	max_global = 400;
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
	reset();
}

bin_2::~bin_2() {
	free(memory);
	free(output_ports);
	free(input_ports);
}

/*double bin_2::get_radius()
{
	double x = output_ports[pos_x_addr];
	double y = output_ports[pos_y_addr];
	double target_x = - output_ports[pos_target_x_addr] + x;
	double target_y = - output_ports[pos_target_y_addr] + y;
	return sqrt(target_x*target_x + target_y*target_y);
}*/

vector<satellite> bin_2::get_targets() {
	double x = get_pos_x();
	double y = get_pos_y();
	double target_x = output_ports[pos_target_x_addr] + x;
	double target_y = output_ports[pos_target_y_addr] + y;

	vector<satellite> target;
	satellite sat(target_x, target_y);
	target.push_back(sat);
	return target;
}

void bin_2::step() {
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
		local_105, local_106, local_108, local_109, local_110, local_112,
		local_113, local_114, local_116, local_117, local_118, local_120,
		local_122, local_124, local_126, local_128, local_130, local_131,
		local_132, local_133, local_135, local_137, local_140, local_143,
		local_145, local_147, local_149, local_151, local_153, local_154,
		local_155, local_156, local_157, local_158, local_159, local_160,
		local_161, local_162, local_163, local_164, local_165, local_166,
		local_168, local_171, local_174, local_176, local_178, local_180,
		local_182, local_184, local_185, local_186, local_187, local_188,
		local_189, local_190, local_191, local_192, local_195, local_197,
		local_200, local_202, local_204, local_205, local_206, local_207,
		local_208, local_209, local_210, local_211, local_212, local_213,
		local_216, local_218, local_220, local_222, local_224, local_226,
		local_228, local_230, local_232, local_233, local_234, local_235,
		local_236, local_237, local_238, local_239, local_240, local_241,
		local_242, local_243, local_244, local_245, local_247, local_248,
		local_250, local_251, local_252, local_253, local_254, local_255,
		local_256, local_257, local_258, local_259, local_260, local_261,
		local_262, local_263, local_264, local_265, local_266, local_267,
		local_268, local_269, local_270, local_273, local_275, local_277,
		local_279, local_281, local_282, local_283, local_284, local_285,
		local_286, local_287, local_288, local_289, local_290, local_291,
		local_292, local_293, local_294, local_295, local_296, local_297,
		local_298, local_299, local_300, local_301, local_302, local_303,
		local_305, local_307, local_309, local_311, local_313, local_315,
		local_317, local_319, local_321, local_322, local_324, local_325,
		local_327, local_328, local_329, local_330, local_331, local_332,
		local_334, local_336, local_337, local_340, local_341, local_342,
		local_345, local_346, local_347, local_348, local_349, local_351,
		local_352, local_354, local_355, local_356, local_358, local_359,
		local_361, local_363, local_364, local_365, local_366, local_367,
		local_368, local_369, local_371;
	const double const_0 = 1, const_2 = 30, const_3 = 0, const_15 = 2,
				 const_17 = 1000, const_27 = 0, const_28 = 8.357e+06,
				 const_29 = 2004, const_34 = 2003, const_38 = -6.357e+06,
				 const_39 = 2002, const_43 = 2001, const_57 = 7.357e+06,
				 const_60 = 6.357e+07, const_76 = 6.67428e-11, const_78 = 6e+24,
				 const_89 = -7377.81, const_92 = -2491.21, const_95 = -6922.34,
				 const_104 = 6.457e+06, const_138 = 6.357e+06,
				 const_141 = 6.557e+06, const_169 = -4719.32,
				 const_172 = -7814.93, const_193 = 6922.34,
				 const_198 = -249.121, const_214 = 7875.22, const_271 = 1,
				 const_338 = 50000, const_343 = 25, const_344 = 45,
				 const_350 = 900, const_362 = 6.357e+06, const_372 = 0;
	local_1 = memory[400 - min_global];
	local_4 = memory[379 - min_global];
	local_5 = local_4 - const_3;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_7 = const_2;
	else
		local_7 = local_1;
	local_8 = local_7 - const_0;
	local_9 = memory[398 - min_global];
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
	local_16 = memory[399 - min_global];
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
	local_25 = memory[396 - min_global];
	local_26 = memory[391 - min_global];
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
	local_49 = memory[381 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_51 = const_3;
	else
		local_51 = local_49;
	local_52 = local_51 - local_48;
	local_53 = local_52 * local_52;
	local_54 = memory[390 - min_global];
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
	local_67 = memory[380 - min_global];
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
	local_77 = memory[382 - min_global];
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
	local_88 = memory[394 - min_global];
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
	local_103 = memory[386 - min_global];
	local_105 = const_29 + const_17;
	local_106 = local_30 - local_105;
	lstatus = (local_106 == 0);
	if (lstatus)
		local_108 = const_104;
	else
		local_108 = const_3;
	local_109 = const_34 + const_17;
	local_110 = local_30 - local_109;
	lstatus = (local_110 == 0);
	if (lstatus)
		local_112 = const_27;
	else
		local_112 = local_108;
	local_113 = const_39 + const_17;
	local_114 = local_30 - local_113;
	lstatus = (local_114 == 0);
	if (lstatus)
		local_116 = const_38;
	else
		local_116 = local_112;
	local_117 = const_43 + const_17;
	local_118 = local_30 - local_117;
	lstatus = (local_118 == 0);
	if (lstatus)
		local_120 = const_27;
	else
		local_120 = local_116;
	lstatus = (local_31 == 0);
	if (lstatus)
		local_122 = const_104;
	else
		local_122 = local_120;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_124 = const_27;
	else
		local_124 = local_122;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_126 = const_38;
	else
		local_126 = local_124;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_128 = const_27;
	else
		local_128 = local_126;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_130 = local_128;
	else
		local_130 = local_103;
	local_131 = local_51 - local_130;
	local_132 = local_131 * local_131;
	local_133 = memory[385 - min_global];
	lstatus = (local_106 == 0);
	if (lstatus)
		local_135 = const_27;
	else
		local_135 = const_3;
	lstatus = (local_110 == 0);
	if (lstatus)
		local_137 = const_28;
	else
		local_137 = local_135;
	lstatus = (local_114 == 0);
	if (lstatus)
		local_140 = const_138;
	else
		local_140 = local_137;
	lstatus = (local_118 == 0);
	if (lstatus)
		local_143 = const_141;
	else
		local_143 = local_140;
	lstatus = (local_31 == 0);
	if (lstatus)
		local_145 = const_27;
	else
		local_145 = local_143;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_147 = const_28;
	else
		local_147 = local_145;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_149 = const_138;
	else
		local_149 = local_147;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_151 = const_141;
	else
		local_151 = local_149;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_153 = local_151;
	else
		local_153 = local_133;
	local_154 = local_69 - local_153;
	local_155 = local_154 * local_154;
	local_156 = local_155 + local_132;
	local_157 = sqrt(local_156);
	local_158 = local_157 * local_157;
	local_159 = local_158 * local_157;
	local_160 = (local_159 == 0) ? 0 : local_81 / local_159;
	local_161 = local_131 * local_160;
	local_162 = input_ports[3];
	local_163 = (local_84 == 0) ? 0 : local_162 / local_84;
	local_164 = local_163 + local_161;
	local_165 = local_164 * local_86;
	local_166 = memory[389 - min_global];
	lstatus = (local_110 == 0);
	if (lstatus)
		local_168 = const_95;
	else
		local_168 = local_135;
	lstatus = (local_114 == 0);
	if (lstatus)
		local_171 = const_169;
	else
		local_171 = local_168;
	lstatus = (local_118 == 0);
	if (lstatus)
		local_174 = const_172;
	else
		local_174 = local_171;
	lstatus = (local_31 == 0);
	if (lstatus)
		local_176 = const_27;
	else
		local_176 = local_174;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_178 = const_95;
	else
		local_178 = local_176;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_180 = const_169;
	else
		local_180 = local_178;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_182 = const_172;
	else
		local_182 = local_180;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_184 = local_182;
	else
		local_184 = local_166;
	local_185 = local_184 * local_84;
	local_186 = local_130 + local_185;
	local_187 = local_186 + local_165;
	local_188 = local_187 - local_102;
	local_189 = local_188 * local_188;
	local_190 = local_70 * local_82;
	local_191 = local_190 * local_86;
	local_192 = memory[393 - min_global];
	lstatus = (local_31 == 0);
	if (lstatus)
		local_195 = const_193;
	else
		local_195 = const_3;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_197 = const_27;
	else
		local_197 = local_195;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_200 = const_198;
	else
		local_200 = local_197;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_202 = const_27;
	else
		local_202 = local_200;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_204 = local_202;
	else
		local_204 = local_192;
	local_205 = local_204 * local_84;
	local_206 = local_66 + local_205;
	local_207 = local_206 + local_191;
	local_208 = local_154 * local_160;
	local_209 = input_ports[2];
	local_210 = (local_84 == 0) ? 0 : local_209 / local_84;
	local_211 = local_210 + local_208;
	local_212 = local_211 * local_86;
	local_213 = memory[388 - min_global];
	lstatus = (local_106 == 0);
	if (lstatus)
		local_216 = const_214;
	else
		local_216 = const_3;
	lstatus = (local_110 == 0);
	if (lstatus)
		local_218 = const_27;
	else
		local_218 = local_216;
	lstatus = (local_114 == 0);
	if (lstatus)
		local_220 = const_169;
	else
		local_220 = local_218;
	lstatus = (local_118 == 0);
	if (lstatus)
		local_222 = const_27;
	else
		local_222 = local_220;
	lstatus = (local_31 == 0);
	if (lstatus)
		local_224 = const_214;
	else
		local_224 = local_222;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_226 = const_27;
	else
		local_226 = local_224;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_228 = const_169;
	else
		local_228 = local_226;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_230 = const_27;
	else
		local_230 = local_228;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_232 = local_230;
	else
		local_232 = local_213;
	local_233 = local_232 * local_84;
	local_234 = local_153 + local_233;
	local_235 = local_234 + local_212;
	local_236 = local_235 - local_207;
	local_237 = local_236 * local_236;
	local_238 = local_237 + local_189;
	local_239 = sqrt(local_238);
	local_240 = local_25 + local_239;
	local_241 = local_163 * local_163;
	local_242 = local_210 * local_210;
	local_243 = local_242 + local_241;
	local_244 = sqrt(local_243);
	local_245 = local_244 - const_3;
	lstatus = (local_245 == 0);
	if (lstatus)
		local_247 = local_240;
	else
		local_247 = const_3;
	local_248 = local_239 - const_17;
	lstatus = (local_248 < 0);
	if (lstatus)
		local_250 = local_247;
	else
		local_250 = const_3;
	local_251 = local_51 - local_102;
	local_252 = local_251 * local_251;
	local_253 = local_69 - local_207;
	local_254 = local_253 * local_253;
	local_255 = local_254 + local_252;
	local_256 = sqrt(local_255);
	local_257 = local_256 * local_256;
	local_258 = local_257 * local_256;
	local_259 = (local_258 == 0) ? 0 : local_81 / local_258;
	local_260 = local_251 * local_259;
	local_261 = local_260 + local_83;
	local_262 = (const_15 == 0) ? 0 : local_261 / const_15;
	local_263 = local_262 * local_84;
	local_264 = local_99 + local_263;
	local_265 = local_253 * local_259;
	local_266 = local_265 + local_190;
	local_267 = (const_15 == 0) ? 0 : local_266 / const_15;
	local_268 = local_267 * local_84;
	local_269 = local_204 + local_268;
	local_270 = memory[392 - min_global];
	lstatus = (local_31 == 0);
	if (lstatus)
		local_273 = const_271;
	else
		local_273 = const_3;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_275 = const_271;
	else
		local_275 = local_273;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_277 = const_271;
	else
		local_277 = local_275;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_279 = const_271;
	else
		local_279 = local_277;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_281 = local_279;
	else
		local_281 = local_270;
	local_282 = local_51 - local_187;
	local_283 = local_282 * local_282;
	local_284 = local_69 - local_235;
	local_285 = local_284 * local_284;
	local_286 = local_285 + local_283;
	local_287 = sqrt(local_286);
	local_288 = local_287 * local_287;
	local_289 = local_288 * local_287;
	local_290 = (local_289 == 0) ? 0 : local_81 / local_289;
	local_291 = local_282 * local_290;
	local_292 = local_291 + local_161;
	local_293 = (const_15 == 0) ? 0 : local_292 / const_15;
	local_294 = local_163 + local_293;
	local_295 = local_294 * local_84;
	local_296 = local_184 + local_295;
	local_297 = local_284 * local_290;
	local_298 = local_297 + local_208;
	local_299 = (const_15 == 0) ? 0 : local_298 / const_15;
	local_300 = local_210 + local_299;
	local_301 = local_300 * local_84;
	local_302 = local_232 + local_301;
	local_303 = memory[387 - min_global];
	lstatus = (local_106 == 0);
	if (lstatus)
		local_305 = const_271;
	else
		local_305 = const_3;
	lstatus = (local_110 == 0);
	if (lstatus)
		local_307 = const_271;
	else
		local_307 = local_305;
	lstatus = (local_114 == 0);
	if (lstatus)
		local_309 = const_271;
	else
		local_309 = local_307;
	lstatus = (local_118 == 0);
	if (lstatus)
		local_311 = const_271;
	else
		local_311 = local_309;
	lstatus = (local_31 == 0);
	if (lstatus)
		local_313 = const_271;
	else
		local_313 = local_311;
	lstatus = (local_35 == 0);
	if (lstatus)
		local_315 = const_271;
	else
		local_315 = local_313;
	lstatus = (local_40 == 0);
	if (lstatus)
		local_317 = const_271;
	else
		local_317 = local_315;
	lstatus = (local_44 == 0);
	if (lstatus)
		local_319 = const_271;
	else
		local_319 = local_317;
	lstatus = (local_5 == 0);
	if (lstatus)
		local_321 = local_319;
	else
		local_321 = local_303;
	local_322 = memory[384 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_324 = const_3;
	else
		local_324 = local_322;
	local_325 = memory[383 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_327 = const_3;
	else
		local_327 = local_325;
	local_328 = local_4 + const_0;
	local_329 = local_102 - local_187;
	local_330 = local_207 - local_235;
	local_331 = memory[395 - min_global];
	local_332 = local_331 + const_0;
	lstatus = (local_245 == 0);
	if (lstatus)
		local_334 = local_332;
	else
		local_334 = const_3;
	lstatus = (local_248 < 0);
	if (lstatus)
		local_336 = local_334;
	else
		local_336 = const_3;
	local_337 = memory[397 - min_global];
	lstatus = (local_5 == 0);
	if (lstatus)
		local_340 = const_338;
	else
		local_340 = local_337;
	local_341 = local_244 * local_84;
	local_342 = local_340 - local_341;
	local_345 = (const_338 == 0) ? 0 : local_342 / const_338;
	local_346 = local_345 * const_344;
	local_347 = local_7 + local_346;
	local_348 = local_347 + const_343;
	local_349 = local_348 * const_15;
	local_351 = (local_84 == 0) ? 0 : const_350 / local_84;
	local_352 = local_351 - local_336;
	lstatus = (local_352 < 0);
	if (lstatus)
		local_354 = local_349;
	else
		local_354 = const_3;
	local_355 = local_342 - const_3;
	local_356 = const_372 - const_0;
	lstatus = (local_355 < 0);
	if (lstatus)
		local_358 = local_356;
	else
		local_358 = local_354;
	local_359 = const_338 - local_341;
	lstatus = (local_359 < 0);
	if (lstatus)
		local_361 = local_356;
	else
		local_361 = local_358;
	local_363 = local_187 - local_51;
	local_364 = local_363 * local_363;
	local_365 = local_235 - local_69;
	local_366 = local_365 * local_365;
	local_367 = local_366 + local_364;
	local_368 = sqrt(local_367);
	local_369 = local_368 - const_362;
	lstatus = (local_369 < 0);
	if (lstatus)
		local_371 = local_356;
	else
		local_371 = local_361;
	output_ports[0] = local_371;
	output_ports[1] = local_342;
	output_ports[2] = local_284;
	output_ports[3] = local_282;
	output_ports[4] = local_330;
	output_ports[5] = local_329;
	memory[379 - min_global] = local_328;
	memory[380 - min_global] = local_69;
	memory[381 - min_global] = local_51;
	memory[382 - min_global] = local_80;
	memory[383 - min_global] = local_327;
	memory[384 - min_global] = local_324;
	memory[385 - min_global] = local_235;
	memory[386 - min_global] = local_187;
	memory[387 - min_global] = local_321;
	memory[388 - min_global] = local_302;
	memory[389 - min_global] = local_296;
	memory[390 - min_global] = local_207;
	memory[391 - min_global] = local_102;
	memory[392 - min_global] = local_281;
	memory[393 - min_global] = local_269;
	memory[394 - min_global] = local_264;
	memory[395 - min_global] = local_336;
	memory[396 - min_global] = local_250;
	memory[397 - min_global] = local_342;
	memory[398 - min_global] = local_24;
	memory[399 - min_global] = local_22;
	memory[400 - min_global] = local_14;
	status = lstatus;

	vm_state::step();
}
#endif
