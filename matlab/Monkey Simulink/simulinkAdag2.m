function Adag2 = simulinkAdag2(p,q1,q2,q3,q4)
%SIMULINKADAG2
%    ADAG2 = SIMULINKADAG2(P,Q1,Q2,Q3,Q4)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    21-Apr-2020 12:54:00

t2 = cos(p);
t3 = sin(p);
t4 = p+q1;
t5 = p+q3;
t6 = p.*2.0;
t7 = q1.*2.0;
t8 = q2.*2.0;
t9 = q3.*2.0;
t10 = q3.*3.0;
t11 = q4.*2.0;
t27 = -p;
t28 = -q1;
t30 = -q2;
t32 = -q3;
t34 = -q4;
t12 = cos(t6);
t13 = cos(t7);
t14 = cos(t8);
t15 = cos(t9);
t16 = cos(t11);
t17 = sin(t6);
t18 = sin(t7);
t19 = sin(t9);
t20 = sin(t11);
t21 = cos(t4);
t22 = cos(t5);
t23 = q2+t4;
t24 = q4+t5;
t25 = sin(t4);
t26 = sin(t5);
t29 = -t7;
t31 = -t8;
t33 = -t9;
t35 = -t11;
t36 = q1+t4;
t37 = p+t8;
t38 = q3+t5;
t39 = p+t11;
t44 = t4+t8;
t45 = t4+t9;
t47 = t4+t11;
t48 = t5+t8;
t49 = t5+t11;
t55 = p+t28;
t57 = p+t32;
t58 = t7+t8;
t59 = t7+t11;
t60 = t8+t9;
t61 = t8+t11;
t62 = t9+t11;
t66 = t4+t30;
t68 = q1+q2+t27;
t71 = t5+t34;
t73 = q3+q4+t27;
t93 = t4.*2.0;
t94 = t6+t8;
t95 = t5.*2.0;
t96 = t6+t11;
t137 = q1+t8+t27;
t143 = q3+t8+t27;
t40 = cos(t36);
t41 = cos(t37);
t42 = cos(t38);
t43 = cos(t39);
t46 = q3+t36;
t50 = t9+t24;
t51 = sin(t36);
t52 = sin(t37);
t53 = sin(t38);
t54 = sin(t39);
t56 = p+t31;
t63 = cos(t55);
t65 = cos(t57);
t67 = q2+t55;
t69 = t5+t29;
t70 = t5+t31;
t72 = q4+t57;
t74 = sin(t55);
t76 = sin(t57);
t77 = cos(t44);
t78 = cos(t45);
t80 = cos(t47);
t81 = cos(t48);
t82 = cos(t49);
t83 = t9+t23;
t84 = t11+t23;
t86 = t8+t24;
t87 = sin(t44);
t88 = sin(t45);
t90 = sin(t47);
t91 = sin(t48);
t92 = sin(t49);
t97 = t7+t33;
t98 = t7+t35;
t99 = t8+t33;
t100 = t8+t35;
t101 = cos(t93);
t102 = cos(t94);
t103 = cos(t95);
t104 = cos(t96);
t105 = cos(t58);
t106 = cos(t59);
t107 = cos(t60);
t108 = cos(t61);
t109 = cos(t62);
t110 = t8+t36;
t111 = t11+t36;
t112 = t8+t38;
t113 = t11+t37;
t114 = t11+t38;
t115 = t9+t49;
t116 = sin(t93);
t117 = sin(t94);
t118 = sin(t95);
t119 = sin(t96);
t120 = sin(t58);
t121 = sin(t59);
t122 = sin(t60);
t123 = sin(t61);
t124 = sin(t62);
t127 = t24+t29;
t128 = t24+t31;
t131 = t6+t31;
t138 = t28+t38;
t139 = t32+t36;
t140 = t28+t39;
t141 = t29+t38;
t142 = t32+t37;
t144 = t31+t38;
t145 = t31+t39;
t156 = t9+t44;
t158 = t11+t44;
t159 = t11+t45;
t161 = t11+t48;
t175 = t23+t62;
t183 = t4+t44;
t184 = t4+t47;
t185 = t5+t48;
t186 = t6+t61;
t187 = t5+t49;
t188 = t11+t58;
t189 = t11+t60;
t190 = cos(t137);
t196 = cos(t143);
t199 = t30+t45;
t201 = t30+t47;
t205 = t29+t49;
t206 = t34+t48;
t208 = t8+t73;
t210 = t31+t49;
t212 = sin(t137);
t218 = sin(t143);
t228 = t36+t61;
t229 = t38+t61;
t230 = t48+t62;
t244 = t31+t96;
t245 = t33+t58;
t246 = t35+t58;
t247 = t29+t62;
t248 = t31+t62;
t278 = t44+t62;
t294 = t44+t47;
t295 = t48+t49;
t338 = q4-t5+t58;
t353 = t14.*2.38704684053025e+67;
t354 = t12.*8.855906007025734e+66;
t355 = t13.*1.321730100495852e+67;
t356 = t17.*8.855906007025734e+66;
t357 = t18.*1.321730100495852e+67;
t358 = t16.*6.631808292533047e+68;
t359 = t20.*6.631808292533047e+68;
t361 = t15.*6.239089986248756e+67;
t363 = t19.*6.239089986248756e+67;
t364 = t13.*1.0931135650861e+69;
t365 = t13.*3.279340695258299e+69;
t366 = t13.*4.372454260344398e+69;
t367 = t15.*1.541079468892648e+69;
t368 = t15.*4.623238406677945e+69;
t369 = t15.*6.164317875570594e+69;
t372 = t14.*2.489734498083521e+70;
t376 = t13.*9.838022085774896e+69;
t380 = t15.*1.386971522003384e+70;
t388 = t16.*9.569120460832383e+70;
t397 = t14.*6.224336245208802e+69;
t416 = t14.*5.601902620687922e+70;
t418 = t16.*7.176840345624287e+70;
t420 = t16.*2.392280115208096e+70;
t429 = t14.*1.867300873562641e+70;
t446 = t16.*2.153052103687286e+71;
t64 = cos(t56);
t75 = sin(t56);
t79 = cos(t46);
t85 = q4+t46;
t89 = sin(t46);
t125 = cos(t69);
t126 = cos(t70);
t129 = sin(t69);
t130 = sin(t70);
t132 = cos(t131);
t133 = cos(t97);
t134 = cos(t98);
t135 = cos(t99);
t136 = cos(t100);
t146 = sin(t131);
t147 = sin(t97);
t148 = sin(t99);
t149 = sin(t100);
t150 = cos(t110);
t151 = cos(t111);
t152 = cos(t112);
t153 = cos(t113);
t154 = cos(t114);
t155 = cos(t115);
t157 = t8+t46;
t160 = t11+t46;
t162 = t8+t50;
t163 = sin(t110);
t164 = sin(t111);
t165 = sin(t112);
t166 = sin(t113);
t167 = sin(t114);
t168 = sin(t115);
t169 = cos(t156);
t171 = cos(t158);
t172 = cos(t159);
t174 = cos(t161);
t177 = sin(t156);
t179 = sin(t158);
t180 = sin(t159);
t182 = sin(t161);
t191 = cos(t138);
t192 = cos(t139);
t193 = cos(t140);
t194 = cos(t141);
t195 = cos(t142);
t197 = cos(t144);
t198 = cos(t145);
t200 = q2+t138;
t202 = q2+t140;
t203 = t34+t46;
t204 = q4+t139;
t207 = q4+t142;
t209 = t29+t50;
t211 = t31+t50;
t213 = sin(t138);
t214 = sin(t139);
t215 = sin(t140);
t216 = sin(t141);
t217 = sin(t142);
t219 = sin(t144);
t220 = sin(t145);
t221 = cos(t183);
t222 = cos(t184);
t223 = cos(t185);
t224 = cos(t186);
t225 = cos(t187);
t226 = cos(t188);
t227 = cos(t189);
t231 = sin(t183);
t232 = sin(t184);
t233 = sin(t185);
t234 = sin(t186);
t235 = sin(t187);
t236 = sin(t188);
t237 = sin(t189);
t238 = cos(t205);
t239 = cos(t210);
t240 = sin(t205);
t241 = sin(t210);
t242 = t5+t69;
t243 = t5+t70;
t249 = t31+t69;
t250 = t34+t69;
t251 = t34+t70;
t252 = q4+t32+t56;
t255 = cos(t244);
t256 = cos(t245);
t257 = cos(t246);
t258 = cos(t247);
t259 = cos(t248);
t260 = t32+t110;
t261 = t28+t114;
t262 = t29+t114;
t263 = t29+t115;
t264 = t31+t114;
t265 = t31+t115;
t268 = sin(t244);
t269 = sin(t245);
t270 = sin(t247);
t271 = sin(t248);
t273 = t31+t127;
t275 = cos(t228);
t276 = cos(t229);
t277 = cos(t230);
t279 = t46+t61;
t280 = sin(t228);
t281 = sin(t229);
t282 = sin(t230);
t283 = cos(t278);
t285 = sin(t278);
t287 = t30+t138;
t288 = t30+t140;
t289 = t31+t138;
t290 = t31+t140;
t291 = t31+t141;
t292 = t34+t139;
t293 = t34+t142;
t302 = t30+t159;
t312 = cos(t294);
t313 = cos(t295);
t314 = sin(t294);
t315 = sin(t295);
t316 = t49+t69;
t317 = t49+t70;
t321 = t31+t205;
t332 = t69+t70;
t333 = t35+t245;
t350 = t69+t210;
t360 = -t354;
t362 = -t356;
t370 = t102.*1.573742422818468e+65;
t371 = t117.*1.573742422818468e+65;
t373 = -t366;
t374 = -t365;
t375 = -t364;
t377 = -t369;
t378 = -t368;
t379 = -t367;
t382 = t105.*9.724868531066844e+65;
t383 = t107.*4.916524036243521e+66;
t385 = t120.*9.724868531066844e+65;
t386 = t122.*4.916524036243521e+66;
t387 = -t372;
t389 = -t376;
t390 = -t380;
t391 = t106.*3.40669080032627e+67;
t393 = t109.*1.685549802196156e+67;
t394 = t121.*3.40669080032627e+67;
t396 = t124.*1.685549802196156e+67;
t398 = t103.*5.580898467129476e+68;
t399 = t101.*7.008639828072512e+67;
t400 = -t388;
t401 = t118.*5.580898467129476e+68;
t402 = t116.*7.008639828072512e+67;
t405 = t104.*7.413940417007138e+67;
t406 = t105.*8.042780977750348e+67;
t407 = t105.*2.412834293325104e+68;
t408 = t105.*3.217112391100139e+68;
t409 = t105.*7.238502879975313e+68;
t410 = t106.*5.465567825430498e+68;
t411 = t107.*1.632777171058208e+68;
t412 = t107.*4.898331513174625e+68;
t413 = t107.*6.531108684232833e+68;
t414 = t109.*3.795546893143553e+68;
t415 = t119.*7.413940417007138e+67;
t417 = -t397;
t432 = t106.*1.639670347629149e+69;
t433 = t106.*2.186227130172199e+69;
t434 = t106.*4.919011042887448e+69;
t438 = t107.*1.469499453952387e+69;
t439 = t109.*1.138664067943066e+69;
t440 = t109.*1.518218757257421e+69;
t441 = t109.*3.415992203829197e+69;
t447 = -t416;
t448 = t108.*5.95912584353199e+67;
t449 = t123.*5.95912584353199e+67;
t451 = -t418;
t452 = -t420;
t458 = -t429;
t464 = -t446;
t510 = t108.*7.888443886550841e+69;
t524 = t108.*5.916332914913131e+69;
t527 = t108.*1.97211097163771e+69;
t538 = t108.*1.774899874473939e+70;
t170 = cos(t157);
t173 = cos(t160);
t176 = t8+t85;
t178 = sin(t157);
t181 = sin(t160);
t253 = cos(t242);
t254 = cos(t243);
t266 = sin(t242);
t267 = sin(t243);
t272 = cos(t249);
t274 = sin(t249);
t284 = cos(t279);
t286 = sin(t279);
t296 = cos(t260);
t297 = cos(t261);
t298 = cos(t262);
t299 = cos(t263);
t300 = cos(t264);
t301 = cos(t265);
t303 = q2+t261;
t304 = t34+t157;
t305 = q4+t260;
t306 = sin(t260);
t307 = sin(t261);
t308 = sin(t262);
t309 = sin(t263);
t310 = sin(t264);
t311 = sin(t265);
t318 = cos(t289);
t319 = cos(t290);
t320 = cos(t291);
t322 = t31+t209;
t323 = sin(t289);
t324 = sin(t290);
t325 = sin(t291);
t326 = cos(t316);
t327 = cos(t317);
t328 = sin(t316);
t329 = sin(t317);
t330 = cos(t321);
t331 = sin(t321);
t334 = cos(t332);
t335 = cos(t333);
t336 = t30+t261;
t337 = t34+t260;
t339 = t31+t261;
t340 = t31+t262;
t341 = t31+t263;
t342 = sin(t332);
t343 = sin(t333);
t351 = cos(t350);
t352 = sin(t350);
t381 = t132.*1.573742422818468e+65;
t384 = t146.*1.573742422818468e+65;
t392 = t135.*4.916524036243521e+66;
t395 = t148.*4.916524036243521e+66;
t403 = t227.*6.859810880737509e+65;
t404 = t237.*6.859810880737509e+65;
t419 = -t398;
t421 = -t401;
t422 = t224.*5.842328984488474e+66;
t423 = t226.*2.506534438970494e+66;
t424 = t259.*6.859810880737509e+65;
t425 = t234.*5.842328984488474e+66;
t426 = t236.*2.506534438970494e+66;
t427 = t271.*6.859810880737509e+65;
t428 = t221.*5.15673365417299e+66;
t430 = -t405;
t431 = t134.*5.465567825430498e+68;
t435 = t135.*1.632777171058208e+68;
t436 = t135.*4.898331513174625e+68;
t437 = t135.*6.531108684232833e+68;
t442 = t231.*5.15673365417299e+66;
t443 = t133.*1.520182880795709e+68;
t444 = -t415;
t445 = t147.*1.520182880795709e+68;
t450 = t133.*5.271969964350629e+69;
t453 = t255.*5.842328984488474e+66;
t454 = t226.*4.021390488875174e+67;
t455 = t227.*4.021390488875174e+67;
t456 = t268.*5.842328984488474e+66;
t459 = t134.*1.639670347629149e+69;
t460 = t134.*2.186227130172199e+69;
t461 = t134.*4.919011042887448e+69;
t462 = t135.*1.469499453952387e+69;
t465 = t232.*1.806440574829736e+68;
t466 = t136.*5.95912584353199e+67;
t467 = t225.*2.36944595368228e+68;
t468 = t149.*5.95912584353199e+67;
t469 = t235.*2.36944595368228e+68;
t470 = t256.*1.118502079461655e+67;
t472 = t269.*1.118502079461655e+67;
t473 = t133.*1.581590989305189e+70;
t475 = t257.*4.021390488875174e+67;
t477 = t226.*1.206417146662552e+68;
t478 = t226.*1.60855619555007e+68;
t479 = t226.*3.619251439987657e+68;
t480 = t259.*4.021390488875174e+67;
t482 = t227.*1.206417146662552e+68;
t483 = t227.*1.60855619555007e+68;
t484 = t227.*3.619251439987657e+68;
t486 = t133.*4.744772967915566e+70;
t487 = t133.*2.108787985740252e+70;
t488 = t222.*1.806440574829736e+68;
t492 = t256.*3.878947357241026e+68;
t494 = t257.*1.206417146662552e+68;
t495 = t257.*1.60855619555007e+68;
t496 = t257.*3.619251439987657e+68;
t501 = t259.*1.206417146662552e+68;
t502 = t259.*1.60855619555007e+68;
t503 = t259.*3.619251439987657e+68;
t507 = t258.*9.871216768412778e+67;
t509 = t270.*9.871216768412778e+67;
t512 = t223.*5.014812675306777e+67;
t513 = t233.*5.014812675306777e+67;
t514 = t256.*3.491052621516923e+69;
t515 = t256.*1.55157894289641e+69;
t516 = t256.*1.163684207172308e+69;
t523 = t258.*1.29844110072503e+69;
t525 = t258.*3.895323302175091e+69;
t526 = t136.*7.888443886550841e+69;
t528 = t258.*5.193764402900121e+69;
t529 = t312.*1.329121360920321e+67;
t530 = t314.*1.329121360920321e+67;
t531 = t258.*1.168596990652527e+70;
t539 = t136.*5.916332914913131e+69;
t540 = t136.*1.97211097163771e+69;
t543 = t313.*1.927217307031671e+67;
t544 = t315.*1.927217307031671e+67;
t549 = t136.*1.774899874473939e+70;
t344 = cos(t339);
t345 = cos(t340);
t346 = cos(t341);
t347 = sin(t339);
t348 = sin(t340);
t349 = sin(t341);
t457 = -t428;
t463 = -t442;
t471 = -t450;
t474 = t253.*2.866849376049002e+67;
t476 = -t454;
t481 = -t455;
t485 = t266.*2.866849376049002e+67;
t489 = -t467;
t490 = -t469;
t491 = -t473;
t493 = -t475;
t497 = -t479;
t498 = -t478;
t499 = -t477;
t500 = -t480;
t504 = -t484;
t505 = -t483;
t506 = -t482;
t508 = -t486;
t511 = -t487;
t517 = -t496;
t518 = -t495;
t519 = -t494;
t520 = -t503;
t521 = -t502;
t522 = -t501;
t532 = t254.*5.014812675306777e+67;
t533 = t267.*5.014812675306777e+67;
t534 = t334.*2.109336336517445e+66;
t535 = t326.*1.86157152477312e+67;
t536 = t342.*2.109336336517445e+66;
t537 = t328.*1.86157152477312e+67;
t541 = -t529;
t542 = -t530;
t545 = t335.*7.262926468759648e+66;
t546 = t343.*7.262926468759648e+66;
t550 = t327.*1.927217307031671e+67;
t551 = t329.*1.927217307031671e+67;
t552 = t335.*2.866054649541727e+68;
t553 = t335.*8.598163948625182e+68;
t554 = t335.*3.82140619938897e+68;
t555 = t335.*9.553515498472424e+67;
t557 = t351.*1.369684955559733e+66;
t558 = t352.*1.369684955559733e+66;
t547 = -t534;
t548 = -t536;
t556 = -t552;
t559 = -t553;
t560 = -t554;
t561 = -t555;
t562 = -t557;
t563 = -t558;
t564 = t375+t379+t406+t410+t411+t414+t417+t431+t435+t452+t471+t476+t481+t492+t493+t500+t523+t527+t540+t561+3.689892373438008e+70;
t565 = t373+t377+t387+t400+t408+t413+t433+t437+t440+t460+t498+t505+t510+t511+t515+t518+t521+t526+t528+t560+1.475956949375203e+71;
t568 = t374+t378+t407+t412+t432+t436+t439+t451+t458+t459+t491+t499+t506+t516+t519+t522+t524+t525+t539+t556+1.106967712031402e+71;
t570 = t389+t390+t409+t434+t438+t441+t447+t461+t462+t464+t497+t504+t508+t514+t517+t520+t531+t538+t549+t559+3.320903136094207e+71;
t566 = 1.0./t564;
t567 = 1.0./t565;
t569 = 1.0./t568;
t571 = 1.0./t570;
Adag2 = reshape([t569.*(t2.*5.304613938439873e+65+t21.*3.026821153266964e+65+t40.*2.111389936894333e+65-t41.*4.372788771006464e+64-t42.*1.840391035282302e+66+t43.*2.186945453527724e+66+t63.*2.737328317401108e+64-t64.*4.372788771006464e+64-t77.*2.589312810733296e+64-t78.*5.937299888864306e+64+t80.*1.143698914001614e+66-t150.*1.553493375569783e+64+t151.*5.441998083588291e+65+t152.*1.67932243310992e+65-t153.*1.995546864600712e+65-t154.*8.17129034251582e+65+t169.*9.875470996760986e+63-t171.*1.272074644543824e+65-t172.*3.855350232030984e+64-t190.*4.552979791000246e+63-t191.*9.624626096619701e+65+t193.*7.055321803494182e+64-t194.*4.579631591132592e+65+t197.*1.67932243310992e+65-t198.*1.995546864600712e+65-t275.*4.004048624553505e+64+t276.*6.900531324807271e+64+t283.*6.412578126664315e+63-t297.*4.573578563025071e+65-t298.*2.973756429350032e+65+t300.*6.900531324807271e+64+t318.*1.070495273785619e+65-t319.*1.173506933242505e+64+t320.*3.369546863446398e+64+t344.*4.163326453189931e+64+t345.*2.187995136676891e+64).*8.0e+4,t569.*(t3.*5.304613938439873e+65+t25.*3.026821153266964e+65+t51.*2.111389936894333e+65-t52.*4.372788771006464e+64-t53.*1.840391035282302e+66+t54.*2.186945453527724e+66+t74.*2.737328317401108e+64-t75.*4.372788771006464e+64-t87.*2.589312810733296e+64-t88.*5.937299888864306e+64+t90.*1.143698914001614e+66-t163.*1.553493375569783e+64+t164.*5.441998083588291e+65+t165.*1.67932243310992e+65-t166.*1.995546864600712e+65-t167.*8.17129034251582e+65+t177.*9.875470996760986e+63-t179.*1.272074644543824e+65-t180.*3.855350232030984e+64+t212.*4.552979791000246e+63-t213.*9.624626096619701e+65+t215.*7.055321803494182e+64-t216.*4.579631591132592e+65+t219.*1.67932243310992e+65-t220.*1.995546864600712e+65-t280.*4.004048624553505e+64+t281.*6.900531324807271e+64+t285.*6.412578126664315e+63-t307.*4.573578563025071e+65-t308.*2.973756429350032e+65+t310.*6.900531324807271e+64+t323.*1.070495273785619e+65-t324.*1.173506933242505e+64+t325.*3.369546863446398e+64+t347.*4.163326453189931e+64+t348.*2.187995136676891e+64).*8.0e+4,t567.*(t21.*3.026821153266964e+66+t63.*2.737328317401108e+65-t77.*2.589312810733296e+65-t78.*5.937299888864306e+65+t80.*1.143698914001614e+67+t169.*9.875470996760986e+64-t171.*1.272074644543824e+66-t172.*3.855350232030984e+65-t190.*4.552979791000246e+64-t191.*9.624626096619701e+66+t193.*7.055321803494182e+65+t283.*6.412578126664315e+64-t297.*4.573578563025071e+66+t318.*1.070495273785619e+66-t319.*1.173506933242505e+65+t344.*4.163326453189931e+65-cos(t23).*3.520637920607437e+65+cos(t66).*1.649101910908861e+66+cos(t67).*2.899737583865569e+65-cos(t68).*1.830374107059832e+65+cos(t83).*3.970104686873776e+65-cos(t84).*3.338392041500493e+66+cos(t175).*2.57796377347109e+65-cos(t199).*6.289567650681596e+65-cos(t200).*6.817854506805679e+66+cos(t201).*8.10168905989322e+66+cos(t202).*7.473923266640592e+65+cos(t287).*2.809373583380689e+66-cos(t288).*4.717694353285033e+65-cos(t302).*4.084093199824566e+65-cos(t303).*2.65157210099638e+66+cos(t336).*7.034792672243827e+65).*(-1.066666666666667e+4),t567.*(t25.*3.026821153266964e+66+t74.*2.737328317401108e+65-t87.*2.589312810733296e+65-t88.*5.937299888864306e+65+t90.*1.143698914001614e+67+t177.*9.875470996760986e+64-t179.*1.272074644543824e+66-t180.*3.855350232030984e+65+t212.*4.552979791000246e+64-t213.*9.624626096619701e+66+t215.*7.055321803494182e+65+t285.*6.412578126664315e+64-t307.*4.573578563025071e+66+t323.*1.070495273785619e+66-t324.*1.173506933242505e+65+t347.*4.163326453189931e+65-sin(t23).*3.520637920607437e+65+sin(t66).*1.649101910908861e+66+sin(t67).*2.899737583865569e+65+sin(t68).*1.830374107059832e+65+sin(t83).*3.970104686873776e+65-sin(t84).*3.338392041500493e+66+sin(t175).*2.57796377347109e+65-sin(t199).*6.289567650681596e+65-sin(t200).*6.817854506805679e+66+sin(t201).*8.10168905989322e+66+sin(t202).*7.473923266640592e+65+sin(t287).*2.809373583380689e+66-sin(t288).*4.717694353285033e+65-sin(t302).*4.084093199824566e+65-sin(t303).*2.65157210099638e+66+sin(t336).*7.034792672243827e+65).*(-1.066666666666667e+4),t569.*(t2.*2.121845575375949e+66-t22.*1.741791135374081e+67+t40.*8.445559747577332e+65-t41.*1.749115508402586e+65-t42.*7.361564141129206e+66+t43.*8.747781814110894e+66-t64.*1.749115508402586e+65+t65.*3.728343553184239e+65+t79.*2.893569949580259e+65+t81.*1.472269142393422e+66+t82.*1.831287686478814e+67+t125.*2.893569949580259e+65+t126.*1.472269142393422e+66-t150.*6.213973502279133e+64+t151.*2.176799233435316e+66+t152.*6.717289732439681e+65-t153.*7.982187458402847e+65-t154.*3.268516137006328e+66-t155.*4.430408443917984e+65-t170.*2.128996482304296e+64-t173.*2.893569949580259e+65-t174.*1.552900890696704e+66+t192.*1.275451112413517e+66-t194.*1.831852636453037e+66-t195.*3.950188398704394e+64-t196.*3.950188398704394e+64+t197.*6.717289732439681e+65-t198.*7.982187458402847e+65-t238.*2.893569949580259e+65-t239.*1.552900890696704e+66-t272.*2.128996482304296e+64-t275.*1.601619449821402e+65+t276.*2.760212529922909e+65+t277.*4.694027732970022e+64+t284.*2.128996482304296e+64-t296.*9.38436249682984e+64-t298.*1.189502571740013e+66-t299.*1.515624646075151e+66+t300.*2.760212529922909e+65+t301.*4.694027732970022e+64+t320.*1.347818745378559e+65+t330.*2.128996482304296e+64+t345.*8.751980546707562e+64+t346.*1.115148275733152e+65).*2.0e+4,t569.*(t3.*2.121845575375949e+66-t26.*1.741791135374081e+67+t51.*8.445559747577332e+65-t52.*1.749115508402586e+65-t53.*7.361564141129206e+66+t54.*8.747781814110894e+66-t75.*1.749115508402586e+65+t76.*3.728343553184239e+65+t89.*2.893569949580259e+65+t91.*1.472269142393422e+66+t92.*1.831287686478814e+67+t129.*2.893569949580259e+65+t130.*1.472269142393422e+66-t163.*6.213973502279133e+64+t164.*2.176799233435316e+66+t165.*6.717289732439681e+65-t166.*7.982187458402847e+65-t167.*3.268516137006328e+66-t168.*4.430408443917984e+65-t178.*2.128996482304296e+64-t181.*2.893569949580259e+65-t182.*1.552900890696704e+66+t214.*1.275451112413517e+66-t216.*1.831852636453037e+66-t217.*3.950188398704394e+64+t218.*3.950188398704394e+64+t219.*6.717289732439681e+65-t220.*7.982187458402847e+65-t240.*2.893569949580259e+65-t241.*1.552900890696704e+66-t274.*2.128996482304296e+64-t280.*1.601619449821402e+65+t281.*2.760212529922909e+65+t282.*4.694027732970022e+64+t286.*2.128996482304296e+64-t306.*9.38436249682984e+64-t308.*1.189502571740013e+66-t309.*1.515624646075151e+66+t310.*2.760212529922909e+65+t311.*4.694027732970022e+64+t325.*1.347818745378559e+65+t331.*2.128996482304296e+64+t348.*8.751980546707562e+64+t349.*1.115148275733152e+65).*2.0e+4,t571.*(t22.*-4.180298724897794e+67+t65.*8.948024527642173e+65+t79.*6.944567878992621e+65+t81.*3.533445941744213e+66+t82.*4.395090447549154e+67+t125.*6.944567878992621e+65+t126.*3.533445941744213e+66-t155.*1.063298026540316e+66-t170.*5.10959155753031e+64-t173.*6.944567878992621e+65-t174.*3.726962137672091e+66+t192.*3.061082669792441e+66-t195.*9.480452156890546e+64-t196.*9.480452156890546e+64-t238.*6.944567878992621e+65-t239.*3.726962137672091e+66-t272.*5.10959155753031e+64+t277.*1.126566655912805e+65+t284.*5.10959155753031e+64-t296.*2.252246999239162e+65-t299.*3.637499150580362e+66+t301.*1.126566655912805e+65+t330.*5.10959155753031e+64+t346.*2.676355861759565e+65+cos(t24).*5.813029291040817e+67-cos(t50).*1.265773544837433e+66-cos(t71).*3.618453335496767e+67-cos(t72).*1.265773544837433e+66+cos(t73).*5.740970393985408e+65-cos(t85).*8.266967568968183e+65-cos(t86).*4.912445504894518e+66-cos(t127).*8.266967568968183e+65-cos(t128).*4.912445504894518e+66+cos(t162).*1.341089923951191e+65+cos(t176).*6.082571073220491e+64+cos(t203).*8.266967568968183e+65-cos(t204).*4.330159634692674e+66+cos(t206).*2.982924732738293e+66+cos(t207).*1.341089923951191e+65-cos(t208).*6.082571073220491e+64-cos(t209).*4.330159634692674e+66+cos(t211).*1.341089923951191e+65+cos(t250).*8.266967568968183e+65+cos(t251).*2.982924732738293e+66+cos(t252).*1.341089923951191e+65+cos(t273).*6.082571073220491e+64+cos(t292).*1.963962540170964e+66-cos(t293).*6.082571073220491e+64-cos(t304).*6.082571073220491e+64+cos(t305).*3.185993354476898e+65+cos(t322).*3.185993354476898e+65-cos(t337).*1.445020999063087e+65-cos(t338).*6.082571073220491e+64).*-2.5e+4,t571.*(t26.*-4.180298724897794e+67+t76.*8.948024527642173e+65+t89.*6.944567878992621e+65+t91.*3.533445941744213e+66+t92.*4.395090447549154e+67+t129.*6.944567878992621e+65+t130.*3.533445941744213e+66-t168.*1.063298026540316e+66-t178.*5.10959155753031e+64-t181.*6.944567878992621e+65-t182.*3.726962137672091e+66+t214.*3.061082669792441e+66-t217.*9.480452156890546e+64+t218.*9.480452156890546e+64-t240.*6.944567878992621e+65-t241.*3.726962137672091e+66-t274.*5.10959155753031e+64+t282.*1.126566655912805e+65+t286.*5.10959155753031e+64-t306.*2.252246999239162e+65-t309.*3.637499150580362e+66+t311.*1.126566655912805e+65+t331.*5.10959155753031e+64+t349.*2.676355861759565e+65+sin(t24).*5.813029291040817e+67-sin(t50).*1.265773544837433e+66-sin(t71).*3.618453335496767e+67-sin(t72).*1.265773544837433e+66-sin(t73).*5.740970393985408e+65-sin(t85).*8.266967568968183e+65-sin(t86).*4.912445504894518e+66-sin(t127).*8.266967568968183e+65-sin(t128).*4.912445504894518e+66+sin(t162).*1.341089923951191e+65+sin(t176).*6.082571073220491e+64+sin(t203).*8.266967568968183e+65-sin(t204).*4.330159634692674e+66+sin(t206).*2.982924732738293e+66+sin(t207).*1.341089923951191e+65+sin(t208).*6.082571073220491e+64-sin(t209).*4.330159634692674e+66+sin(t211).*1.341089923951191e+65+sin(t250).*8.266967568968183e+65+sin(t251).*2.982924732738293e+66+sin(t252).*1.341089923951191e+65+sin(t273).*6.082571073220491e+64+sin(t292).*1.963962540170964e+66-sin(t293).*6.082571073220491e+64-sin(t304).*6.082571073220491e+64+sin(t305).*3.185993354476898e+65+sin(t322).*3.185993354476898e+65-sin(t337).*1.445020999063087e+65+sin(t338).*6.082571073220491e+64).*-2.5e+4,t569.*(-t353-t355+t358+t360+t361+t370+t381+t382-t383-t391-t392+t393+t399-t403+t419+t422+t423-t424+t430-t443-t448+t453+t457-t466+t470+t474+t488+t489-t507+t512+t532+t535+t541+t543+t545+t547+t550+t562+1.517326910120979e+68).*2.0e+1,t569.*(-t357+t359+t362+t363+t371+t384+t385-t386-t394+t395+t396+t402-t404+t421+t425+t426-t427+t444+t445-t449+t456+t463+t465+t468-t472+t485+t490-t509+t513+t533+t537+t542+t544-t546+t548+t551+t563).*2.0e+1,t569.*(t357-t359+t362-t363+t371+t384-t385+t386+t394-t395-t396+t402+t404+t421+t425-t426+t427+t444-t445+t449+t456+t463+t465-t468+t472+t485+t490+t509+t513+t533+t537+t542+t544+t546+t548+t551+t563).*2.0e+1,t569.*(t353+t355-t358+t360-t361+t370+t381-t382+t383+t391+t392-t393+t399+t403+t419+t422-t423+t424+t430+t443+t448+t453+t457+t466-t470+t474+t488+t489+t507+t512+t532+t535+t541+t543-t545+t547+t550+t562-1.517326910120979e+68).*-2.0e+1,t566.*(t2.*8.028018956273114e+48+t40.*3.195383987257177e+48-t41.*6.617791898300275e+47-t42.*2.785253420826091e+49+t43.*3.309729937183603e+49-t64.*6.617791898300275e+47-t150.*2.35106162526634e+47+t151.*8.235936541669961e+48+t152.*2.54149170574064e+48-t153.*3.020066727392716e+48-t154.*1.236645579267578e+49-t194.*6.930828454817321e+48+t197.*2.54149170574064e+48-t198.*3.020066727392716e+48-t275.*6.059739433028144e+47+t276.*1.044328520921582e+48-t298.*4.500492074109817e+48+t300.*1.044328520921582e+48+t320.*5.099482527422936e+47+t345.*3.311318530871549e+47).*-1.762033354208707e+21,t566.*(t3.*8.028018956273114e+48+t51.*3.195383987257177e+48-t52.*6.617791898300275e+47-t53.*2.785253420826091e+49+t54.*3.309729937183603e+49-t75.*6.617791898300275e+47-t163.*2.35106162526634e+47+t164.*8.235936541669961e+48+t165.*2.54149170574064e+48-t166.*3.020066727392716e+48-t167.*1.236645579267578e+49-t216.*6.930828454817321e+48+t219.*2.54149170574064e+48-t220.*3.020066727392716e+48-t280.*6.059739433028144e+47+t281.*1.044328520921582e+48-t308.*4.500492074109817e+48+t310.*1.044328520921582e+48+t325.*5.099482527422936e+47+t348.*3.311318530871549e+47).*-1.762033354208707e+21],[2,7]);