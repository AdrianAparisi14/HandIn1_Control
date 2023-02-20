function B = inertiaMatrix(I12,I21,I22,I23,I31,I32,I33,a2,a3,m1,m2,m3,pl1_1,pl1_3,pl2_1,pl2_2,pl2_3,pl3_1,pl3_2,pl3_3,q1,q2,q3)
%inertiaMatrix
%    B = inertiaMatrix(I12,I21,I22,I23,I31,I32,I33,A2,A3,M1,M2,M3,PL1_1,PL1_3,PL2_1,PL2_2,PL2_3,PL3_1,PL3_2,PL3_3,Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    20-Feb-2023 18:21:18

t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
t5 = sin(q2);
t6 = q2+q3;
t7 = t2.^2;
t8 = t3.^2;
t9 = t4.^2;
t10 = t5.^2;
t11 = a2.*t3;
t12 = pl2_2.*t3;
t13 = pl2_3.*t2;
t14 = pl3_3.*t2;
t15 = a2.*t5;
t16 = cos(t6);
t17 = pl2_1.*t5;
t18 = pl2_3.*t4;
t19 = pl3_3.*t4;
t20 = sin(t6);
t23 = I23.*t2.*t4;
t24 = I33.*t2.*t4;
t29 = pl2_1.*t2.*t3;
t32 = pl2_1.*t3.*t4;
t33 = pl2_2.*t2.*t5;
t34 = pl2_2.*t4.*t5;
t38 = I21.*t2.*t3.*t5;
t39 = I22.*t2.*t3.*t5;
t40 = I21.*t3.*t4.*t5;
t41 = I22.*t3.*t4.*t5;
t21 = t16.^2;
t22 = t20.^2;
t25 = a3.*t16;
t26 = t2.*t11;
t27 = pl3_2.*t16;
t28 = a3.*t20;
t30 = t4.*t11;
t31 = pl3_1.*t20;
t35 = -t14;
t36 = I33.*t7;
t37 = I33.*t9;
t42 = pl3_1.*t2.*t16;
t43 = pl3_1.*t4.*t16;
t44 = pl3_2.*t2.*t20;
t45 = -t23;
t46 = -t24;
t47 = pl3_2.*t4.*t20;
t50 = -t32;
t51 = -t33;
t52 = -t39;
t53 = -t41;
t57 = I21.*t2.*t4.*t8;
t58 = I22.*t2.*t4.*t10;
t63 = I31.*t2.*t16.*t20;
t64 = I32.*t2.*t16.*t20;
t66 = I31.*t4.*t16.*t20;
t67 = I32.*t4.*t16.*t20;
t70 = t12+t15+t17;
t48 = -t26;
t49 = -t30;
t54 = -t43;
t55 = -t44;
t56 = -t47;
t59 = t11+t25;
t60 = I32.*t9.*t22;
t61 = I31.*t2.*t4.*t21;
t62 = I32.*t2.*t4.*t22;
t65 = I31.*t7.*t21;
t68 = I31.*t9.*t21;
t69 = I32.*t7.*t22;
t73 = -t64;
t74 = -t67;
t75 = t70.^2;
t77 = t38+t52;
t78 = t27+t28+t31;
t79 = t40+t53;
t85 = t18+t26+t29+t51;
t90 = t45+t57+t58;
t71 = t2.*t59;
t72 = t4.*t59;
t80 = t78.^2;
t81 = t4.*t77;
t82 = t2.*t79;
t83 = t15+t78;
t87 = t63+t73;
t88 = t66+t74;
t89 = t13+t34+t49+t50;
t93 = t37+t65+t69;
t94 = t36+t60+t68;
t96 = t46+t61+t62;
t106 = t4.*t70.*t85;
t76 = -t72;
t84 = -t82;
t86 = t83.^2;
t91 = t4.*t87;
t92 = t2.*t88;
t97 = t2.*t94;
t98 = t4.*t93;
t99 = t2.*t96;
t100 = t4.*t96;
t101 = t19+t42+t55+t71;
t104 = t35+t43+t56+t72;
t108 = t2.*t70.*t89;
t111 = t7.*t78.*t83;
t112 = t9.*t78.*t83;
t95 = -t92;
t102 = -t99;
t103 = -t100;
t105 = t2.*t101;
t107 = -t4.*(t14+t47+t54+t76);
t109 = t48+t101;
t110 = t14+t30+t47+t54+t76;
t116 = t4.*t78.*t101;
t117 = -t2.*t78.*(t14+t47+t54+t76);
t118 = t2.*t78.*(t14+t47+t54+t76);
t119 = t4.*t83.*t101;
t120 = -t2.*t83.*(t14+t47+t54+t76);
t127 = t106+t108;
t113 = t4.*t110;
t114 = t2.*t109;
t121 = -t119;
t122 = t98+t102;
t123 = t97+t103;
t126 = t105+t107;
t128 = m2.*t127;
t131 = t116+t118;
t135 = -m3.*(t119+t2.*t83.*(t14+t47+t54+t76));
t115 = -t114;
t124 = t2.*t123;
t125 = t4.*t122;
t129 = -t128;
t132 = m3.*t131;
t134 = t120+t121;
t130 = t113+t115;
t133 = -t132;
t141 = t81+t84+t91+t95+t129+t135;
t136 = t126.*t130;
t138 = t91+t95+t133;
t137 = -t136;
t139 = t111+t112+t137;
t140 = m3.*t139;
t142 = t124+t125+t140;
B = reshape([I12+I22.*t8+I21.*t10+I31.*t22+I32.*t21+m3.*((t14+t47+t54+t76).^2+t101.^2)+m1.*((pl1_1.*t2+pl1_3.*t4).^2+(pl1_1.*t4-pl1_3.*t2).^2)+m2.*(t85.^2+t89.^2),t141,t138,t141,t124+t125+m2.*(t7.*t75+t9.*t75+(t2.*t85-t4.*t89).^2)+m3.*(t7.*t86+t9.*t86+t126.^2)-t2.*(t4.*t90-t2.*(I23.*t7+I21.*t8.*t9+I22.*t9.*t10))-t4.*(t2.*t90-t4.*(I23.*t9+I21.*t7.*t8+I22.*t7.*t10)),t142,t138,t142,t124+t125+m3.*(t7.*t80+t9.*t80+t130.^2)],[3,3]);
