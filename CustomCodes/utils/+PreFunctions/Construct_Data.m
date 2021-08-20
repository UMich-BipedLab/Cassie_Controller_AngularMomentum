function Data = Construt_Data()
Data.stanceLeg = 0;

Data.Lx_G = 0;
Data.Lx_LToe = 0;
Data.Lx_RToe = 0;
Data.Lx_LToe_vg = 0;
Data.Lx_RToe_vg = 0;

Data.Ly_G = 0;
Data.Ly_LToe = 0;
Data.Ly_RToe = 0;
Data.Ly_LToe_vg = 0;
Data.Ly_RToe_vg = 0;

Data.Lx_stToe = 0;
Data.Ly_stToe = 0;
Data.Lx_swToe = 0;
Data.Ly_swToe = 0;

Data.Lx_stToe_kf = 0;
Data.Ly_stToe_kf = 0;

Data.tg_direction = 0;
Data.stTD0 = 0;
Data.Vx_tgd = 0;
Data.Vy_tgd = 0;

Data.p_stToe = zeros(3,1);
Data.p_swToe = zeros(3,1);
Data.p_LToe = zeros(3,1);
Data.p_RToe = zeros(3,1);

Data.v_stToe = zeros(3,1);
Data.v_swToe = zeros(3,1);
Data.v_LToe = zeros(3,1);
Data.v_RToe = zeros(3,1);

Data.rp_stToe = zeros(3,1);
Data.rv_stToe = zeros(3,1);

Data.rp_LToe = zeros(3,1);
Data.rv_LToe = zeros(3,1);
Data.rp_RToe = zeros(3,1);
Data.rv_RToe = zeros(3,1);

Data.rp_LToe_fil = zeros(3,1);
Data.rp_RToe_fil = zeros(3,1);

Data.x0 = 0;
Data.dx0_next_goal = 0;
Data.x0_next_goal = 0;
Data.dxf_next_goal = 0;

Data.dxf_this_stTD0 = 0;
Data.dx0_next_stTD0 = 0;
Data.x0_next_tgd_goal = 0;
Data.dxf_next_tgd_goal = 0;

Data.vx_com0_next_stTd0 = 0;

Data.dyf_this_stTD0 = 0;
Data.dyf_this_tgd = 0;
Data.dyf_next_tgd_goal = 0;

Data.p_com = zeros(3,1);
Data.v_com = zeros(3,1);
Data.vx_com = 0;
Data.vy_com = 0;
Data.vz_com = 0;
Data.px_com = 0;
Data.py_com = 0;
Data.pz_com = 0;
Data.pseudo_com_vx_stTD0 = 0;

Data.vx_com_tgd = 0;
Data.vy_com_tgd = 0;
Data.Lx_stToe_stTD0_obs = 0;
Data.Lx_stToe_stTD0_kf = 0;
Data.Ly_stToe_stTD0_obs = 0;
Data.Ly_stToe_stTD0_kf = 0;

Data.q = zeros(20,1);
Data.dq = zeros(20,1);
Data.dq_ss = zeros(20,1);
Data.u = zeros(10,1);
Data.s = 0;
Data.t = 0;

Data.u_ff = zeros(8,1);
Data.u_fb = zeros(8,1);
Data.u_gv = zeros(8,1);

Data.qmd = zeros(10,1);
Data.dqmd = zeros(10,1);
Data.qm0 = zeros(10,1);
Data.dqm0 = zeros(10,1);

Data.qsL_1 = 0;
Data.qsL_2 = 0;
Data.qsR_1 = 0;
Data.qsR_2 = 0;

Data.dqsL_1 = 0;
Data.dqsL_2 = 0;
Data.dqsR_1 = 0;
Data.dqsR_2 = 0;



Data.hr = zeros(8,1);
Data.dhr = zeros(8,1);
Data.ddhr = zeros(8,1);
Data.h0 = zeros(8,1);
Data.dh0 = zeros(8,1);

Data.hr_recover = zeros(8,1);
Data.dhr_recover = zeros(8,1);
Data.iter_num = 0;

Data.qd_control = zeros(8,1);
Data.dqd_control = zeros(8,1);
Data.q0_control = zeros(8,1);
Data.dq0_control = zeros(8,1);
Data.qd_control_adjusted = zeros(8,1);
Data.dqd_control_adjusted = zeros(8,1);

%% test
Data.tau_knee_st_est = 0;
Data.tau_kneespring_st_est = 0;
Data.tau_heelspring_st_est = 0;
Data.comp_knee_st = 0;

Data.kneespring_st_stiffness = 0;
Data.heelspring_st_stiffness = 0;

Data.q1 = 0;
Data.q2 = 0;
Data.q3 = 0;
Data.q4 = 0;
Data.q5 = 0;
Data.q6 = 0;
Data.q7 = 0;
Data.q8 = 0;
Data.q9 = 0;
Data.q10 = 0;
Data.q11 = 0;
Data.q12 = 0;
Data.q13 = 0;
Data.q14 = 0;
Data.q15 = 0;
Data.q16 = 0;
Data.q17 = 0;
Data.q18 = 0;
Data.q19 = 0;
Data.q20 = 0;

Data.dq1 = 0;
Data.dq2 = 0;
Data.dq3 = 0;
Data.dq4 = 0;
Data.dq5 = 0;
Data.dq6 = 0;
Data.dq7 = 0;
Data.dq8 = 0;
Data.dq9 = 0;
Data.dq10 = 0;
Data.dq11 = 0;
Data.dq12 = 0;
Data.dq13 = 0;
Data.dq14 = 0;
Data.dq15 = 0;
Data.dq16 = 0;
Data.dq17 = 0;
Data.dq18 = 0;
Data.dq19 = 0;
Data.dq20 = 0;
end