function StateEstimator_Data = Construt_StateEstimator_Data()
StateEstimator_Data.a_world = zeros(3,1);
StateEstimator_Data.rp_Origin2LToe = zeros(3,1);
StateEstimator_Data.rp_Origin2RToe = zeros(3,1);

StateEstimator_Data.ITx_kf = zeros(4,1);
StateEstimator_Data.ITy_kf = zeros(4,1);
StateEstimator_Data.ITz_kf = zeros(4,1);
end