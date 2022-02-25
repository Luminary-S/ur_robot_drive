    delta_F_threshold_2 = [3, 10, 3, 0.02, 0.02, 2]
    F_t = [0, 0, -29, 0, 0, 0]
    F_comp = [0,0,0, 0.168, -0.045, 0 ]
    F_d = list_minus(F_t, F_comp)
    K_ori = np.array([1e8, 1e8, 3000, 5, 5, 1e8])