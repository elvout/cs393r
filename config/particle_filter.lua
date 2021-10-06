map = "maps/GDC1.txt"
init_x = 14.7
init_y = 14.24
init_r = 0

-- Error in translation caused by translation.
motion_model_k1 = 0.4
-- Error in translation caused by rotation.
motion_model_k2 = 1
-- Error in rotation caused by translation.
motion_model_k3 = 0.4
-- Error in rotation caused by rotation.
motion_model_k4 = 1


lidar_stddev = 0.1  -- meters, inflated

-- The bound around the expected LIDAR reading in which a Gaussian
-- distribution is used.
sensor_model_d_short = -2 * lidar_stddev
sensor_model_d_long = 1 * lidar_stddev

sensor_model_gamma = 0.3
