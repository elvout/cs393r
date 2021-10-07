map = "maps/GDC1.txt"
init_x = 14.7
init_y = 14.24
init_r = 0

-- Error in translation caused by translation.
motion_model_k1 = 0.45
-- Error in translation caused by rotation.
motion_model_k2 = 1.6
-- Error in rotation caused by translation.
motion_model_k3 = 0.65
-- Error in rotation caused by rotation.
motion_model_k4 = 3

lidar_stddev = 0.1  -- meters, inflated

-- The bound around the expected LIDAR reading in which a Gaussian
-- distribution is used.
sensor_model_d_short = -2.5 * lidar_stddev
sensor_model_d_long = 2.5 * lidar_stddev

sensor_model_gamma = 0.1
