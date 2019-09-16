def vel2wheel(v, omega, wheel_dist, wheel_rad):
    
    gain = 1
    trim = 0
    
    # Maximal speed
    if v > 0.5:
        v = 0.5
    elif v < -0.5:
        v = -0.5
    
    
##### Fill the code here:

    left_rate = 0
    right_rate = 0
    
####


    
    
    return left_rate, right_rate