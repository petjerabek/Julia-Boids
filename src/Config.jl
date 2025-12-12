Base.@kwdef struct SimConfig
    width::Float64 = 800.0
    height::Float64 = 600.0
    n_boids::Int = 1500
    
    speed::Float64 = 2.0 # max speed
    perception::Float64 = 60.0
    separation_dist::Float64 = 25.0
    
    w_sep::Float64 = 1.1
    w_align::Float64 = 1.1
    w_coh::Float64 = 0.9
    
    fov_deg::Float64 = 270.0
    max_force::Float64 = 0.03 # steering force limit
    eps::Float64 = 1e-12
end

# helper to pre-calculate FOV threshold
cos_half_fov(c::SimConfig) = cos(deg2rad(c.fov_deg / 2.0))