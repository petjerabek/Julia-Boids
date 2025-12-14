Base.@kwdef struct SimConfig
    width::Float64 = 2300.0
    height::Float64 = 1700.0
    n_boids::Int = 1500
    
    speed::Float64 = 4.5
    perception::Float64 = 100.0
    separation_dist::Float64 = 35.0
    
    w_sep::Float64 = 1.5
    w_align::Float64 = 1.0
    w_coh::Float64 = 0.5
    
    fov_deg::Float64 = 300.0
    max_force::Float64 = 0.03
    eps::Float64 = 1e-12
end

# helper to pre-calculate FOV threshold
cos_half_fov(c::SimConfig) = cos(deg2rad(c.fov_deg / 2.0))