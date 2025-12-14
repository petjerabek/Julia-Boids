Base.@kwdef struct SimConfig
    width::Float64 = 5000.0
    height::Float64 = 5000.0
    n_boids::Int = 10000

    dt::Float64 = 0.02

    speed::Float64 = 200.0
    perception::Float64 = 80.0
    separation_dist::Float64 = 20.0

    w_sep::Float64 = 80.0
    w_align::Float64 = 110.0
    w_coh::Float64 = 10.0
    
    fov_deg::Float64 = 80.0
    max_force::Float64 = 1000.0
    eps::Float64 = 1e-12
end

# helper to pre-calculate FOV threshold
cos_half_fov(c::SimConfig) = cos(deg2rad(c.fov_deg / 2.0))