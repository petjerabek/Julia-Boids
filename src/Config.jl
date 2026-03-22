Base.@kwdef struct SimConfig
    width::Float64 = 5000.0 #sim fetaure
    height::Float64 = 5000.0 #sf
    n_boids::Int = 10000 # flock feature

    dt::Float64 = 0.02 # sf

    speed::Float64 = 200.0 #ff
    perception::Float64 = 80.0 #ff
    separation_dist::Float64 = 20.0 #ff

    w_sep::Float64 = 80.0 #ff
    w_align::Float64 = 110.0 # ff
    w_coh::Float64 = 10.0 #ff
    
    fov_deg::Float64 = 80.0
    max_force::Float64 = 1000.0
    eps::Float64 = 1e-12
end

# helper to pre-calculate FOV threshold
cos_half_fov(c::SimConfig) = cos(deg2rad(c.fov_deg / 2.0))

# should live closer to simulation logic?
# helper function is weird, should be above flock
# maybe the config should be proeprty of flock?