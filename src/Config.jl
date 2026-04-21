Base.@kwdef struct SimConfig 
    width::Float64 = 5000.0
    height::Float64 = 5000.0
    dt::Float64 = 0.02
end

# should live closer to simulation logic?
# maybe the config should be proeprty of flock?

Base.@kwdef struct FlockConfig
    n_boids::Int = 10000
    speed::Float64 = 200.0
    perception::Float64 = 80.0
    separation_dist::Float64 = 25.0

    # classic Reynolds ratio ~1.5 : 1.0 : 1.0
    w_sep::Float64 = 60.0
    w_align::Float64 = 40.0
    w_coh::Float64 = 30.0

    fov_deg::Float64 = 270.0
    max_force::Float64 = 1000.0
    eps::Float64 = 1e-12
end

# helper to pre-calculate FOV threshold
cos_half_fov(c::FlockConfig) = cos(deg2rad(c.fov_deg / 2.0))
# helper function is weird, should be above flock

# build configs with world dimensions chosen to keep avg neighbors per boid
# roughly constant at `target_neighbors` — preserves O(N) scaling in benchmarks
# and gives consistent on-screen density in the visualizer
function scaled_config(n_boids::Int; target_neighbors::Float64 = 8.0)
    defaults = FlockConfig()
    vision_area = π * defaults.perception^2
    side_len = sqrt(n_boids * vision_area / target_neighbors)
    return SimConfig(width = side_len, height = side_len), FlockConfig(n_boids = n_boids)
end
