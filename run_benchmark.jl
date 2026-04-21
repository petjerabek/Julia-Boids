using Boids
using BenchmarkTools

function run_benchmark(n_boids::Int)
    println("Initializing Simulation with $n_boids boids...")

    sim_cfg, flock_cfg = scaled_config(n_boids)
    sim = Simulation(sim_cfg, flock_cfg)

    println("World Size: $(round(Int, sim_cfg.width))x$(round(Int, sim_cfg.height))")
    println("Running warmup...")
    step!(sim)

    println("Benchmarking step!...")
    @btime step!($sim)
end

run_benchmark(20000)
