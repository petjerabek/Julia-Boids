using Boids
using BenchmarkTools

function run_benchmark(n_boids::Int)
    println("Initializing Simulation with $n_boids boids...")
    
    # get default config to read the perception radius
    defaults = SimConfig()
    
    # scale world size to achieve ~8 neighbors per boid to preserve constant density and O(N) complexity
    # tagret area = (total boids * vision area) / target neighbors
    vision_area = π * (defaults.perception^2)
    target_area = (n_boids * vision_area) / 8.0
    side_len = sqrt(target_area)
    
    # create simulation with scaled world size
    cfg = SimConfig(
        n_boids = n_boids,
        width = side_len,
        height = side_len
    )
    sim = Simulation(cfg)
    
    println("World Size: $(round(Int, side_len))x$(round(Int, side_len))")
    println("Running warmup...")
    step!(sim) 
    
    println("Benchmarking step!...")
    @btime step!($sim)
end

run_benchmark(10000)