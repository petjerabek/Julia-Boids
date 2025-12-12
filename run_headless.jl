using Pkg

using Boids
using BenchmarkTools

function run_benchmark()
    cfg = SimConfig(n_boids=2000)
    sim = Simulation(cfg)
    
    step!(sim) # compile
    
    println("Benchmarking step with $(cfg.n_boids) boids...")
    @btime step!($sim)
end

run_benchmark()