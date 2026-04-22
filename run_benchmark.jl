using Boids
using BenchmarkTools

function run_benchmark(n_boids::Int)
    println("Initializing Simulation with $n_boids boids...")

    sim = scaled_config(n_boids)

    println("World Size: $(round(Int, sim.width))x$(round(Int, sim.height))")
    println("Running warmup...")
    step!(sim)

    println("Benchmarking step!...")
    @btime step!($sim)
end

run_benchmark(20000)
