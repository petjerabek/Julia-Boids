module Boids

using StaticArrays
using LinearAlgebra
using Random

export FlockConfig, Simulation, step!, randomize!, scaled_config

include("Config.jl")
include("Simulation.jl")

end