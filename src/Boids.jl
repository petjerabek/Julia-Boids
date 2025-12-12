module Boids

using StaticArrays
using StructArrays
using LinearAlgebra
using Random

export SimConfig, Simulation, step!, randomize!

include("Config.jl")
include("Simulation.jl")

end