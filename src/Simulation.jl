using StructArrays
using LinearAlgebra
using Random
using StaticArrays
using CellListMap

# singele boid data structure
# SVector is stack-allocated (Value Type) --> avoids heap allocations and GC pressure
struct Boid
    pos::SVector{2, Float64}
    vel::SVector{2, Float64}
end

# accumulator for flocking rule contributions
# stack-allocated (no GC overhead), contiguous memory layout
# race condition eliminated via lock-free parallel reduction (thread-safe)
struct Accumulator
    align_vel::SVector{2, Float64}
    coh_pos::SVector{2, Float64}
    sep_vec::SVector{2, Float64}
    count_ac::Int
    count_s::Int
end

# reduction operator for merging thread-local accumulators
import Base: +
function +(a::Accumulator, b::Accumulator)
    return Accumulator(
        a.align_vel + b.align_vel,
        a.coh_pos + b.coh_pos,
        a.sep_vec + b.sep_vec,
        a.count_ac + b.count_ac,
        a.count_s + b.count_s
    )
end

const ZERO_ACC = Accumulator(SVector(0.,0.), SVector(0.,0.), SVector(0.,0.), 0, 0)

# simulation container
# parametric structure allows storing complex types without sacrificing type stability
mutable struct Simulation{T, B, C}
    config::SimConfig
    flock::T
    box::B
    cell_list::C
    accumulators::Vector{Accumulator} # already a concrete type
end

# randomize boid positions and velocities
function randomize_data!(flock, cfg::SimConfig)
    for i in 1:length(flock)
        flock.pos[i] = SVector(rand() * cfg.width, rand() * cfg.height)
        a = rand() * 2π
        flock.vel[i] = SVector(cos(a), sin(a)) * cfg.speed
    end
end

function Simulation(cfg::SimConfig)
    # initilize boids as StructArray for better memory layout (AoS -> SoA)
    # undef for performance; no need to initialize twice
    flock = StructArray{Boid}(undef, cfg.n_boids)
    randomize_data!(flock, cfg)
    
    # spatial partitioning box for CellListMap, auto-wrapping edges
    box = Box([cfg.width, cfg.height], cfg.perception)
    
    # spatial partitioning cell list for CellListMap, accelerates neighbor searches
    cl = CellList(flock.pos, box)
    
    # standard vector of zeroed accumulators, heap-allocated
    acc = fill(ZERO_ACC, cfg.n_boids)

    return Simulation(cfg, flock, box, cl, acc)
end

# API endpoint to randomize the simulation
function randomize!(sim::Simulation)
    randomize_data!(sim.flock, sim.config)
end

# no call overhead magnitude limiter with strict type consistency
@inline function limit_magnitude(v::SVector{2, T}, max_val::T, eps::T) where T
    norm_v = norm(v)
    if norm_v > max_val
        return v * (max_val / (norm_v + eps))
    end
    return v
end

# === KERNEL (PAIRWISE INTERACTION) ===

function interact!(pos_i, pos_j, i, j, d2, out, flock_vel, params)
    d = sqrt(d2)
    
    # unpack parameters
    sep_sq = params.separation_dist^2
    fov_thresh = params.fov_thresh
    eps = params.eps

    # fetch velocities (CellListMap only manages positions)
    vel_i = flock_vel[i]
    vel_j = flock_vel[j]
    
    # vector from i to j
    # pos_j is already relative, wrapped coordinate calculated by CellListMap
    offset_ij = pos_j - pos_i
    
    # --- check i -> j ---
    vel_norm_i = norm(vel_i)
    if vel_norm_i > eps
        cos_theta = dot(vel_i, offset_ij) / (vel_norm_i * d + eps)
        if cos_theta >= fov_thresh # j is within i's FOV
            out[i] = out[i] + Accumulator(
                vel_j,          
                pos_i + offset_ij, # virtual position for cohesion center
                (d2 < sep_sq) ? (-offset_ij / (d2 + eps)) : SVector(0.,0.), 
                1,              
                (d2 < sep_sq) ? 1 : 0 
            )
        end
    end

    # --- check j -> i ---
    offset_ji = -offset_ij
    vel_norm_j = norm(vel_j)
    if vel_norm_j > eps
        cos_theta = dot(vel_j, offset_ji) / (vel_norm_j * d + eps)
        if cos_theta >= fov_thresh # i is within j's FOV
            out[j] = out[j] + Accumulator(
                vel_i,
                pos_j + offset_ji,
                (d2 < sep_sq) ? (-offset_ji / (d2 + eps)) : SVector(0.,0.),
                1,
                (d2 < sep_sq) ? 1 : 0
            )
        end
    end
    
    return out
end

# === PHYSICS LOOP ===

function step!(sim::Simulation)
    cfg = sim.config
    flock = sim.flock
    
    # update spatial partitioning
    sim.cell_list = UpdateCellList!(sim.flock.pos, sim.box, sim.cell_list)
    
    # reset accumulators
    fill!(sim.accumulators, ZERO_ACC)
    
    params = (
        separation_dist = cfg.separation_dist,
        fov_thresh = cos_half_fov(cfg),
        eps = cfg.eps
    )
    
    # run pairwise interactions in parallel using CellListMap
    map_pairwise!(
        (pos_i, pos_j, i, j, d2, out) -> interact!(pos_i, pos_j, i, j, d2, out, flock.vel, params),
        sim.accumulators,
        sim.box,
        sim.cell_list,
        parallel=true
    )
    
    # apply forces and integrate
    @inbounds for i in 1:length(flock)
        acc_data = sim.accumulators[i]
        pos_i = flock.pos[i]
        vel_i = flock.vel[i]
        
        steer_coh = SVector(0.0, 0.0)
        steer_align = SVector(0.0, 0.0)
        steer_sep = SVector(0.0, 0.0)
        
        c_ac = acc_data.count_ac
        c_s = acc_data.count_s
        
        if c_ac > 0
            avg_pos = acc_data.coh_pos / c_ac
            desired = avg_pos - pos_i
            desired = limit_magnitude(desired * cfg.speed, cfg.speed, cfg.eps)
            steer_coh = (desired - vel_i) * cfg.w_coh
            
            avg_vel = acc_data.align_vel / c_ac
            desired = limit_magnitude(avg_vel, cfg.speed, cfg.eps)
            steer_align = (desired - vel_i) * cfg.w_align
        end
        
        if c_s > 0
            avg_sep = acc_data.sep_vec / c_s
            desired = limit_magnitude(avg_sep, cfg.speed, cfg.eps)
            steer_sep = (desired - vel_i) * cfg.w_sep
        end
        
        total_steer = steer_sep + steer_align + steer_coh
        acc = limit_magnitude(total_steer, cfg.max_force, cfg.eps)
        
        # semi-implicit Euler integration
        new_vel = vel_i + (acc * cfg.dt)
        new_vel = limit_magnitude(new_vel, cfg.speed, cfg.eps)
        
        flock.vel[i] = new_vel
        
        new_pos = pos_i + (new_vel * cfg.dt)
        flock.pos[i] = SVector(mod(new_pos[1], cfg.width), mod(new_pos[2], cfg.height))
    end
end