using LinearAlgebra
using Random
using StaticArrays
using CellListMap # https://m3g.github.io/CellListMap.jl/stable/

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

Base.zero(::Type{Accumulator}) = Accumulator(SVector(0.,0.), SVector(0.,0.), SVector(0.,0.), 0, 0)

# flock: behavioral config plus agent state (positions, velocities) and per-boid scratch buffers
# SoA layout via parallel vectors of SVectors (stack-allocated, GC-friendly)
struct Flock
    cfg::FlockConfig
    pos::Vector{SVector{2, Float64}}
    vel::Vector{SVector{2, Float64}}
    vel_norms::Vector{Float64}         # cached |vel| to avoid recomputing sqrt
    accumulators::Vector{Accumulator}  # per-boid reduction target for pairwise interactions
end

Base.length(f::Flock) = length(f.pos)
Base.eachindex(f::Flock) = eachindex(f.pos)

function Flock(width::Float64, height::Float64, cfg::FlockConfig)
    n = cfg.n_boids
    f = Flock(
        cfg,
        Vector{SVector{2, Float64}}(undef, n),
        Vector{SVector{2, Float64}}(undef, n),
        Vector{Float64}(undef, n),
        fill(zero(Accumulator), n)
    )
    randomize!(f, width, height)
    return f
end

function randomize!(flock::Flock, width::Float64, height::Float64)
    speed = flock.cfg.speed
    for i in eachindex(flock)
        flock.pos[i] = SVector(rand() * width, rand() * height)
        a = rand() * 2π
        flock.vel[i] = SVector(cos(a), sin(a)) * speed
        flock.vel_norms[i] = speed
    end
end

# simulation container
# parametric structure allows storing complex types without sacrificing type stability
mutable struct Simulation{B, C, A}
    width::Float64
    height::Float64
    dt::Float64
    flock::Flock
    box::B
    cell_list::C
    aux::A  # preallocated AuxThreaded for in-place UpdateCellList!
    # thread-local reduction buffers for CellListMap (reused each step to avoid deepcopy)
    accumulators_threaded::Vector{Vector{Accumulator}}
end

function Simulation(width::Float64, height::Float64, flock_cfg::FlockConfig; dt::Float64 = 0.02)
    flock = Flock(width, height, flock_cfg)

    # spatial partitioning box for CellListMap, auto-wrapping edges
    box = Box([width, height], flock_cfg.perception)

    # spatial partitioning cell list for CellListMap, accelerates neighbor searches
    cl = CellList(flock.pos, box)

    # preallocate AuxThreaded so each UpdateCellList! call reuses buffers
    aux = CellListMap.AuxThreaded(cl)

    # one per-batch accumulator buffer, sized by CellListMap's batch count
    nbatches = cl.nbatches.map_computation
    accumulators_threaded = [fill(zero(Accumulator), flock_cfg.n_boids) for _ in 1:nbatches]

    return Simulation(width, height, dt, flock, box, cl, aux, accumulators_threaded)
end

# API endpoint to randomize the simulation
function randomize!(sim::Simulation)
    randomize!(sim.flock, sim.width, sim.height)
    sim.cell_list = UpdateCellList!(sim.flock.pos, sim.box, sim.cell_list, sim.aux)
end

# no call overhead magnitude limiter with strict type consistency
@inline function limit_magnitude(v::SVector{2, T}, max_val::T, eps::T) where T
    norm_v = norm(v)
    return norm_v > max_val ? v * (max_val / (norm_v + eps)) : v
end

# canonical Reynolds steering: normalize the target direction to desired
# speed, then return (desired - vel) * weight. Returns zero if the target
# direction is near zero — otherwise, a near-zero target produces a braking
# force (-vel * weight) which collapses boids to a standstill.
@inline function steer_toward(target::SVector{2, T}, vel::SVector{2, T},
                              speed::T, eps::T, weight::T) where T
    target_mag = norm(target)
    target_mag > eps || return zero(SVector{2, T})
    desired = target * (speed / target_mag)
    return (desired - vel) * weight
end

# toroidal coordinate wrap for periodic boundary conditions
@inline wrap(p::SVector{2, T}, w::T, h::T) where T = SVector(mod(p[1], w), mod(p[2], h))

# === KERNEL (PAIRWISE INTERACTION) ===

function interact!(pos_i, pos_j, i, j, d2, out, flock_vel, flock_norms, sep_sq, fov_thresh, eps)
    d = sqrt(d2)

    # access pre-calculated norms (memory read is faster than sqrt instruction)
    vel_norm_i = flock_norms[i]
    vel_norm_j = flock_norms[j]

    # vector from i to j
    # pos_j is already relative, wrapped coordinate calculated by CellListMap
    offset_ij = pos_j - pos_i

    within_sep = d2 < sep_sq
    sep_count = within_sep ? 1 : 0

    # --- check i -> j ---
    if vel_norm_i > eps
        cos_theta = dot(flock_vel[i], offset_ij) / (vel_norm_i * d + eps)
        if cos_theta >= fov_thresh # j is within i's FOV
            out[i] = out[i] + Accumulator(
                flock_vel[j],
                pos_i + offset_ij, # virtual position for cohesion center
                within_sep ? (-offset_ij / (d2 + eps)) : zero(SVector{2, Float64}),
                1,
                sep_count
            )
        end
    end

    # --- check j -> i ---
    offset_ji = -offset_ij
    if vel_norm_j > eps
        cos_theta = dot(flock_vel[j], offset_ji) / (vel_norm_j * d + eps)
        if cos_theta >= fov_thresh # i is within j's FOV
            out[j] = out[j] + Accumulator(
                flock_vel[i],
                pos_j + offset_ji,
                within_sep ? (-offset_ji / (d2 + eps)) : zero(SVector{2, Float64}),
                1,
                sep_count
            )
        end
    end

    return out
end

# === PHYSICS LOOP ===

function step!(sim::Simulation)
    flock_cfg = sim.flock.cfg
    flock = sim.flock
    pos = flock.pos
    vel = flock.vel
    vel_norms = flock.vel_norms
    accumulators = flock.accumulators
    accumulators_threaded = sim.accumulators_threaded

    dt = sim.dt
    width = sim.width
    height = sim.height
    speed = flock_cfg.speed
    eps = flock_cfg.eps
    w_coh = flock_cfg.w_coh
    w_align = flock_cfg.w_align
    w_sep = flock_cfg.w_sep
    max_force = flock_cfg.max_force
    sep_sq = flock_cfg.separation_dist^2
    fov_thresh = cos(deg2rad(flock_cfg.fov_deg / 2.0))

    # update spatial partitioning in place using preallocated AuxThreaded
    sim.cell_list = UpdateCellList!(pos, sim.box, sim.cell_list, sim.aux)
    # should be switched for updating or not updating pair list instead

    # reset the merged output and every pre-allocated thread-local buffer
    fill!(accumulators, zero(Accumulator))
    for buf in accumulators_threaded
        fill!(buf, zero(Accumulator))
    end

    # pre-calculate velocity norms
    @inbounds for i in eachindex(flock)
        vel_norms[i] = norm(vel[i])
    end

    # run pairwise interactions in parallel using CellListMap
    map_pairwise!(
        (pos_i, pos_j, i, j, d2, out) -> interact!(
            pos_i, pos_j, i, j, d2, out, vel, vel_norms, sep_sq, fov_thresh, eps),
        accumulators,
        sim.box,
        sim.cell_list;
        output_threaded=accumulators_threaded,
        parallel=true,
    )

    # apply forces and integrate
    @inbounds for i in eachindex(flock)
        acc_data = accumulators[i]
        pos_i = pos[i]
        vel_i = vel[i]

        steer_coh = zero(SVector{2, Float64})
        steer_align = zero(SVector{2, Float64})
        steer_sep = zero(SVector{2, Float64})

        c_ac = acc_data.count_ac
        c_s = acc_data.count_s

        if c_ac > 0
            avg_pos = acc_data.coh_pos / c_ac
            steer_coh = steer_toward(avg_pos - pos_i, vel_i, speed, eps, w_coh)
            # avg_pos - pos_i could be accumulated right way

            avg_vel = acc_data.align_vel / c_ac
            steer_align = steer_toward(avg_vel, vel_i, speed, eps, w_align)
        end

        if c_s > 0
            avg_sep = acc_data.sep_vec / c_s
            steer_sep = steer_toward(avg_sep, vel_i, speed, eps, w_sep)
        end

        total_steer = steer_sep + steer_align + steer_coh
        acc = limit_magnitude(total_steer, max_force, eps)

        # semi-implicit Euler integration, then re-normalize to constant cruising
        # speed so opposing steering rules can't bleed velocity away over time
        new_vel = vel_i + (acc * dt)
        new_vel_mag = norm(new_vel)
        if new_vel_mag > eps
            new_vel = new_vel * (speed / new_vel_mag)
        end

        vel[i] = new_vel
        pos[i] = wrap(pos_i + new_vel * dt, width, height)
    end
end

# optimize temporary data
