using StructArrays
using LinearAlgebra
using Random
using StaticArrays

# single boid representation
struct Boid
    pos::SVector{2, Float64}
    vel::SVector{2, Float64}
    acc::SVector{2, Float64}
end

# simulation container
# paramtetric struct allows flock to store exact, concrete type of the StructArray
mutable struct Simulation{T}
    config::SimConfig
    flock::T 
end

function Simulation(cfg::SimConfig)
    # empty StructArray
    flock = StructArray{Boid}(undef, cfg.n_boids)
    # compiler auto-detects T = StructArray{Boid, 1, ...}
    sim = Simulation(cfg, flock)
    randomize!(sim)
    return sim
end

function randomize!(sim::Simulation)
    cfg = sim.config
    for i in 1:length(sim.flock)
        sim.flock.pos[i] = SVector(rand() * cfg.width, rand() * cfg.height)
        a = rand() * 2π
        sim.flock.vel[i] = SVector(cos(a), sin(a)) * cfg.speed
        sim.flock.acc[i] = zero(SVector{2, Float64})
    end
end

# === HELPER FUNCTIONS ===

@inline function limit_magnitude(v::SVector{2, T}, max_val::T, eps::T) where T
    norm_v = norm(v)
    if norm_v > max_val
        return v * (max_val / (norm_v + eps))
    end
    return v
end

@inline function wrap_pos(pos::SVector{2, T}, w::T, h::T) where T
    return SVector(mod(pos[1], w), mod(pos[2], h))
end

# === PHYSICS LOOP ===

function step!(sim::Simulation)
    cfg = sim.config
    flock = sim.flock
    N = length(flock)
    
    w, h = cfg.width, cfg.height
    fov_thresh = cos_half_fov(cfg)
    sep_sq = cfg.separation_dist^2
    perc_sq = cfg.perception^2
    
    # calculate forces
    @inbounds for i in 1:N
        pos_i = flock.pos[i]
        vel_i = flock.vel[i]
        
        sep_vec = SVector(0.0, 0.0)
        align_vel = SVector(0.0, 0.0)
        coh_pos = SVector(0.0, 0.0)
        
        count_ac = 0 
        count_s = 0 
        
        for j in 1:N
            if i == j; continue; end
            
            d = flock.pos[j] - pos_i
            dx = (d[1] + w/2) % w - w/2
            dy = (d[2] + h/2) % h - h/2
            offset = SVector(dx, dy)
            
            dist_sq = dot(offset, offset)
            
            if dist_sq >= perc_sq; continue; end
            
            dist = sqrt(dist_sq)
            
            # FOV check
            vel_norm = norm(vel_i)
            visible = false
            
            if vel_norm > cfg.eps
                cos_theta = dot(vel_i, offset) / (vel_norm * dist + cfg.eps)
                if cos_theta >= fov_thresh
                    visible = true
                end
            end
            
            if visible
                align_vel += flock.vel[j]
                coh_pos += (pos_i + offset)
                count_ac += 1
                
                if dist_sq < sep_sq
                    sep_vec -= offset / (dist_sq + cfg.eps)
                    count_s += 1
                end
            end
        end
        
        # apply rules
        steer_coh = SVector(0.0, 0.0)
        steer_align = SVector(0.0, 0.0)
        steer_sep = SVector(0.0, 0.0)
        
        if count_ac > 0
            avg_pos = coh_pos / count_ac
            desired = avg_pos - pos_i
            desired = limit_magnitude(desired * cfg.speed, cfg.speed, cfg.eps)
            steer_coh = (desired - vel_i) * cfg.w_coh
            
            avg_vel = align_vel / count_ac
            desired = limit_magnitude(avg_vel, cfg.speed, cfg.eps)
            steer_align = (desired - vel_i) * cfg.w_align
        end
        
        if count_s > 0
            avg_sep = sep_vec / count_s
            desired = limit_magnitude(avg_sep, cfg.speed, cfg.eps)
            steer_sep = (desired - vel_i) * cfg.w_sep
        end
        
        total_steer = steer_sep + steer_align + steer_coh
        flock.acc[i] = limit_magnitude(total_steer, cfg.max_force, cfg.eps)
    end
    
    # integration (Euler)
    @inbounds for i in 1:N
        flock.vel[i] += flock.acc[i]
        flock.vel[i] = limit_magnitude(flock.vel[i], cfg.speed, cfg.eps)
        
        new_pos = flock.pos[i] + flock.vel[i]
        flock.pos[i] = wrap_pos(new_pos, w, h)
        
        flock.acc[i] = SVector(0.0, 0.0)
    end
end