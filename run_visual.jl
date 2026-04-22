using Boids
using GLMakie

# build the figure and observables. caller drives updates by mutating sim
# and calling notify(pos) / notify(rotations).
function build_plot(sim::Simulation)
    pos = Observable(sim.flock.pos)

    rotations_buf = [atan(v[2], v[1]) for v in sim.flock.vel]
    rotations = Observable(rotations_buf)

    fig = Figure(size = (1200, 800), backgroundcolor = :black)
    ax = Axis(
        fig[1, 1],
        backgroundcolor = :black,
        aspect = DataAspect(),
        limits = (0, sim.width, 0, sim.height),
    )
    hidedecorations!(ax)

    # marker sized in world units — scales naturally as world grows with n_boids
    scatter!(ax, pos,
        rotation = rotations,
        marker = '➤',
        markersize = sim.flock.cfg.separation_dist * 0.7,
        markerspace = :data,
        color = :cyan,
    )

    text!(ax, 10, 10, text = "Space: Pause | R: Randomize", color = :gray, fontsize = 14)

    return fig, pos, rotations_buf, rotations
end

function run_app(; n_boids::Int)
    sim = scaled_config(n_boids)

    fig, pos, rotations_buf, rotations = build_plot(sim)

    running = Ref(true)
    accumulator = Ref(0.0)

    on(events(fig).keyboardbutton) do event
        if event.key == Keyboard.space && event.action == Keyboard.press
            running[] = !running[]
            println(running[] ? "Resumed" : "Paused")
        elseif event.key == Keyboard.r && event.action == Keyboard.press
            randomize!(sim)
            vel = sim.flock.vel
            @inbounds for i in eachindex(rotations_buf)
                rotations_buf[i] = atan(vel[i][2], vel[i][1])
            end
            notify(pos)
            notify(rotations)
        end
    end

    # drive physics from Makie's per-frame tick — one physics pass per render,
    # naturally rate-limited to the display, no manual async task or sleep
    on(events(fig.scene).tick) do tick
        running[] || return

        # cap max simulation time per frame to prevent spiral of death
        dt_frame = min(tick.delta_time, 0.25)

        # consume accumulated time in fixed steps so physics is deterministic
        # regardless of frame rate
        accumulator[] += dt_frame
        while accumulator[] >= sim.dt
            step!(sim)
            accumulator[] -= sim.dt
        end

        vel = sim.flock.vel
        @inbounds for i in eachindex(rotations_buf)
            rotations_buf[i] = atan(vel[i][2], vel[i][1])
        end
        notify(pos)
        notify(rotations)
    end

    display(fig)

    # keep the script alive until the window is closed
    while isopen(fig.scene)
        sleep(0.1)
    end
end

run_app(n_boids = 1000)
