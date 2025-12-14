using Boids
using GLMakie

function run_app()
    cfg = SimConfig()
    sim = Simulation(cfg)

    # wrap simulation arrays in observables so Makie knows when to redraw
    obs_points = Observable(sim.flock.pos)
    obs_vels = Observable(sim.flock.vel)
    
    # @lift --> whenever vels change, rotations for markers update automatically
    obs_rotations = @lift begin
        [atan(v[2], v[1]) for v in $obs_vels]
    end

    # create scene
    fig = Figure(resolution = (1200, 800), backgroundcolor = :black)
    ax = Axis(fig[1, 1], 
        backgroundcolor = :black, 
        aspect = DataAspect(),
        limits = (0, cfg.width, 0, cfg.height)
    )
    hidedecorations!(ax) # remove grid and numbers

    # fast scatter plot
    scatter!(ax, obs_points, 
        rotation = obs_rotations,
        marker = '➤', 
        markersize = 3,
        color = :cyan
    )
    
    # text instructions
    text!(ax, 10, 10, text="Space: Pause | R: Randomize", color=:gray, fontsize=14)

    # interaction loop
    running = Observable(true)
    
    on(events(fig).keyboardbutton) do event
        if event.key == Keyboard.space && event.action == Keyboard.press
            running[] = !running[]
            println(running[] ? "Resumed" : "Paused")
        elseif event.key == Keyboard.r && event.action == Keyboard.press
            randomize!(sim)
            notify(obs_points) 
            notify(obs_vels)
        end
    end

    display(fig)

    # initialize timing
    t_prev = time()
    accumulator = 0.0

    # render loop, async --> window stays responsive
    @async while isopen(fig.scene)
        t_now = time()
        delta_time = t_now - t_prev
        t_prev = t_now

        # cap max simulation time per frame to prevent spiral of death
        if delta_time > 0.25
            delta_time = 0.25 
        end

        if running[]
            accumulator += delta_time
            
            # consume accumulated time in fixed steps
            while accumulator >= cfg.dt
                step!(sim)
                accumulator -= cfg.dt
            end
            
            # notify Makie that data has changed
            notify(obs_points)
            notify(obs_vels)
        end
        
        sleep(1/60) # cap to ~60 FPS
    end
end

run_app()