using GLMakie, GeometryBasics, LinearAlgebra, Random

mutable struct Boid
    position::Point2f
    velocity::Point2f
end

const N_BOIDS = 100
const BOUNDS = (Point2f(0, 0), Point2f(100, 100))
const SPEED = 0.5

function initialize_boids()
    boids = Vector{Boid}(undef, N_BOIDS)
    min_x, min_y = BOUNDS[1]
    max_x, max_y = BOUNDS[2]
    for i in 1:N_BOIDS
        position = Point2f(rand(min_x:max_x), rand(min_y:max_y))
        angle = rand() * 2π
        velocity = Point2f(cos(angle), sin(angle)) * SPEED
        boids[i] = Boid(position, velocity)
    end
    return boids
end

function update_boids!(boids)
    width = BOUNDS[2][1] - BOUNDS[1][1]
    for boid in boids
        new_position = boid.position += boid.velocity
        boid.position = mod.(new_position .- BOUNDS[1], width) .+ BOUNDS[1]
    end
end

boids = initialize_boids()

boid_position = Observable([boid.position for boid in boids])

fig = Figure(size = (800, 800))
ax = Axis(fig[1, 1], limits = (BOUNDS[1][1], BOUNDS[2][1], BOUNDS[1][2], BOUNDS[2][2]))

hidedecorations!(ax)
scatter!(ax, boid_position, markersize = 10, color = :blue)

display(fig)

@async begin
    while isopen(fig.scene)
        update_boids!(boids)
        boid_position[] = [boid.position for boid in boids]
        sleep(1/60)
    end
end