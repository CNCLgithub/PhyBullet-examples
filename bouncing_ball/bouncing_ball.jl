using Gen
using PyCall
using PhySMC
using PhyBullet
using Accessors
using UnicodePlots

include(joinpath(@__DIR__, "helpers.jl"))

function draw_trace(tr::Gen.Trace)
    (t, _, _) = get_args(tr)
    # get the prior choice for restitution
    choices = get_choices(tr)
    restitution = choices[:prior => 1 => :restitution]
    # get the z positions
    states = get_retval(tr)
    zs = map(st -> st.kinematics[1].position[3], states)
    plt = lineplot(1:t, zs,
                   title="Height of ball", name="res: $(restitution)",
                   xlabel="t", ylabel="z", canvas=DotCanvas,
                   border=:ascii)
end

function update_plot(plt, tr::Gen.Trace, n::Int)
    (t, _, _) = get_args(tr)
    choices = get_choices(tr)
    restitution = choices[:prior => 1 => :restitution]
    states = get_retval(tr)
    zs = map(st -> st.kinematics[1].position[3], states)
    lineplot!(plt, 1:t, zs,
              name="res: $(restitution)")
end

function main()

    # start with a ball above a table
    client, ball_id = simple_scene()

    # configure simulator with the provided
    # client id
    sim = BulletSim(;client=client)
    # This is the object of interest in the scene
    # (the table is static)
    ball = RigidBody(ball_id)
    # Retrieve the default latents for the ball
    # as well as its initial positions
    # Note: alternative latents will be suggested by the `prior`
    init_state = BulletState(sim, [ball])
    # arguments for `model`
    gargs = (60, # number of steps (1s)
             sim,
             init_state)

    # execute `model`
    trace, _ = generate(model, gargs)
    # visualize the height for the ball across time
    plt = draw_trace(trace)
    # visualize unique tracetories for different
    # resitution values
    for i = 2:10
        trace, _ = generate(model, gargs)
        plt = update_plot(plt, trace, i)
    end
    display(plt);
    return nothing
end


main();
