using Gen
using PyCall
using PhySMC
using PhyBullet
using Accessors
using Plots

include(joinpath(@__DIR__, "helpers.jl"))

function main()

    # start with a ball above a table
    client, obj_ramp_id, obj_table_id = ramp()

    # configure simulator with the provided
    # client id
    sim = BulletSim(;client=client)
    # These are the objects of interest in the scene
    # (the rest is static)
    obj_ramp = RigidBody(obj_ramp_id)
    obj_table = RigidBody(obj_table_id)
    # Retrieve the default latents for the objects
    # as well as their initial positions
    # Note: alternative latents will be suggested by the `prior`
    init_state = BulletState(sim, [obj_ramp, obj_table])
    # arguments for `model`
    gargs = (100, # number of steps (1s)
             sim,
             init_state)

    # execute `model`
    trace, _ = generate(model, gargs)

    # visualize the x  position of the objects across time
    generations = [generate(model, gargs) for i in 1:10]
    plts = plot([plot_trace(trace) for (trace, _) in generations]...)
    display(plts);

    println("press enter to exit the program")
    readline()
    return nothing
end


main();
