using Gen
using PyCall
using PhySMC
using PhyBullet
using Accessors

################################################################################
# Scene
################################################################################

function ramp(
    slope::Float64=2/3,
    tableRampIntersection::Float64=0.,
    )
    # for debugging
    #client = @pycall pb.connect(pb.GUI)::Int64
    #pb.resetDebugVisualizerCamera(4.5, 0, -40, [0.0, 0.0, 0.0]; physicsClientId=client)
    client = @pycall pb.connect(pb.DIRECT)::Int64
    pb.setGravity(0,0,-10; physicsClientId = client)
    

    # add a table base
    grey = [0.5, 0.5, 0.5, 1]
    base_dims = [5, 1, 0.75] # in meters
    table_dims = [base_dims[1] + 0.2, base_dims[2] + 0.2, 0.1]  # Width, depth, height
    table_base_col_id = pb.createCollisionShape(pb.GEOM_BOX, halfExtents = base_dims / 2, physicsClientId = client)
    table_base_obj_id = pb.createMultiBody(baseCollisionShapeIndex = table_base_col_id, basePosition = [0,0,-(base_dims[3]+table_dims[3])/2], physicsClientId = client)
    pb.changeDynamics(table_base_obj_id, -1; mass = 0., restitution = 0.9, physicsClientId=client)
    pb.changeVisualShape(table_base_obj_id, -1, rgbaColor=grey, physicsClientId=client)

    # Create the tabletop (a flat box)
    table_col_id = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=table_dims/2)
    table_body_id = pb.createMultiBody(baseCollisionShapeIndex=table_col_id, basePosition=[0, 0, -table_dims[3]/2])
    pb.changeDynamics(table_body_id, -1; mass = 0., restitution = 0.9, physicsClientId=client)
    pb.changeVisualShape(table_body_id, -1, rgbaColor=grey.+0.2, physicsClientId=client)

    # Create the four frame-like boxes around the tabletop
    frame_height = 0.25
    frame_thickness = 0.05

    frame_dims = [
        [table_dims[1] + 2 * frame_thickness, frame_thickness, frame_height],  # Longer sides
        [table_dims[1] + 2 * frame_thickness, frame_thickness, frame_height],  # Longer sides
        [frame_thickness, table_dims[2], frame_height],  # Shorter sides
        [frame_thickness, table_dims[2], frame_height]  # Shorter sides
    ]

    frame_positions = [
        [0, table_dims[2] / 2 + frame_thickness / 2, 0],  # Top side
        [0, -table_dims[2] / 2 - frame_thickness / 2, 0],  # Bottom side
        [table_dims[1] / 2 + frame_thickness / 2, 0, 0],  # Right side
        [-table_dims[1] / 2 - frame_thickness / 2, 0, 0]  # Left side
    ]

    for (dims, pos) in zip(frame_dims, frame_positions)
        frame_col_id = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=dims/2)::Int64
        frame_obj_id = pb.createMultiBody(baseCollisionShapeIndex=frame_col_id, basePosition=pos)::Int64
        pb.changeVisualShape(frame_obj_id, -1, rgbaColor=grey, physicsClientId=client)
    end

    # add a ramp
    ramp_col_id = pb.createCollisionShape(pb.GEOM_MESH, fileName="examples/ramp/ramp.obj", physicsClientId=client, meshScale=[2, base_dims[2], slope*2])
    ramp_position = [-2+tableRampIntersection, -base_dims[2]/2, 0]
    ramp_obj_id = pb.createMultiBody(baseCollisionShapeIndex=ramp_col_id, basePosition=ramp_position, physicsClientId=client)
    pb.changeDynamics(ramp_obj_id, -1; mass=0.0, restitution=0.9, physicsClientId=client)
    pb.changeVisualShape(ramp_obj_id, -1, rgbaColor=[1, 1, 1, 1], physicsClientId=client)

    # add a floor
    floor_col_id = pb.createCollisionShape(pb.GEOM_PLANE, physicsClientId=client)
    floor_obj_id = pb.createMultiBody(baseCollisionShapeIndex=floor_col_id, basePosition=[0,0,-base_dims[3]], physicsClientId=client)
    pb.changeDynamics(floor_obj_id, -1; mass=0.0, restitution=0.9, physicsClientId=client)


    #  add walls
    wall_dims = [[0.1, 8.0, 5.0], [0.1, 8.0, 5.0], [8.0, 0.1, 5.0]] # Width, length, height
    wall_positions = [
        [4.0, 0.0, 1.0],  # Right Wall
        [-4.0, 0.0, 1.0],  # Left Wall
        [0, 4, wall_dims[3][3]/2-base_dims[3]] # Back Wall
    ]
    for (dims, pos) in zip(wall_dims, wall_positions)
        wall_col_id = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=dims./2, physicsClientId=client)
        wall_obj_id = pb.createMultiBody(baseCollisionShapeIndex=wall_col_id, basePosition=pos, physicsClientId=client)
        pb.changeDynamics(wall_obj_id, -1; mass=0.0, restitution=0.9, physicsClientId=client)
        pb.changeVisualShape(wall_obj_id, -1, rgbaColor=grey+[0.2, 0.2, 0.2, 0], physicsClientId=client)
    end

    # add an object on the ramp
    obj_ramp_dims = [0.15, 0.3, 0.075]
    theta_radians = -atan(slope)
    orientation = [cos(theta_radians / 2), 0, sin(theta_radians / 2), 0]

    obj_on_ramp_col_id = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=obj_ramp_dims/2, physicsClientId=client)
    lift = obj_ramp_dims[3]/2
    position = [
        -1+tableRampIntersection+lift*cos(theta_radians),
        0,
        1*slope-lift*sin(theta_radians)
    ]
    obj_on_ramp_obj_id = pb.createMultiBody(baseCollisionShapeIndex=obj_on_ramp_col_id, basePosition=position, baseOrientation=orientation, physicsClientId=client)
    pb.changeDynamics(obj_on_ramp_obj_id, -1; mass=1.0, restitution=0.9, physicsClientId=client)
    
    # add an object on the table that will collide with the object on the ramp as that one slides down
    obj_on_table_dims = [0.2, 0.2, 0.1]
    obj_on_table_col_id = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=obj_on_table_dims/2, physicsClientId=client)
    obj_on_table_obj_id = pb.createMultiBody(baseCollisionShapeIndex=obj_on_table_col_id, basePosition=[1, 0, obj_on_table_dims[3]/2], physicsClientId=client)
    pb.changeDynamics(obj_on_table_obj_id, -1; mass=1.0, restitution=0.9, physicsClientId=client)

    (client, obj_on_ramp_obj_id, obj_on_table_obj_id)
end


"""
    data_generating_procedure(t::Int64)

Create a trial (ground truth and observations) with `t` timepoints
"""
function data_generating_procedure(t::Int64)

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
    gargs = (t, # number of steps
             sim,
             init_state)

    # execute `model`
    trace, _ = Gen.generate(model, gargs)
    choices = get_choices(trace)
    # extract noisy positions
    obs = Gen.choicemap()
    for i = 1:t
        addr = :kernel => i => :observe
        _choices = Gen.get_submap(choices, addr)
        Gen.set_submap!(obs, addr, _choices)
    end
    
    return (gargs, obs, trace)

end

################################################################################
# Distributions
################################################################################

struct TruncNorm <: Gen.Distribution{Float64} end
const trunc_norm = TruncNorm()
function Gen.random(::TruncNorm, mu::U, noise::T, low::T, high::T) where {U<:Real,T<:Real}
    d = Distributions.Truncated(Distributions.Normal(mu, noise),
                                low, high)
    return Distributions.rand(d)
end;
function Gen.logpdf(::TruncNorm, x::Float64, mu::U, noise::T, low::T, high::T) where {U<:Real,T<:Real}
    d = Distributions.Truncated(Distributions.Normal(mu, noise),
                                low, high)
    return Distributions.logpdf(d, x)
end;

################################################################################
# Generative Model
################################################################################

@gen function prior(ls::RigidBodyLatents)
    mass= @trace(gamma(1.2, 10.), :mass)

    new_ls = setproperties(ls.data;
                           mass = mass)
    new_latents = RigidBodyLatents(new_ls)
    return new_latents
end

@gen function observe(k::RigidBodyState)
    pos = k.position # XYZ position
    # add noise to position
    obs = @trace(broadcasted_normal(pos, 0.01), :position)
    return obs
end

@gen function kernel(t::Int, prev_state::BulletState, sim::BulletSim)
    # use of PhySMC.step
    next_state::BulletState = PhySMC.step(sim, prev_state)
    # elem state could be a different type
    # here we have two `RigidBody` elements
    # so  `next_state.kinematics = [RigidBodyState, RigidBodyState]`
    obs = @trace(Gen.Map(observe)(next_state.kinematics), :observe)
    return next_state
end

@gen function model(t::Int, sim::BulletSim, template::BulletState)
    # sample new mass and restitution for objects
    latents = @trace(Gen.Map(prior)(template.latents), :prior)
    init_state = setproperties(template; latents = latents)
    # simulate `t` timesteps
    states = @trace(Gen.Unfold(kernel)(t, init_state, sim), :kernel)
    return states
end

@load_generated_functions

################################################################################
# Inference
################################################################################

# this proposal function implements a truncated random walk for the both mass priors
@gen function proposal(tr::Gen.Trace)
    # get previous values from `tr`
    choices = get_choices(tr)
    prev_mass_1 = choices[:prior => 1 => :mass]
    prev_mass_2 = choices[:prior => 2 => :mass]
    
    # sample new values conditioned on the old ones
    mass_1 = {:prior => 1 => :mass} ~ trunc_norm(prev_mass_1, .25, 0., Inf)
    mass_2 = {:prior => 2 => :mass} ~ trunc_norm(prev_mass_2, .25, 0., Inf)
    
    # the return of this function is not
    # neccessary but could be useful
    # for debugging.
    return (mass_1, mass_2)
end

################################################################################
# Visuals
################################################################################

function plot_trace(tr::Gen.Trace, title="Trajectory")
    (t, _, _) = get_args(tr)
    # get the prior choice for the two masses
    choices = get_choices(tr)
    masses = [round(choices[:prior => i => :mass], digits=2) for i in 1:2]

    # get the x positions
    states = get_retval(tr)
    xs = [map(st -> st.kinematics[i].position[1], states) for i = 1:2]

    # return plot
    plot(1:t, xs, title=title, labels=["ramp: $(masses[1])" "table: $(masses[2])"], xlabel="t", ylabel="x")
end

"""
plot_traces(truth::Gen.DynamicDSLTrace, traces::Vector{Gen.DynamicDSLTrace})

Display the observed and final simulated trajectory as well as distributions for latents and the score
"""
function plot_traces(truth::Gen.DynamicDSLTrace, traces::Vector{Gen.DynamicDSLTrace})
    t = length(truth[:kernel])
    
    observed_plt = plot_trace(truth, "True trajectory")
    simulated_plt = plot_trace(last(traces), "Last trace")

    num_traces = length(traces)
    mass_logs = [[t[:prior => i => :mass] for t in traces] for i in 1:2]
    scores = [get_score(t) for t in traces]

    scores_plt = plot(1:num_traces, scores, title="Scores", xlabel="trace number", ylabel="log score")
    mass_plts = [Plots.histogram(1:num_traces, mass_logs[i], title="Mass $(i == 1 ? "Ramp object" : "Table object")", legend=false) for i in 1:2]
    ratio_plt = Plots.histogram(1:num_traces, mass_logs[1]./mass_logs[2], title="mass ramp object / mass table object", legend=false)
    plot(observed_plt, simulated_plt, mass_plts..., scores_plt, ratio_plt,  size=(1200, 800))
end
