using Gen
using PyCall
using PhySMC
using PhyBullet
using Accessors

################################################################################
# Scene
################################################################################

function simple_scene(mass::Float64=1.0,
                      restitution::Float64=0.9)
    client = @pycall pb.connect(pb.DIRECT)::Int64
    pb.setGravity(0,0,-10; physicsClientId = client)

    # add a table
    dims = [1.0, 1.0, 0.1] # in meters
    col_id = pb.createCollisionShape(pb.GEOM_BOX,
                                     halfExtents = dims,
                                     physicsClientId = client)
    obj_id = pb.createMultiBody(baseCollisionShapeIndex = col_id,
                                basePosition = [0., 0., -0.1],
                                physicsClientId = client)
    pb.changeDynamics(obj_id,
                      -1;
                      mass = 0., # 0 mass are stationary
                      restitution = 0.9, # some is necessary
                      physicsClientId=client)


    # add a ball
    bcol_id = pb.createCollisionShape(pb.GEOM_SPHERE,
                                      radius = 0.1,
                                      physicsClientId = client)
    bobj_id = pb.createMultiBody(baseCollisionShapeIndex = bcol_id,
                                 basePosition = [0., 0., 1.0],
                                 physicsClientId = client)
    pb.changeDynamics(bobj_id,
                      -1;
                      mass = mass,
                      restitution = restitution,
                      physicsClientId=client)

    (client, bobj_id)
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
    mass = @trace(gamma(1.2, 10.), :mass)
    res = @trace(uniform(0, 1), :restitution)
    new_ls = setproperties(ls.data;
                           mass = mass,
                           restitution = res)
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
    # but here we only have one `RigidBody` element
    # so  `next_state.kinematics = [RigidBodyState]`
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
# Visuals
################################################################################

get_zs(trace, t) = [trace[:kernel => i => :observe => 1][3] for i in 1:t]

function plot_zs(trace::Gen.DynamicDSLTrace)
    t = length(trace[:kernel])
    
    return plot(1:t, get_zs(trace, t), title="Height of ball", xlabel="t", ylabel="z", label="Observation")
end

"""
plot_traces(truth::Gen.DynamicDSLTrace, traces::Vector{Gen.DynamicDSLTrace})

Display the observed and final simulated trajectory as well as distributions for latents and the score
"""
function plot_traces(truth::Gen.DynamicDSLTrace, traces::Vector{Gen.DynamicDSLTrace})
    t = length(truth[:kernel])
    trajectory_plt = plot_zs(truth)
    plot!(trajectory_plt, 1:t, get_zs(last(traces), t), label="Last trace")

    steps = length(traces)
    mass_log = [t[:prior => 1 => :mass] for t in traces]
    res_log = [t[:prior => 1 => :restitution] for t in traces]
    scores = [get_score(t) for t in traces]

    scores_plt = Plots.plot(1:steps, scores, title="Log of scores", xlabel="step", ylabel="log score")
    mass_plt = Plots.histogram(1:steps, mass_log, title="Histogram of mass", legend=false)
    res_plt = Plots.histogram(1:steps, res_log, title="Histogram of restitution", legend=false)

    Plots.plot(trajectory_plt, scores_plt, mass_plt, res_plt)
end
