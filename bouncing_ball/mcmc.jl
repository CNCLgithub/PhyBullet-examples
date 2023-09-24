using Gen
using UnicodePlots
using Distributions
using Plots

include(joinpath(@__DIR__, "helpers.jl"))

# Truncated Distributions
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

# this proposal function implements a truncated random walk for mass and restitution
@gen function proposal(tr::Gen.Trace)
    # HINT: https://www.gen.dev/tutorials/iterative-inference/tutorial#mcmc-2
    #
    # get previous values from `tr`
    choices = get_choices(tr)
    prev_mass = choices[:prior => 1 => :mass]
    prev_res  = choices[:prior => 1 => :restitution]
    
    # sample new values conditioned on the old ones
    mass = {:prior => 1 => :mass} ~ trunc_norm(prev_mass, .1, 0., Inf)
    restitution = {:prior => 1 => :restitution} ~ trunc_norm(prev_res, .1, 0., 1.)
    
    # the return of this function is not
    # neccessary but could be useful
    # for debugging.
    return (mass, restitution)
end

"""
    inference_procedure

Performs Metropolis-Hastings MCMC.
"""
function inference_procedure(gm_args::Tuple,
                             obs::Gen.ChoiceMap,
                             steps::Int = 100)

    # start with an initial guess of physical latents
    # `ls` is the log score or how well this
    # initial guess explains the observations
    tr, ls = Gen.generate(model, gm_args, obs)

    println("Initial logscore: $(ls)")

    # count the number of accepted moves and track accepted proposals
    acceptance_count = 0
    traces = Vector{Gen.DynamicDSLTrace}(undef, steps) 

    for i = 1:steps
        if i % 10 == 0
            println("$(i) steps completed")
        end
        # apply the proposal funciton to generate a
        # new guess over the ball's latents
        # that is related to the previous trace
        # see `?mh` in the REPL for more info
        tr, accepted = mh(tr, proposal, ())
        traces[i] = tr
        acceptance_count += Int(accepted)
    end

    acceptance_ratio = acceptance_count / steps

    println("Final logscore: $(get_score(tr))")
    println("Acceptance ratio: $(acceptance_ratio)")

    return (traces, acceptance_ratio)
end

"""
    data_generating_procedure(t::Int64)

Create a trial (ground truth and observations) with `t` timepoints
"""
function data_generating_procedure(t::Int64)

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

"""
plot_traces(truth::Gen.DynamicDSLTrace, traces::Vector{Gen.DynamicDSLTrace})

Display the observed and final simulated trajectory as well as distributions for latents and the score
"""
function plot_traces(truth::Gen.DynamicDSLTrace, traces::Vector{Gen.DynamicDSLTrace})
    t = length(truth[:kernel])
    get_zs(trace) = [trace[:kernel => i => :observe => 1][3] for i in 1:t]
    trajectory_plt = plot(1:t, get_zs(truth), title="Height of ball", xlabel="t", ylabel="z", label="Observation")
    plot!(trajectory_plt, 1:t, get_zs(last(traces)), label="Last trace")

    steps = length(traces)
    mass_log = [t[:prior => 1 => :mass] for t in traces]
    res_log = [t[:prior => 1 => :restitution] for t in traces]
    scores = [get_score(t) for t in traces]

    scores_plt = Plots.plot(1:steps, scores, title="Log of scores", xlabel="step", ylabel="log score")
    mass_plt = Plots.histogram(1:steps, mass_log, title="Histogram of mass", legend=false)
    res_plt = Plots.histogram(1:steps, res_log, title="Histogram of restitution", legend=false)

    Plots.plot(trajectory_plt, scores_plt, mass_plt, res_plt)
end

function main()

    t = 60 # 1 second of observations
    (gargs, obs, truth) = data_generating_procedure(t)

    (traces, aratio) = inference_procedure(gargs, obs, 50)    

    display(plot_traces(truth, traces))

    println("press enter to exit the program")
    readline()
    return nothing
end


main();
