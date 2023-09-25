# PhyBullet - Examples

This repository contains examples for the use of the [PhyBullet](https://github.com/CNCLgithub/PhyBullet) backend for [PhySMC](https://github.com/CNCLgithub/PhySMC).

## Project structure
There are currently two examples in their respective subfolder:
- bouncing ball: a ball is dropped onto a table
- ramp: an object slides down a ramp on a table and collides with another object on the table

Both examples include implementation of MCMC and particle filter inference procedures.

## Setup

First, follow the installation guidelines for [PhyBullet](https://github.com/CNCLgithub/PhyBullet).

To run the examples of this repo, activate the environment of this repository and instantiate the dependencies:
```
using Pkg
Pkg.activate("path/to/project")
Pkg.instantiate()
```

Then, use the environment specified by the package in this repository when you execute a script, e.g.:

```
julia --project=. ramp/particle_filter.jl
```

