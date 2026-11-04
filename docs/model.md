# Bounded oscillator model

The native evaluator integrates a one-dimensional mass–spring–damper.
The state is position in meters and velocity in meters per second.
Acceleration is force minus spring and damper forces, divided by mass.
Time is in seconds and input force is in newtons.

Held control takes one force per segment. Switch times increase strictly
and lie inside the horizon. The new control applies after the solver
lands on the boundary. Sampled PD computes a saturated force from
position error and velocity at each control sample and holds that force
until the next sample. Classical RK4 shortens steps at control and
terminal boundaries. State, accumulators and controller state reset
for each rollout. The declared envelope limits the step relative to
spring and damping scales. The position constraint is checked at
numerical step boundaries and refined-time replay is part of validation.

The objective combines squared terminal position error, 0.1 times
terminal velocity squared, mean integrated squared position error and
0.001 times integrated squared force. The scales are synthetic study
weights. A SciPy matrix exponential supplies an independent oracle in
under-, critically and overdamped cases. Scenario studies vary mass
and damping with seeded bounded sampling. The example includes held-out
evaluation and a small archive binned by error and effort.

The packaged release covers oscillator control. Headless rigid-body
and pushing require an installable engine and a matched source task.
