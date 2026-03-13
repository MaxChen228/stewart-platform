# Stability And Known Issues

## Key Stability Finding

The "all cranks vertical-up" posture is:

- a unique solution in the current theoretical model
- but also a very poor operating point
- effectively a near-singular / badly conditioned region

Theoretical evidence observed during development:

- calibration pose condition number is extremely large
- operating home condition number is much smaller

Conclusion:

- use all-up only as encoder calibration reference
- do not use it as the normal working home

## Why The Machine Can Tip

The project has repeatedly shown that small actuator error near bad geometry can produce visible platform tilt.

Reasons:

1. near-singular geometry
2. closed-chain amplification of small mismatches
3. structural compliance
4. real hardware asymmetry

## Structural Compliance Hypothesis

The user observed:

- top plate bends
- base plate bends

This is highly relevant.

If top or base geometry flexes under load:

- IK assumptions are violated
- attachment points move under force
- equal actuator angles no longer imply the same platform pose

This can produce:

- drift
- hunting
- "return but not exactly return"
- tilt during nominal Z motion

## Current Practical Stability Rule

Normal operation should happen near the working home, not near calibration height.

Recommended sequence:

1. manually set all-up
2. calibrate
3. immediately return to home
4. operate only around home

## Residual Error After Motion

Observed behavior:

- after repeated moves, small angle offsets can remain even with encoder feedback

Why this can still happen:

- encoder measures motor axis, not platform pose
- structural flex and backlash still exist
- holding behavior near target may hunt
- one actuator arriving slightly differently changes the whole closed chain

So "has encoder" does not imply "platform pose is perfectly repeatable".

## Most Likely Remaining Root Causes

Current likely contributors, in rough priority order:

1. insufficient base rigidity
2. insufficient top-platform rigidity
3. bracket / crank support flex
4. linkage backlash / joint play
5. geometry parameter mismatch
6. holding current / control tuning

## Low-Cost Validation Ideas

To check whether rigidity dominates:

1. hold a pose with motors enabled and lightly press one platform corner
2. compare enabled vs disabled behavior under the same manual force
3. visually mark base/platform reference points and check whether plates bend under load

If geometry flex is the main issue, controller tuning alone will not fully solve it.
