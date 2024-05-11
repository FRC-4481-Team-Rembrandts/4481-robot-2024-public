# 2024 Public Robot Code

Full code used at Worlds for our 2024 robot Coda.

## Key features
- Automatic driving to pose for amping.
- Automatic rotation to speaker and aiming of shamper with look-uptable.
- Heading correction, direction slew rate limiter and exponential scaling for teleop driving
- Adaptive time sampling of path to get the next target state on the trajectory robust against collisions and large deviations.
- Recovery of missed notes in autonomous.
- Photon Vision localization with custom post-processing filter.
