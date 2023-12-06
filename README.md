# grrt-star

A GPU parallelized implementation of the Fast-dRRT* algorithm for path planning with multiple autonomous robots.

## Executables

### `grrt_solver`

### `grrt_player`

The `grrt_player` is meant to play back a solved system using `rviz2` in real-time (at least real-time with regards to the true robot motions, but the solution will be pre-computed, and no )

The player will __not__ attempt to make any use of parallelism, the GPU, or `MPI`, since it is just meant to play-back a previous solution to a problem.