# grrt-star

A GPU parallelized implementation of the dRRT algorithm for path planning with multiple autonomous robots. Our method relies on multi-core and multi-node parallelism with OpenMP and MPI, respectively. We also employ CUDA acceleration techniques with custom CUDA kernels for computing voxel-voxel intersections.

## Executables

### `grrt_solver`

The `grrt_solver` is the main solver executable, which takes a configuration file and produces a solution. The syntax is as follows:

```
gRRT Solver
Usage: ./grrt_solver [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -c,--config TEXT            Configuration file path
  -m,--mpi BOOLEAN            Use MPI
  -t,--timeout UINT           Timeout in seconds
  -v,--voxel-type TEXT        Voxel type
```

### `grrt_player`

The `grrt_player` is meant to play back a solved system using `rviz2` in real-time (at least real-time with regards to the true robot motions, but the solution will be pre-computed, and no )

The player will __not__ attempt to make any use of parallelism, the GPU, or `MPI`, since it is just meant to play-back a previous solution to a problem.

The syntax for running the `grrt_player` executable is as follows:

```
gRRT Player
Usage: ./grrt_player [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -c,--config TEXT            Configuration file path
  -s,--solution TEXT          Solution file path
  -v,--voxel UINT             Index to visualize
```

## Development

The `Dockerfile` in the root directory defines the runtime and build environments that we target for our implementation, including ROS, `spdlog`, and other dependencies. A development terminal environment can be automatically built and entered with the `./scripts/dev.sh` command.