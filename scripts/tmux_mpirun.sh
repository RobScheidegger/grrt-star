export TMUX_MPI_MPIRUN="mpiexec --allow-run-as-root"
export TMUX_MPI_MODE=pane
export TMUX_MPI_SYNC_PANES=1

tmux-mpi 2 gdb grrt_solver &