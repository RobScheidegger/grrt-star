export OMP_NUM_THREADS=4
mpirun -n 4 --allow-run-as-root ./grrt_solver -m 1 $@