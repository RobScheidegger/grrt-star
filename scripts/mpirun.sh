export OMP_NUM_THREADS=1
mpirun -n 4 --allow-run-as-root ./grrt_solver -m 1 $@