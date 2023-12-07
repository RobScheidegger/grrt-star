ros2 run rviz2 rviz2 -d scripts/default.rviz &
./grrt_player -c configs/test/test_two_drone_no_conflict.yml -s configs/test/solution_two_drone_no_conflict.txt $@


# Kill all background jobs
kill $(jobs -p)
