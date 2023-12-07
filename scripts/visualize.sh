ros2 run rviz2 rviz2 -d scripts/default.rviz &
./grrt_player -c configs/$1.yml -s configs/$1.yml.sol


# Kill all background jobs
kill $(jobs -p)
