ros2 run rviz2 rviz2 -d scripts/default.rviz &
./grrt_player -c configs/circle_10.yml -s configs/circle_10.yml.sol $@ 

# Kill all background jobs
kill $(jobs -p)
