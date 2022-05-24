rosservice call /ariac/start_competition
sleep 2

rosrun gazebo_ros spawn_model -sdf -x 0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_sensor_red_ariac/model.sdf -reference_frame movable_tray_dark_wood_1::movable_tray_dark_wood_1::link -model assembly_sensor_red_10