for i in {120..0..-1}
  do
    rostopic pub -1 /roboy/middleware/MotorCommand roboy_middleware_msgs/MotorCommand "{id: 6, motors: [2], set_points: [$i]}" &
    sleep 0.1
  done
