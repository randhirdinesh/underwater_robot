import launch 
import launch_ros_actions

def generate_launch_description():
    return Launch.Description([
        launch_ros.actions.Node(
            package= 'rl_ha_rrt',
            executable= 'rl_ha_rrt_planner',
            name= 'rl_ha_rrt_planner',
            output = 'screen',
        ),
        ])