from simple_launch import SimpleLauncher

sl = SimpleLauncher()


def launch_setup():

    with sl.group(ns = 'world'):
        sl.robot_state_publisher('ecn_2023', 'world.xacro', name = 'description',
                                 remappings = {'robot_description': 'description'})

    # UAV part
    for i in (1,2,3,4):
        ns = f'uav{i}'
        with sl.group(ns = ns):
            sl.robot_state_publisher('ecn_2023', 'drone.urdf.xacro', xacro_args={'id': i})
            sl.node('ecn_2023', 'uav_sim.py')

    sl.node('ecn_2023', 'uav_target')

    sl.rviz(sl.find('ecn_2023', 'world.rviz'))

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
