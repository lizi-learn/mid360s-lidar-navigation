"""Keep command/observation processes at normal priority during planning.

Only CPU-heavy planning/localization children lower their own priority. There
is no realtime scheduling or privilege requirement and no motor process here.
"""

def background_prefix(physical_gait):
    # ROS launch treats a list as concatenated substitutions, not argv.
    return 'nice -n 10' if physical_gait else ''


def workflow_command(root, mode, simulation, params, map_file=None, physical_gait=False):
    command = ['bash', str(root / 'run.sh'), 'mode:=' + mode,
               'simulation:=' + str(simulation).lower(), 'params:=' + str(params)]
    if mode == 'navigation':
        command.append('map:=' + str(map_file))
    command.append('physical_gait:=' + str(physical_gait).lower())
    return command
