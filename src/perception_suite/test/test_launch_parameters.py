"""
Every parameter a node declares must be reachable from the launch file.

This test exists because the same mistake happened twice. 1_camera_node grew
capture_width and capture_height, and 1_bag_node grew decode_backend, and in
both cases the parameter was declared on the node but never listed in the
launch file. The failure mode is the reason it is worth a test: `ros2 launch
... decode_backend:=subprocess` is ACCEPTED, silently, and does nothing. The
parameter stays at its default, there is no error, no warning and nothing in
any log. You conclude the knob does not help, when in fact it was never set.

The check is static. It reads the AST of the node modules and of the launch
file and compares sets of names, so it needs no ROS installation and runs on a
laptop as well as on the board:

    python3 test/test_launch_parameters.py     # standalone, prints a report
    pytest test/test_launch_parameters.py      # under colcon test

What it catches: a parameter declared on a node that no `ros2 launch
--show-args` would ever list, and a name in SOURCES that is not a declared
launch argument (which raises at launch time) or not a declared node parameter
(which rclpy rejects as an undeclared override).

What it does NOT catch: a parameter that is wired, but to the wrong node. That
needs the launch file actually evaluated, which needs ROS.

To add a parameter: declare it on the node, add a DeclareLaunchArgument for
it, and for a source node add its name to that source's list in SOURCES. To
deliberately keep one off the command line, add it to NOT_EXPOSED with a
reason.
"""

import ast
import pathlib
import sys

PKG = pathlib.Path(__file__).resolve().parent.parent
NODES = PKG / 'perception_suite'
LAUNCH = PKG / 'launch' / 'pipeline.launch.py'

# Modules under perception_suite/ that are libraries, not nodes.
LIBRARIES = {'__init__', 'common', 'engine', 'settings'}

# load_pipeline_config() declares these on behalf of whatever node calls it.
INHERITED = {'config', 'model'}

# Parameters deliberately NOT on the command line, each with its reason.
# Anything unreachable and not listed here fails the test.
NOT_EXPOSED = {
    ('6_visualizer_node', 'window_x'):
        'where the dashboard opens on our bench monitor; of no use to anyone '
        'running this on another board',
    ('6_visualizer_node', 'window_y'):
        'see window_x',
}

# Unreachable, SHOULD be exposed, and not yet, because exposing it is more
# than a launch argument. Both of these declare a non-string default, and a
# LaunchConfiguration always substitutes a string, so wiring them as they
# stand raises InvalidParameterTypeException. The node has to declare a string
# default and coerce, the way 1_camera_node and 1_bag_node already do. For
# trust_flags that also means parsing the bool by hand, because bool('false')
# is True.
DEFERRED = {
    ('live_odom_node', 'stale_timeout_s'): 'declares float 0.5; needs a string default',
    ('live_odom_node', 'trust_flags'): 'declares bool True; needs a string default and a parser',
}


def _literal_string(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def declared_parameters(path):
    """Parameter names a module declares, via node.declare_parameter('x', ...)."""
    names = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and node.args):
            continue
        name = _literal_string(node.args[0])
        if name is None:
            raise AssertionError(
                f'{path.name}: declare_parameter() with a non-literal name. '
                'This test cannot see it, so it cannot check it.')
        names.add(name)
    if 'load_pipeline_config' in path.read_text():
        names |= INHERITED
    return names


def node_modules():
    return sorted(p for p in NODES.glob('*.py') if p.stem not in LIBRARIES)


def launch_arguments(tree):
    """Names a `ros2 launch --show-args` would list."""
    return {_literal_string(node.args[0])
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'DeclareLaunchArgument'
            and node.args}


def sources_table(tree):
    """Return the SOURCES literal: source name -> (executable, [arguments])."""
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'SOURCES'
                        for t in node.targets)):
            continue
        if not isinstance(node.value, ast.Dict):
            raise AssertionError('SOURCES is no longer a dict literal; this '
                                 'test reads it statically')
        table = {}
        for key, value in zip(node.value.keys, node.value.values):
            executable = _literal_string(value.elts[0])
            arguments = [_literal_string(a) for a in value.elts[1].elts]
            table[_literal_string(key)] = (executable, arguments)
        return table
    raise AssertionError('launch file has no SOURCES assignment')


def launch_defaults(tree):
    """Argument name -> literal default, for the ones that have one."""
    out = {}
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == 'DeclareLaunchArgument'
                and node.args):
            continue
        for keyword in node.keywords:
            if keyword.arg == 'default_value':
                value = _literal_string(keyword.value)
                if value is not None:
                    out[_literal_string(node.args[0])] = value
    return out


def module_constant(path, name):
    """Return a module-level NAME = 'literal' assignment."""
    for node in ast.walk(ast.parse(path.read_text())):
        if (isinstance(node, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == name
                        for t in node.targets)):
            return _literal_string(node.value)
    return None


# Launch argument -> (module, constant) whose values must agree, because the
# launch file duplicates the literal instead of importing the module.
MIRRORED_DEFAULTS = {
    'can_msg_type': ('live_odom_node', 'DEFAULT_CAN_TYPE'),
}


def findings():
    """Every problem, as (kind, node, parameter) tuples. Empty means clean."""
    tree = ast.parse(LAUNCH.read_text())
    arguments = launch_arguments(tree)
    sources = sources_table(tree)
    out = []

    for path in node_modules():
        for name in sorted(declared_parameters(path) - arguments):
            if (path.stem, name) in NOT_EXPOSED:
                continue
            if (path.stem, name) in DEFERRED:
                continue
            out.append(('unreachable', path.stem, name))

    for source, (executable, arguments_for_source) in sorted(sources.items()):
        declared = declared_parameters(NODES / f'{executable}.py')
        for name in sorted(set(arguments_for_source) - arguments):
            out.append(('not-a-launch-argument', executable, name))
        for name in sorted(set(arguments_for_source) - declared):
            out.append(('not-declared-by-node', executable, name))
        for name in sorted(declared - set(arguments_for_source) - INHERITED):
            if (executable, name) in NOT_EXPOSED or (executable, name) in DEFERRED:
                continue
            out.append(('missing-from-SOURCES', executable, name))

    defaults = launch_defaults(tree)
    for argument, (module, constant) in sorted(MIRRORED_DEFAULTS.items()):
        theirs = module_constant(NODES / f'{module}.py', constant)
        if defaults.get(argument) != theirs:
            out.append(('default-drift', module,
                        f'{argument}={defaults.get(argument)!r} but '
                        f'{constant}={theirs!r}'))

    # A stale entry in either exemption list is itself a finding: it means the
    # parameter was wired up or removed and the note was left behind.
    for table, kind in ((NOT_EXPOSED, 'stale-NOT_EXPOSED'),
                        (DEFERRED, 'stale-DEFERRED')):
        for (module, name) in sorted(table):
            path = NODES / f'{module}.py'
            if not path.exists() or name not in declared_parameters(path):
                out.append((kind, module, f'{name} (no longer declared)'))
            elif name in arguments:
                out.append((kind, module, f'{name} (now a launch argument)'))
    return out


def test_every_declared_parameter_is_reachable():
    found = findings()
    assert not found, 'launch wiring:\n' + '\n'.join(
        f'  {kind:22s} {module} :: {name}' for kind, module, name in found)


def main():
    tree = ast.parse(LAUNCH.read_text())
    arguments = launch_arguments(tree)
    print(f'{len(arguments)} launch arguments, {len(node_modules())} nodes\n')
    for path in node_modules():
        declared = declared_parameters(path)
        unreachable = sorted(declared - arguments)
        note = ''
        if unreachable:
            note = '  not on the command line: ' + ', '.join(unreachable)
        print(f'  {path.stem:22s} {len(declared):2d} parameters{note}')
    found = findings()
    print()
    if not found:
        print('PASS: every declared parameter is reachable or exempted')
        return 0
    print(f'FAIL: {len(found)} finding(s)')
    for kind, module, name in found:
        print(f'  {kind:22s} {module} :: {name}')
    return 1


if __name__ == '__main__':
    sys.exit(main())
