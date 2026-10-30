"""Install live_odom_node into the workspace copy and wire it into the launch.

Runs AFTER bev_overlay.py, whose anchors it depends on: bev_overlay is what
introduces can_csv, planner_mode and the odom_node block in the first place.

The wiring mirrors bev_overlay's exactly, one interface wider:

    can_csv:=<path>        replay recorded odometry   (lab, unchanged)
    can_topic:=/CAN        subscribe to the vehicle's CAN topic (vehicle, new)

Either one selects the BEV grid planner unless planner_mode says otherwise, so
the vehicle command is the same shape as the one in the report. Both default to
empty, so a pipeline launched with neither behaves precisely as it did before
this file existed.

Idempotent: every edit is guarded, so applying it twice is a no-op.
"""
import ast
import sys
from pathlib import Path

PKG = Path(sys.argv[sys.argv.index("--pkg") + 1]) / "perception_suite"
OVERLAYS = Path("/bench/overlays")

(PKG / "live_odom_node.py").write_text((OVERLAYS / "live_odom_node.py").read_text())
print("  copied live_odom_node.py")

L = PKG.parent / "launch" / "pipeline.launch.py"
lt = L.read_text()

if "live_odom_node" in lt:
    print("  pipeline.launch.py already wired")
else:
    # 1. The launch argument, declared beside the csv one it parallels.
    anchor = "        DeclareLaunchArgument('can_csv', default_value='',"
    assert lt.count(anchor) == 1, f"{lt.count(anchor)} matches for the can_csv argument"
    lt = lt.replace(anchor, """        DeclareLaunchArgument('can_topic', default_value='',
                              description='socketcan interface to read live ego '
                                          'odometry from, e.g. can0. Empty '
                                          'disables the node. Mutually exclusive '
                                          'with can_csv: the csv is the recorded '
                                          'bags, this is the vehicle'),
""" + anchor, 1)

    # 2. The node itself, gated the same way odom_node is.
    anchor = ("                 PythonExpression([\"'\", LaunchConfiguration('can_csv'), "
              "\"' != ''\"]))),")
    assert lt.count(anchor) == 1, f"{lt.count(anchor)} matches for the odom_node condition"
    lt = lt.replace(anchor, anchor + """
        Node(package='perception_suite', executable='live_odom_node', output='screen',
             parameters=[{'can_topic': LaunchConfiguration('can_topic')}],
             condition=IfCondition(
                 PythonExpression(["'", LaunchConfiguration('can_topic'), "' != ''"]))),""", 1)

    # 3. A live bus selects the grid planner, exactly as a csv does.
    anchor = ("                 \"' or ('grid' if '\", LaunchConfiguration('can_csv'), "
              "\"' else '')\"])}),")
    assert lt.count(anchor) == 1, f"{lt.count(anchor)} matches for the planner_mode expression"
    lt = lt.replace(anchor,
                    "                 \"' or ('grid' if '\", LaunchConfiguration('can_csv'),\n"
                    "                 \"' or '\", LaunchConfiguration('can_topic'),\n"
                    "                 \"' else '')\"])}),", 1)

    L.write_text(lt)
    ast.parse(lt)
    print("  pipeline.launch.py: live_odom_node wired, gated on can_topic")

setup = PKG.parent / "setup.py"
s = setup.read_text()
if "live_odom_node" in s:
    print("  setup.py already has the entry point")
else:
    i = s.find("'console_scripts': [") + len("'console_scripts': [")
    assert i > len("'console_scripts': ["), "console_scripts not found in setup.py"
    s = s[:i] + "\n            'live_odom_node = perception_suite.live_odom_node:main'," + s[i:]
    setup.write_text(s)
    print("  setup.py: live_odom_node entry point registered")
