"""
Starts the whole pipeline: one source node, then stages 2-6.

    ros2 launch perception_suite pipeline.launch.py
    ros2 launch perception_suite pipeline.launch.py source:=video video_path:=other.mp4
    ros2 launch perception_suite pipeline.launch.py source:=camera
    ros2 launch perception_suite pipeline.launch.py source:=camera camera_index:=0
    ros2 launch perception_suite pipeline.launch.py source:=bag bag_path:=/bags/bag_20261006_125333
    ros2 launch perception_suite pipeline.launch.py scene:=snow
    ros2 launch perception_suite pipeline.launch.py visualize:=false
    ros2 launch perception_suite pipeline.launch.py backend:=tensorrt
    ros2 launch perception_suite pipeline.launch.py model:=semantic backend:=tensorrt
    ros2 launch perception_suite pipeline.launch.py config:=demo_route.yaml \
        video_path:=demo_route.mp4

config:= is the input (a YAML file name in src/perception_suite/config),
model:= the segmentation model, backend:= how it runs and scene:= its prompts;
see perception_suite/settings.py. Run it from the workspace root, where the
relative video and weight paths point.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from perception_suite import settings

# Source name -> (executable, launch arguments it takes as parameters).
# A new input (camera, beamng) is one entry here plus its node.
SOURCES = {
    'video': ('1_video_node', ['video_path', 'loop', 'video_fps', 'width']),
    'camera': ('1_camera_node', ['camera_index', 'camera_fps', 'width',
                                 'capture_width', 'capture_height',
                                 'pixel_format', 'rotate_180',
                                 'warmup_frames']),
    'bag': ('1_bag_node', ['image_topic', 'width', 'source_width',
                           'source_height', 'decoder', 'queue_depth',
                           'decode_backend']),
}

# 1_video_node declares an int default for width, the other two declare strings.
STRING_WIDTH_SOURCES = ('camera', 'bag')


def _is_true(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _validated(context):
    """Everything that can reject an argument, run before any stage starts."""
    path = settings.resolve(LaunchConfiguration('config').perform(context))
    settings.model(path, LaunchConfiguration('model').perform(context))
    settings.scene_prompts(path, LaunchConfiguration('scene').perform(context))
    parameters = {'config': LaunchConfiguration('config'),
                  'model': LaunchConfiguration('model'),
                  'scene': LaunchConfiguration('scene'),
                  'backend': LaunchConfiguration('backend')}
    segmentation = Node(
        package='perception_suite', executable='3_segmentation_node', output='screen',
        parameters=[parameters])
    return [segmentation] + _source(context)


def _source(context):
    name = LaunchConfiguration('source').perform(context)
    if name not in SOURCES:
        raise ValueError(f"source must be one of {sorted(SOURCES)}, got '{name}'")
    executable, arguments = SOURCES[name]
    parameters = {arg: LaunchConfiguration(arg).perform(context) for arg in arguments}
    # perform() always yields a string, and the two source nodes disagree on the
    # type they declare: 1_video_node uses an integer default, 1_camera_node a
    # string one. rclpy rejects a mismatch with InvalidParameterTypeException,
    # so coerce to whatever this source expects rather than to one type for both.
    parameters['width'] = (str(parameters['width'])
                           if name in STRING_WIDTH_SOURCES
                           else int(parameters['width']))
    node = Node(package='perception_suite', executable=executable, name='source_node',
                parameters=[parameters], output='screen')
    # Frames sent while the segmenter is still loading its model are dropped,
    # so the source waits for the stages to come up.
    delay = float(LaunchConfiguration('source_delay').perform(context))
    actions = [TimerAction(period=delay, actions=[node])]

    # source:=bag decodes a topic rather than reading a file, so something has
    # to publish that topic. A bag is the usual answer and is what HATCI asked
    # for, but an empty bag_path is legitimate: the same node serves a live
    # camera node publishing the same topic. See item 3.8 in OPEN-ITEMS.md.
    if name == 'bag':
        path = LaunchConfiguration('bag_path').perform(context)
        if path:
            command = ['ros2', 'bag', 'play', path,
                       '--rate', LaunchConfiguration('bag_rate').perform(context),
                       '--topics']
            command += LaunchConfiguration('bag_topics').perform(context).split()
            if _is_true(LaunchConfiguration('bag_loop').perform(context)):
                command.append('--loop')
            # Started after the decoder is subscribed. An H.264 stream cannot
            # be joined mid-GOP, so missing the first keyframe costs every
            # frame up to the next one.
            actions.append(TimerAction(
                period=delay + 2.0,
                actions=[ExecuteProcess(cmd=command, output='screen')]))
    return actions


def generate_launch_description():
    """
    Build the pipeline's launch description.

    Six nodes by default, one of them the visualizer, which visualize:=false
    leaves out. An odometry node joins when a CAN source is given, and
    7_controller_node joins with control:=true. The source is chosen with
    source:=video|camera|bag.
    """
    config = {'config': LaunchConfiguration('config'), 'model': LaunchConfiguration('model')}

    def stage(executable, extra=None, **kwargs):
        parameters = [config]
        if extra:
            parameters.append(extra)
        return Node(package='perception_suite', executable=executable,
                    parameters=parameters, output='screen', **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='video',
                              description=f'Input node: {", ".join(sorted(SOURCES))}'),
        DeclareLaunchArgument('source_delay', default_value='10.0',
                              description='Seconds to wait before starting the source'),
        DeclareLaunchArgument('camera_index', default_value='0',
                              description='V4L2 index for source:=camera. The GMSL '
                                          'dashcam enumerates at /dev/video0 on our '
                                          'carrier; the other three channels exist '
                                          'but have nothing attached'),
        DeclareLaunchArgument('camera_fps', default_value='30.0',
                              description='capture rate for source:=camera'),
        DeclareLaunchArgument('capture_width', default_value='1920',
                              description='What the sensor is asked to deliver\n'
                                          'for source:=camera, before the\n'
                                          'downscale to width. 1920x1080 is what\n'
                                          'our GMSL dashcam offers; a different\n'
                                          'camera will need its own mode, and\n'
                                          'V4L2 silently substitutes the nearest\n'
                                          'supported one rather than failing'),
        DeclareLaunchArgument('capture_height', default_value='1080',
                              description='Sensor capture height for '
                                          'source:=camera; see capture_width'),
        DeclareLaunchArgument('pixel_format', default_value='NV16',
                              description='Sensor pixel format for source:=camera. '
                                          'NV16 is the only one that decodes '
                                          'correctly on this hardware: the device '
                                          'advertises UYVY first, and OpenCV\'s own '
                                          'V4L2 backend mis-decodes every format it '
                                          'offers. Do not change without looking at '
                                          'the output'),
        DeclareLaunchArgument('rotate_180', default_value='false',
                              description='Rotate camera frames 180 degrees. Off by '
                                          'default: a silent rotation would put the '
                                          'road at the top of the frame and make the '
                                          'ground projection confidently wrong rather '
                                          'than obviously broken. Our bench camera '
                                          'presents inverted and needs true'),
        DeclareLaunchArgument('warmup_frames', default_value='120',
                              description='Frames discarded after the stream starts. '
                                          'Torn rows fade over seconds, not frames: '
                                          'measured on the bench carrier, 0.9 per cent '
                                          'of pixels are still torn at frame 5 and the '
                                          'first wholly clean frame is 120, which is '
                                          '4 seconds at 30 FPS'),
        DeclareLaunchArgument('width', default_value='1280',
                              description='Width the source publishes. Nothing between\n'
                                          'the source and 2_preprocess_node reads the\n'
                                          'larger frame, so setting this to the working\n'
                                          'width removes a resample and 4x the bytes'),
        DeclareLaunchArgument('video_path', default_value='test_video.mp4',
                              description='Video file for source:=video'),
        DeclareLaunchArgument('bag_path', default_value='',
                              description='Bag to replay for source:=bag. Left\n'
                                          'empty, the node still runs and waits\n'
                                          'for whatever else publishes the topic.\n'
                                          'Note: SIGKILLing launch before it has\n'
                                          'shut down can leave the player running,\n'
                                          'and a second player on the same topic\n'
                                          'shows up as the frame counter jumping\n'
                                          'backwards. Let launch exit properly.'),
        DeclareLaunchArgument('bag_topics',
                              default_value='/image_processing/image_compressed '
                                            '/image_processing/camera_info',
                              description='Topics to replay. Images only by\n'
                                          'default; append /CAN to replay the\n'
                                          'vehicle data too (ads_messages/msg/CANMsg,\n'
                                          'built from src/ads_messages).'),
        DeclareLaunchArgument('bag_loop', default_value='false',
                              description='Replay the bag end to end repeatedly'),
        DeclareLaunchArgument('bag_rate', default_value='1.0',
                              description='Replay speed for source:=bag. LEAVE IT\n'
                                          'AT 1.0. Lowering it is tempting, because\n'
                                          'it silences the frame-gap warnings,\n'
                                          'but it\n'
                                          'breaks the odometry: live_odom_node\n'
                                          'integrates dead reckoning on wall-clock\n'
                                          'arrival time, and the BEV grid decays on\n'
                                          'wall-clock too, so at rate r the ego\n'
                                          'motion is scaled by 1/r and the grid is\n'
                                          'stretched by the same factor. At 0.5 the\n'
                                          'vehicle appears to travel twice as far\n'
                                          'as it did and curvature reads half what\n'
                                          'it is. Nothing warns you.\n'
                                          '1.0 costs nothing. We lose no frames.\n'
                                          'The gaps the source node reports are\n'
                                          'already in the recording: 43 absent and\n'
                                          '43 repeated counters, confirmed by\n'
                                          'reading the bag directly. See the\n'
                                          'docstring in 1_bag_node.py.'),
        DeclareLaunchArgument('image_topic',
                              default_value='/image_processing/image_compressed',
                              description='Compressed H.264 topic for source:=bag'),
        DeclareLaunchArgument('source_width', default_value='2880',
                              description='Native frame width for source:=bag,\n'
                                          'used only to keep the aspect ratio'),
        DeclareLaunchArgument('source_height', default_value='1860',
                              description='Native frame height for source:=bag'),
        DeclareLaunchArgument('decoder', default_value='nvv4l2decoder',
                              description='GStreamer H.264 decoder for\n'
                                          'source:=bag. avdec_h264 is the\n'
                                          'software fallback.'),
        DeclareLaunchArgument('queue_depth', default_value='200',
                              description='Subscription depth for source:=bag. A\n'
                                          'dropped message corrupts a P-frame\n'
                                          'stream, so this is deliberately deep.'),
        DeclareLaunchArgument('decode_backend', default_value='auto',
                              description='How source:=bag decodes H.264.\n'
                                          'appsink decodes in-process and is\n'
                                          'faster; subprocess shells out to\n'
                                          'gst-launch and needs no python3-gi.\n'
                                          'auto prefers appsink when available.'),
        DeclareLaunchArgument('loop', default_value='true',
                              description='Restart the video at the end instead of '
                                          'stopping, for source:=video; false plays it once'),
        DeclareLaunchArgument('video_fps', default_value='30.0',
                              description='Publish rate for source:=video; raise it '
                                          'to measure the pipeline ceiling'),
        DeclareLaunchArgument('config', default_value=settings.DEFAULT_CONFIG,
                              description='Pipeline config for the input: a YAML file '
                                          'name in src/perception_suite/config'),
        DeclareLaunchArgument('model', default_value=settings.DEFAULT_MODEL,
                              description='Segmentation model from perception.models in '
                                          'the config: yoloe or semantic'),
        DeclareLaunchArgument('scene', default_value=settings.DEFAULT_SCENE,
                              description='YOLOE prompt set from perception.scenes in the '
                                          'config (e.g. trail, snow, sandy); ignored by '
                                          'the semantic model'),
        DeclareLaunchArgument('backend', default_value='pytorch',
                              choices=['pytorch', 'tensorrt'],
                              description='Segmentation inference: pytorch, or tensorrt '
                                          '(NVIDIA GPU; builds an engine on first use)'),
        DeclareLaunchArgument('planner_mode', default_value='',
                              description='Override the planner: baseline, advanced '
                                          'or grid. Empty lets the config decide, '
                                          'except that a can_csv selects grid - so '
                                          'this is how to run the baseline planner '
                                          'WITH odometry, which is a real case: that '
                                          'planner ignores ego motion, so the '
                                          'odometry then feeds only the speed '
                                          'readout and the controller'),
        DeclareLaunchArgument('can_topic', default_value='',
                              description='ROS topic carrying live vehicle CAN, '
                                          'e.g. /CAN. Empty disables the node. '
                                          'Mutually exclusive with can_csv: the '
                                          'csv is the recorded bags, this is the '
                                          'vehicle. Type is ads_messages/msg/CANMsg, '
                                          'published by HATCI\'s CAN node'),
        DeclareLaunchArgument('can_csv', default_value='',
                              description='Extracted CAN csv to replay as odometry. '
                                          'Empty disables the node, and the planner '
                                          'then falls back to its static speed_mps'),
        DeclareLaunchArgument('speed_mps', default_value='0.0',
                              description='Fixed vehicle speed given to the planner'),
        DeclareLaunchArgument('frame_id', default_value='base_link',
                              description='frame_id on the published Path.\n'
                                          'Origin on the ground below the\n'
                                          'camera, x forward, y left (REP 103).\n'
                                          'Change it to whatever your TF tree\n'
                                          'calls that frame'),
        DeclareLaunchArgument('visualize', default_value='true',
                              description='Open the visualizer window'),
        DeclareLaunchArgument('control', default_value='false',
                              description='Run 7_controller_node, which turns the '
                                          'planner output into a steering and speed '
                                          'command on /command. Off by default: the '
                                          'camera and bag tests do not need it, and a '
                                          'pipeline that starts commanding a vehicle '
                                          'because someone launched it should be an '
                                          'explicit choice'),
        DeclareLaunchArgument('odom_timeout_s', default_value='0.5',
                              description='How old odometry may be before the\n'
                                          'controller commands a stop, the\n'
                                          'planner drops the pose so the BEV\n'
                                          'grid forgets rather than smearing,\n'
                                          'and the dashboard zeroes its dials.\n'
                                          'Only applies once odometry has been\n'
                                          'seen: a pipeline with no CAN source\n'
                                          'at all runs on fixed speed_mps'),
        DeclareLaunchArgument('command_timeout_s', default_value='0.5',
                              description='How long the dashboard keeps showing\n'
                                          'the last control command before the\n'
                                          'dials fall back to zero. A frozen dial\n'
                                          'reads as live, so a stopped controller\n'
                                          'has to look like no controller'),
        # First, so a bad config, model, scene or source fails before any
        # stage node starts.
        OpaqueFunction(function=_validated),
        stage('2_preprocess_node'),
        Node(package='perception_suite', executable='odom_node', output='screen',
             parameters=[{'can_csv': LaunchConfiguration('can_csv'),
                          'loop': LaunchConfiguration('loop')}],
             condition=IfCondition(
                 PythonExpression(["'", LaunchConfiguration('can_csv'), "' != ''"]))),
        Node(package='perception_suite', executable='live_odom_node', output='screen',
             parameters=[{'can_topic': LaunchConfiguration('can_topic')}],
             condition=IfCondition(
                 PythonExpression(["'", LaunchConfiguration('can_topic'), "' != ''"]))),
        stage('4_postprocess_node'),
        stage('5_planner_node', {
            'speed_mps': LaunchConfiguration('speed_mps'),
            'frame_id': LaunchConfiguration('frame_id'),
            # A CAN csv is only useful to the grid planner, so supplying one
            # selects it; the baseline planner ignores ego motion entirely.
            # An explicit planner_mode wins; otherwise a CAN csv selects grid.
            'odom_timeout_s': LaunchConfiguration('odom_timeout_s'),
            'planner_mode': PythonExpression(
                ["'", LaunchConfiguration('planner_mode'),
                 "' or ('grid' if '", LaunchConfiguration('can_csv'),
                 "' or '", LaunchConfiguration('can_topic'),
                 "' else '')"])}),
        stage('6_visualizer_node', {
            'command_timeout_s': LaunchConfiguration('command_timeout_s'),
            'odom_timeout_s': LaunchConfiguration('odom_timeout_s')},
            condition=IfCondition(LaunchConfiguration('visualize'))),
        stage('7_controller_node', {
            'speed_mps': LaunchConfiguration('speed_mps'),
            'odom_timeout_s': LaunchConfiguration('odom_timeout_s')},
            condition=IfCondition(LaunchConfiguration('control'))),
    ])
