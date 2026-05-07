"""generate a random maze and export it as a gazebo .sdf world.

usage:
    python -m maze_worlds.generate_maze --size 10 --cell 1.0 --out worlds/maze_10.sdf

algo: recursive backtracker (iterativ, mit stack).
maze ist ein grid von cells, jede cell hat 4 wände (N/E/S/W).
beim besuchen brechen wir die wand zur nächsten cell auf.
am ende exportieren wir alle übriggebliebenen wände als boxen ins .sdf.
"""
import argparse
import random
from pathlib import Path


def generate_grid(size, seed=None):
    """recursive backtracker. liefert dict {(x,y): {'N','E','S','W'} mit verbleibenden wänden}."""
    if seed is not None:
        random.seed(seed)

    cells = {(x, y): {'N', 'E', 'S', 'W'} for x in range(size) for y in range(size)}
    visited = set()
    stack = [(0, 0)]
    visited.add((0, 0))

    opp = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
    delta = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}

    while stack:
        x, y = stack[-1]
        nbrs = []
        for d, (dx, dy) in delta.items():
            nx, ny = x + dx, y + dy
            if 0 <= nx < size and 0 <= ny < size and (nx, ny) not in visited:
                nbrs.append((d, nx, ny))
        if not nbrs:
            stack.pop()
            continue
        d, nx, ny = random.choice(nbrs)
        cells[(x, y)].discard(d)
        cells[(nx, ny)].discard(opp[d])
        visited.add((nx, ny))
        stack.append((nx, ny))

    # entry / exit aufmachen
    cells[(0, 0)].discard('S')
    cells[(size - 1, size - 1)].discard('N')

    return cells


def cells_to_walls(cells, size, cell_size):
    """jede wand wird zu einem (x, y, length, orientation) tupel.
    duplikate (eine wand zwischen 2 cells) werden gemerged.
    """
    walls = set()
    for (x, y), w in cells.items():
        cx = x * cell_size
        cy = y * cell_size
        if 'N' in w:
            walls.add((cx, cy + cell_size, cell_size, 'h'))
        if 'S' in w:
            walls.add((cx, cy, cell_size, 'h'))
        if 'E' in w:
            walls.add((cx + cell_size, cy, cell_size, 'v'))
        if 'W' in w:
            walls.add((cx, cy, cell_size, 'v'))
    return walls


def bot_sdf(start_x, start_y, yaw=1.5708):
    """maze_bot als inline-sdf — wird direkt in die welt gepackt
    damit das spawn nicht über ros_gz_sim laufen muss."""
    return f"""
    <model name="maze_bot">
      <pose>{start_x} {start_y} 0.05 0 0 {yaw}</pose>

      <link name="base_link">
        <inertial>
          <mass>2.0</mass>
          <inertia><ixx>0.02</ixx><iyy>0.025</iyy><izz>0.03</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c"><geometry><box><size>0.30 0.25 0.10</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>0.30 0.25 0.10</size></box></geometry>
          <material>
            <ambient>0.05 0.2 0.5 1</ambient>
            <diffuse>0.1 0.4 0.9 1</diffuse>
            <specular>0.2 0.4 0.9 1</specular>
          </material>
        </visual>
      </link>

      <link name="left_wheel">
        <pose>0 0.15 -0.05 0 0 0</pose>
        <inertial>
          <mass>0.2</mass>
          <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">
          <pose>0 0 0 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>0.025</length></cylinder></geometry>
        </collision>
        <visual name="v">
          <pose>0 0 0 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>0.025</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>
        </visual>
      </link>

      <link name="right_wheel">
        <pose>0 -0.15 -0.05 0 0 0</pose>
        <inertial>
          <mass>0.2</mass>
          <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">
          <pose>0 0 0 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>0.025</length></cylinder></geometry>
        </collision>
        <visual name="v">
          <pose>0 0 0 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>0.025</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>
        </visual>
      </link>

      <link name="caster">
        <pose>0.11 0 -0.07 0 0 0</pose>
        <inertial>
          <mass>0.05</mass>
          <inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c"><geometry><sphere><radius>0.02</radius></sphere></geometry>
          <surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface>
        </collision>
        <visual name="v"><geometry><sphere><radius>0.02</radius></sphere></geometry></visual>
      </link>

      <link name="lidar_link">
        <pose>0 0 0.07 0 0 0</pose>
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <visual name="v">
          <geometry><cylinder><radius>0.04</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.6 0.05 0.05 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material>
        </visual>
        <sensor name="lidar" type="gpu_lidar">
          <update_rate>10</update_rate>
          <topic>scan</topic>
          <gz_frame_id>lidar_link</gz_frame_id>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
              <vertical><samples>1</samples><min_angle>0</min_angle><max_angle>0</max_angle></vertical>
            </scan>
            <range><min>0.08</min><max>8.0</max><resolution>0.01</resolution></range>
          </ray>
          <always_on>1</always_on>
          <visualize>true</visualize>
        </sensor>
      </link>

      <joint name="left_wheel_joint" type="revolute">
        <parent>base_link</parent><child>left_wheel</child>
        <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>
      <joint name="right_wheel_joint" type="revolute">
        <parent>base_link</parent><child>right_wheel</child>
        <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>
      <joint name="caster_joint" type="fixed">
        <parent>base_link</parent><child>caster</child>
      </joint>
      <joint name="lidar_joint" type="fixed">
        <parent>base_link</parent><child>lidar_link</child>
      </joint>

      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.30</wheel_separation>
        <wheel_radius>0.04</wheel_radius>
        <topic>cmd_vel</topic>
        <odom_topic>odom</odom_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
        <odom_publish_frequency>30</odom_publish_frequency>
        <tf_topic>tf</tf_topic>
      </plugin>

      <plugin filename="gz-sim-joint-state-publisher-system"
              name="gz::sim::systems::JointStatePublisher">
        <topic>joint_states</topic>
      </plugin>

      <plugin filename="gz-sim-pose-publisher-system"
              name="gz::sim::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <static_publisher>true</static_publisher>
        <static_update_frequency>10</static_update_frequency>
      </plugin>
    </model>"""


def walls_to_sdf(walls, size, cell_size, wall_height=1.2, wall_thickness=0.08):
    boxes = []
    for i, (x, y, length, orient) in enumerate(walls):
        if orient == 'h':
            sx, sy, sz = length, wall_thickness, wall_height
            px, py, pz = x + length / 2, y, wall_height / 2
        else:
            sx, sy, sz = wall_thickness, length, wall_height
            px, py, pz = x, y + length / 2, wall_height / 2

        boxes.append(f"""
    <model name="wall_{i}">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>0.15 0.15 0.2 1</ambient>
            <diffuse>0.25 0.28 0.35 1</diffuse>
            <specular>0.4 0.4 0.5 1</specular>
            <emissive>0.02 0.02 0.04 1</emissive>
          </material>
        </visual>
      </link>
      <pose>{px} {py} {pz} 0 0 0</pose>
    </model>""")

    floor_size = size * cell_size + 4
    start_x = 0.5 * cell_size
    start_y = -0.4
    goal_x = (size - 0.5) * cell_size
    goal_y = size * cell_size + 0.4

    markers = f"""
    <model name="start_marker">
      <static>true</static>
      <link name="link">
        <visual name="v">
          <geometry><box><size>{cell_size * 0.7} 0.4 0.01</size></box></geometry>
          <material>
            <ambient>0.0 0.6 0.1 1</ambient>
            <diffuse>0.0 0.9 0.15 1</diffuse>
            <emissive>0.0 0.4 0.05 1</emissive>
          </material>
        </visual>
      </link>
      <pose>{start_x} {start_y} 0.005 0 0 0</pose>
    </model>
    <model name="goal_marker">
      <static>true</static>
      <link name="link">
        <visual name="v">
          <geometry><box><size>{cell_size * 0.7} 0.4 0.01</size></box></geometry>
          <material>
            <ambient>0.7 0.05 0.05 1</ambient>
            <diffuse>1.0 0.1 0.1 1</diffuse>
            <emissive>0.5 0.0 0.0 1</emissive>
          </material>
        </visual>
      </link>
      <pose>{goal_x} {goal_y} 0.005 0 0 0</pose>
    </model>"""

    bot = bot_sdf(start_x, start_y - 0.1, yaw=1.5708)

    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="maze">
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.3 0.3 0.35 1</ambient>
      <background>0.05 0.06 0.1 1</background>
      <grid>false</grid>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 15 0 0 0</pose>
      <diffuse>0.95 0.95 1.0 1</diffuse>
      <specular>0.3 0.3 0.4 1</specular>
      <direction>-0.4 0.5 -1</direction>
      <attenuation>
        <range>50</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <light type="point" name="goal_glow">
      <pose>{goal_x} {goal_y} 1.5 0 0 0</pose>
      <diffuse>1.0 0.2 0.2 1</diffuse>
      <specular>0.8 0.1 0.1 1</specular>
      <attenuation>
        <range>5</range>
        <constant>0.4</constant>
        <linear>0.5</linear>
        <quadratic>0.2</quadratic>
      </attenuation>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>{floor_size} {floor_size}</size></plane></geometry></collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>{floor_size} {floor_size}</size></plane></geometry>
          <material>
            <ambient>0.12 0.13 0.16 1</ambient>
            <diffuse>0.2 0.22 0.26 1</diffuse>
            <specular>0.1 0.1 0.12 1</specular>
          </material>
        </visual>
      </link>
    </model>
{markers}
{''.join(boxes)}
{bot}
  </world>
</sdf>
"""
    return sdf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=int, default=10, help='maze size in cells (NxN)')
    ap.add_argument('--cell', type=float, default=1.0, help='cell size in meters')
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--out', type=str, default='worlds/maze.sdf')
    args = ap.parse_args()

    cells = generate_grid(args.size, args.seed)
    walls = cells_to_walls(cells, args.size, args.cell)
    sdf = walls_to_sdf(walls, args.size, args.cell)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(sdf, encoding='utf-8')
    print(f"wrote {out} ({len(walls)} walls, {args.size}x{args.size} grid)")


if __name__ == '__main__':
    main()
