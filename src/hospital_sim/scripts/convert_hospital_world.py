#!/usr/bin/python3
"""把 aws-robomaker-hospital-world 的 Gazebo Classic world 转成 gz sim (Harmonic) 可用的 SDF。

改动:
  1. 去掉 ODE physics,换成 gz 的 Physics/Sensors/Imu/SceneBroadcaster/UserCommands 系统插件
  2. model://sun、model://ground_plane 改为内联定义(不依赖 Classic 模型库)
  3. OpenRobotics Fuel 模型(病床/病人/椅子...) 改成 https://fuel.gazebosim.org URI
     (已用 `gz fuel download` 缓存到 ~/.gz/fuel, 离线可用)
  4. 可选去掉天花板(--no-ceiling, 默认去掉: 方便俯视观察; 保留则雷达会扫到天花板)

用法:
  python3 convert_hospital_world.py <hospital.world> <out.sdf> [--keep-ceiling]
"""
import sys
import xml.etree.ElementTree as ET

FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'

SYSTEMS = [
    ('gz-sim-physics-system', 'gz::sim::systems::Physics', {}),
    ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands', {}),
    ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster', {}),
    ('gz-sim-sensors-system', 'gz::sim::systems::Sensors', {'render_engine': 'ogre2'}),
    ('gz-sim-imu-system', 'gz::sim::systems::Imu', {}),
]

SUN = """
<light type="directional" name="sun">
  <cast_shadows>false</cast_shadows>
  <pose>0 0 30 0 0 0</pose>
  <diffuse>0.9 0.9 0.9 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <direction>-0.3 0.2 -0.9</direction>
</light>"""

GROUND = """
<model name="ground_plane">
  <static>true</static>
  <pose>0 0 -0.1 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
    </collision>
    <visual name="visual">
      <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
      <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
    </visual>
  </link>
</model>"""


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    src, dst = sys.argv[1], sys.argv[2]
    keep_ceiling = '--keep-ceiling' in sys.argv

    tree = ET.parse(src)
    world = tree.getroot().find('world')
    world.set('name', 'hospital')

    for tag in ('physics', 'scene'):
        for e in world.findall(tag):
            world.remove(e)

    # 顶层 include: sun / ground_plane
    for inc in list(world.findall('include')):
        uri = inc.find('uri').text.strip()
        if uri in ('model://sun', 'model://ground_plane'):
            world.remove(inc)

    removed = 0
    for model in list(world.findall('model')):
        inc = model.find('include')
        if inc is None:
            continue
        uri_el = inc.find('uri')
        name = uri_el.text.strip().replace('model://', '')
        if not keep_ceiling and 'ceiling' in name:
            world.remove(model)
            removed += 1
            continue
        if not name.startswith('aws_robomaker_'):
            uri_el.text = FUEL + name

    head = []
    physics = ET.fromstring(
        '<physics name="1ms" type="ignored"><max_step_size>0.002</max_step_size>'
        '<real_time_factor>1.0</real_time_factor></physics>')
    head.append(physics)
    for fname, cname, params in SYSTEMS:
        p = ET.Element('plugin', {'filename': fname, 'name': cname})
        for k, v in params.items():
            ET.SubElement(p, k).text = v
        head.append(p)
    head.append(ET.fromstring(
        '<scene><ambient>0.7 0.7 0.7 1</ambient><background>0.8 0.8 0.85 1</background>'
        '<shadows>false</shadows><grid>false</grid></scene>'))
    head.append(ET.fromstring(SUN))
    head.append(ET.fromstring(GROUND))
    for i, e in enumerate(head):
        world.insert(1 + i, e)

    tree.getroot().set('version', '1.9')
    ET.indent(tree, space='  ')
    tree.write(dst, encoding='utf-8', xml_declaration=True)
    print(f'wrote {dst} (removed {removed} ceiling models)')


if __name__ == '__main__':
    main()
