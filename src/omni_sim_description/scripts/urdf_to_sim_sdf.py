#!/usr/bin/python3
"""robot.urdf.xacro -> gz 用的 SDF, 并给所有碰撞体写入零摩擦 <surface>.

为什么需要: URDF 的 <gazebo reference><mu1>/<mu2> 经 sdformat 转换后, 对轮子(被当成 lump 处理的
碰撞体)不生效. 碰撞体摩擦不为 0 时, 地面摩擦力会被 IMU 当作比力测到 (≈1g, 与运动方向相反),
而 VelocityControl 又强行保持速度 → IMU 与真实运动不一致 → LIO 发散. 这里直接在 SDF 里写死.

用法: urdf_to_sim_sdf.py <robot.urdf.xacro> <out.sdf> [xacro_arg:=value ...]
"""
import subprocess
import sys
import xml.etree.ElementTree as ET

SURFACE = ('<surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode>'
           '<bullet><friction>0.0</friction><friction2>0.0</friction2></bullet></friction>'
           '<contact><ode><kp>1e6</kp><kd>100</kd></ode></contact></surface>')


def main():
    xacro_file, out = sys.argv[1], sys.argv[2]
    urdf = subprocess.run(['xacro', xacro_file, *sys.argv[3:]], check=True,
                          capture_output=True, text=True).stdout
    tmp = out + '.urdf'
    with open(tmp, 'w') as f:
        f.write(urdf)
    sdf = subprocess.run(['gz', 'sdf', '-p', tmp], check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(sdf)
    n = 0
    for col in root.iter('collision'):
        old = col.find('surface')
        if old is not None:
            col.remove(old)
        col.append(ET.fromstring(SURFACE))
        n += 1
    ET.ElementTree(root).write(out, encoding='utf-8', xml_declaration=True)
    print(f'wrote {out} ({n} collisions set to zero friction)')


if __name__ == '__main__':
    main()
