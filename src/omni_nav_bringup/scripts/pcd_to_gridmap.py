#!/usr/bin/python3
"""Point-LIO 建出的 3D 点云地图 (odom/map 系, z 向上, 地面 z≈0) -> Nav2 2D 栅格地图 (pgm + yaml)

  占据: z in [obstacle_min_z, obstacle_max_z] 的点落到的格子
  空闲: 有地面点 (|z| < ground_tol) 且不是占据的格子
  未知: 其余

用法:
  pcd_to_gridmap.py map.pcd out_prefix [--res 0.05] [--zmin 0.10] [--zmax 1.60] [--ground 0.08]
输出 out_prefix.pgm / out_prefix.yaml
"""
import argparse
import os

import numpy as np


def read_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip()
            k, *v = line.split()
            header[k] = v
            if k == 'DATA':
                break
        fields, sizes, types = header['FIELDS'], list(map(int, header['SIZE'])), header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        n = int(header['POINTS'][0])
        fmt = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4',
               ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
        dt = np.dtype([(fn if fn != '_' else f'_pad{i}', fmt[(t, s)], (c,) if c > 1 else ())
                       for i, (fn, s, t, c) in enumerate(zip(fields, sizes, types, counts))])
        if header['DATA'][0] == 'binary':
            arr = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
        elif header['DATA'][0] == 'ascii':
            arr = np.loadtxt(f, dtype=dt)
        else:
            raise RuntimeError('binary_compressed PCD not supported, re-save as binary')
    return np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('out')
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--zmin', type=float, default=0.10)
    ap.add_argument('--zmax', type=float, default=1.60)
    ap.add_argument('--ground', type=float, default=0.08)
    ap.add_argument('--margin', type=float, default=1.0)
    a = ap.parse_args()

    P = read_pcd(a.pcd)
    P = P[np.isfinite(P).all(1)]
    lo = P[:, :2].min(0) - a.margin
    hi = P[:, :2].max(0) + a.margin
    W, H = np.ceil((hi - lo) / a.res).astype(int)
    ij = np.floor((P[:, :2] - lo) / a.res).astype(int)

    occ = np.zeros((H, W), bool)
    free = np.zeros((H, W), bool)
    m_obs = (P[:, 2] > a.zmin) & (P[:, 2] < a.zmax)
    m_gnd = np.abs(P[:, 2]) < a.ground
    occ[ij[m_obs, 1], ij[m_obs, 0]] = True
    free[ij[m_gnd, 1], ij[m_gnd, 0]] = True
    # 地面点稀疏的地方做一次 3x3 闭运算补洞
    f2 = free.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            f2 |= np.roll(np.roll(free, dy, 0), dx, 1)
    free = f2 & ~occ

    img = np.full((H, W), 205, np.uint8)
    img[free] = 254
    img[occ] = 0
    img = np.flipud(img)   # pgm 第一行是 y 最大处

    with open(a.out + '.pgm', 'wb') as f:
        f.write(f'P5\n{W} {H}\n255\n'.encode())
        f.write(img.tobytes())
    with open(a.out + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(a.out)}.pgm\nmode: trinary\nresolution: {a.res}\n'
                f'origin: [{lo[0]:.3f}, {lo[1]:.3f}, 0.0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'{len(P)} pts -> {W}x{H} grid, occupied {occ.sum()}, free {free.sum()} -> {a.out}.pgm/.yaml')


if __name__ == '__main__':
    main()
