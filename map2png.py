#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图查看辅助: 把 nav2 保存的 .pgm 转成 .png, 并打印尺寸/分辨率/占用统计
用法:
  python3 map2png.py                 # 自动挑 auto_save 里最新的一张
  python3 map2png.py <pgm|yaml|名字> # 指定地图文件(可省略扩展名)
生成的 .png 与地图同目录, 在 VS Code 里直接点开预览, 或 eog 打开。
"""
import os
import sys
import glob

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
MAP_DIR = os.path.join(SCRIPT_DIR, 'src', 'cod_-rm2026_-navigation',
                       'src', 'cod_bringup', 'maps')


def find_newest():
    """找不到参数时: 从 auto_save 快照和 maps 正式图里挑最新修改的一张"""
    cands = glob.glob(os.path.join(MAP_DIR, 'auto_save', '*.pgm')) + \
            glob.glob(os.path.join(MAP_DIR, '*.pgm'))
    if not cands:
        sys.exit('没找到任何 .pgm 地图, 检查: %s' % MAP_DIR)
    return max(cands, key=os.path.getmtime)


def resolve(arg):
    """把用户给的路径/名字解析成 .pgm 文件"""
    for base_dir in ('.', SCRIPT_DIR):
        for c in (arg, arg + '.pgm', arg + '.yaml'):
            p = os.path.join(base_dir, c) if not os.path.isabs(c) else c
            if os.path.isfile(p):
                if p.endswith('.pgm'):
                    return p
                pgm = os.path.splitext(p)[0] + '.pgm'
                if os.path.isfile(pgm):
                    return pgm
                sys.exit('%s 存在但没有对应的 .pgm' % p)
    sys.exit('找不到地图文件: %s' % arg)


def read_yaml_info(pgm):
    yml = os.path.splitext(pgm)[0] + '.yaml'
    if not os.path.isfile(yml):
        return None
    try:
        import yaml
        with open(yml) as f:
            return yaml.safe_load(f)
    except Exception:
        return None


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    pgm = resolve(arg) if arg else find_newest()

    from PIL import Image
    img = Image.open(pgm)
    png = os.path.splitext(pgm)[0] + '.png'
    img.save(png)

    print('地图文件: %s' % pgm)
    print('像素尺寸: %d x %d' % img.size)
    info = read_yaml_info(pgm)
    if info:
        res = float(info.get('resolution', 0.05))
        print('分辨率  : %.3f m/px  ->  实际 %.1f m x %.1f m'
              % (res, img.size[0] * res, img.size[1] * res))
        if 'origin' in info:
            print('原点    : %s' % info['origin'])

    # 占用统计: PGM 约定 0=占用(黑) 254=空闲(白) 205=未知(灰)
    try:
        import numpy as np
        a = np.asarray(img.convert('L'), dtype=np.uint8)
        total = a.size
        occ = int((a < 50).sum())
        free = int((a > 250).sum())
        unk = total - occ - free
        print('区域占比: 占用 %.1f%% | 空闲 %.1f%% | 未知 %.1f%%'
              % (occ / total * 100, free / total * 100, unk / total * 100))
        if free + occ < total * 0.02:
            print('提示: 有效区域过少, 地图可能是空的, 检查建图是否真的在跑')
    except ImportError:
        pass

    print('已生成  : %s' % png)
    print('查看方式: VS Code 里点开该 png, 或执行  eog %s' % png)


if __name__ == '__main__':
    main()
