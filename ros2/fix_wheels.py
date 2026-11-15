#!/usr/bin/env python3
"""
JNNP RoboDojo - fix the swinging / jumping turns in Gazebo.

Why the robot swings:
  * It is a 4-wheel SKID-STEER robot. When it spins, the wheels must slide sideways.
  * The wheels touch the floor with their CAD mesh shapes (many flat faces), so while
    sliding they bump from face to face and the grip is high -> the robot jerks and
    swings back when it stops.

What this script does to the robot file (URDF):
  1. Replaces each wheel's touching shape (collision) with a perfectly round cylinder
     of the same size (worked out from the wheel's own mesh file). The look is unchanged.
  2. Sets a medium grip for the wheels (mu 0.5) so they slide smoothly when spinning.
A backup is saved as  <file>.before_wheelfix  first.

Run (laptop, Ubuntu):
    python3 ~/fix_wheels.py
"""
import glob
import math
import os
import re
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

PKG_DIR = os.environ.get('ROBODOJO_PKG', os.path.expanduser('~/ros2_ws/src/jnnp_robodojo_robot'))
GRIP = 0.5            # wheel friction (lower = slides easier when spinning)


def find_urdf():
    files = [f for f in glob.glob(os.path.join(PKG_DIR, 'urdf', '*.urdf'))]
    if not files:
        sys.exit(f'No .urdf file found in {PKG_DIR}/urdf')
    return sorted(files)[0]


def read_stl(path):
    data = open(path, 'rb').read()
    pts = []
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if len(data) == 84 + 50 * n:                      # binary STL
            for i in range(n):
                off = 84 + 50 * i + 12
                for v in range(3):
                    pts.append(struct.unpack('<3f', data[off + 12 * v: off + 12 * v + 12]))
            return pts
    for line in data.decode('latin-1').splitlines():       # ASCII STL
        p = line.split()
        if len(p) == 4 and p[0] == 'vertex':
            pts.append(tuple(float(x) for x in p[1:]))
    return pts


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def resolve_mesh(filename):
    if filename.startswith('package://'):
        rest = filename[len('package://'):]
        rest = rest.split('/', 1)[1] if '/' in rest else rest
        return os.path.join(PKG_DIR, rest)
    if filename.startswith('file://'):
        return filename[len('file://'):]
    return os.path.join(PKG_DIR, filename)


def nums(text, default):
    return [float(x) for x in text.split()] if text else default


def main():
    urdf = find_urdf()
    print('Robot file:', urdf)
    text = open(urdf).read()

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(text, parser=parser)

    plugin = next((p for p in root.iter('plugin') if 'diff-drive' in (p.get('filename') or '')), None)
    if plugin is None:
        sys.exit('DiffDrive plugin not found in the robot file.')
    wheel_joints = [e.text.strip() for e in plugin if e.tag in ('left_joint', 'right_joint')]
    plugin_radius = float(plugin.findtext('wheel_radius', '0'))
    print('Wheel joints:', ', '.join(wheel_joints))

    joints = {j.get('name'): j for j in root.findall('joint')}
    links = {l.get('name'): l for l in root.findall('link')}
    wheel_links = []

    for jn in wheel_joints:
        joint = joints.get(jn)
        if joint is None:
            sys.exit(f'Joint {jn} not found.')
        link_name = joint.find('child').get('link')
        wheel_links.append(link_name)
        axis = nums(joint.find('axis').get('xyz') if joint.find('axis') is not None else '', [1, 0, 0])
        ai = max(range(3), key=lambda i: abs(axis[i]))
        link = links[link_name]
        col = link.find('collision')
        if col is None:
            print(f'  {link_name}: no collision shape - skipped')
            continue
        geom = col.find('geometry')
        mesh = geom.find('mesh')
        if mesh is None:
            print(f'  {link_name}: already a simple shape ({geom[0].tag}) - only grip is changed')
            continue

        path = resolve_mesh(mesh.get('filename'))
        scale = nums(mesh.get('scale'), [1, 1, 1])
        org = col.find('origin')
        xyz = nums(org.get('xyz') if org is not None else '', [0, 0, 0])
        R = rpy_matrix(*nums(org.get('rpy') if org is not None else '', [0, 0, 0]))
        pts = read_stl(path)
        if not pts:
            sys.exit(f'Could not read wheel mesh {path}')
        tp = []
        for p in pts:
            s = [p[0] * scale[0], p[1] * scale[1], p[2] * scale[2]]
            tp.append([R[i][0] * s[0] + R[i][1] * s[1] + R[i][2] * s[2] + xyz[i] for i in range(3)])
        lo = [min(p[i] for p in tp) for i in range(3)]
        hi = [max(p[i] for p in tp) for i in range(3)]
        centre = [(lo[i] + hi[i]) / 2 for i in range(3)]
        width = hi[ai] - lo[ai]
        radius = max(hi[i] - lo[i] for i in range(3) if i != ai) / 2
        if plugin_radius and abs(radius - plugin_radius) / plugin_radius > 0.15:
            print(f'  NOTE: mesh radius {radius:.4f} m differs from plugin wheel_radius {plugin_radius} m')
        # cylinder is along z: turn it to lie along the wheel axle
        rpy = {0: '0 1.5708 0', 1: '1.5708 0 0', 2: '0 0 0'}[ai]

        if org is None:
            org = ET.Element('origin')
            col.insert(0, org)
        org.set('xyz', ' '.join(f'{c:.5f}' for c in centre))
        org.set('rpy', rpy)
        geom.remove(mesh)
        ET.SubElement(geom, 'cylinder', radius=f'{radius:.5f}', length=f'{width:.5f}')
        print(f'  {link_name}: round cylinder  radius {radius * 1000:.1f} mm  width {width * 1000:.1f} mm')

    # grip: remove old friction settings for the wheels, add new ones
    for g in root.findall('gazebo'):
        if g.get('reference') in wheel_links:
            for tag in ('mu1', 'mu2', 'fdir1', 'kp', 'kd', 'minDepth', 'maxVel'):
                for e in g.findall(tag):
                    g.remove(e)
            if len(g) == 0:
                root.remove(g)                     # empty now - drop it
    for ln in wheel_links:
        g = ET.SubElement(root, 'gazebo', reference=ln)
        for tag, val in (('mu1', GRIP), ('mu2', GRIP), ('kp', '1000000.0'), ('kd', '100.0'),
                         ('minDepth', '0.001')):
            ET.SubElement(g, tag).text = str(val)

    backup = urdf + '.before_wheelfix'
    if not os.path.exists(backup):
        shutil.copy(urdf, backup)
        print('Backup saved:', backup)
    ET.indent(root, '  ')
    out = ET.tostring(root, encoding='unicode')
    head = re.match(r'\s*<\?xml[^>]*\?>', text)
    open(urdf, 'w').write((head.group(0).strip() if head else '<?xml version="1.0"?>') + '\n' + out + '\n')
    print('Done. Wheels are now round, with smooth grip.')


if __name__ == '__main__':
    main()
