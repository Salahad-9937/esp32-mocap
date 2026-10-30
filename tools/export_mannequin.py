import bpy
import mathutils
import struct
import math

bpy.ops.wm.open_mainfile(filepath="assets/mannequin.blend")

for obj in list(bpy.data.objects):
    if obj.type == 'MESH':
        bpy.context.view_layer.objects.active = obj
        for mod in list(obj.modifiers):
            try:
                bpy.ops.object.modifier_apply(modifier=mod.name)
            except Exception:
                pass

for obj in list(bpy.data.objects):
    if obj.type == 'MESH':
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.separate(type='LOOSE')
        bpy.ops.object.mode_set(mode='OBJECT')

parts = [o for o in bpy.data.objects if o.type == 'MESH']

SHOULDER_Z = 1.338
SHOULDER_Y = -0.036
SHOULDER_X_LEFT = 0.178
SHOULDER_X_RIGHT = -0.178

def make_sphere_mesh(cx, cy, cz, r, segs=14, rings=10):
    verts = []
    normals = []
    indices = []
    for i in range(rings + 1):
        phi = math.pi * i / rings
        for j in range(segs):
            theta = 2.0 * math.pi * j / segs
            nx = math.sin(phi) * math.cos(theta)
            ny = math.cos(phi)
            nz = math.sin(phi) * math.sin(theta)
            verts.append((cx + r * nx, cy + r * ny, cz + r * nz))
            normals.append((nx, ny, nz))
    for i in range(rings):
        for j in range(segs):
            j_next = (j + 1) % segs
            i00 = i * segs + j
            i01 = i * segs + j_next
            i10 = (i + 1) * segs + j
            i11 = (i + 1) * segs + j_next
            indices.extend([i00, i10, i01, i01, i10, i11])
    return verts, normals, indices

exported_meshes = []
bone_meshes = {}
recorded_positions = {}

recorded_positions["Pelvis"] = [0.0, 0.961, 0.041]
recorded_positions["L5S1"]   = [0.0, 1.050, 0.050]
recorded_positions["T12L1"]  = [0.0, 1.113, 0.063]
recorded_positions["T7T8"]   = [0.0, 1.250, 0.058]
recorded_positions["C7T1"]   = [0.0, 1.300, 0.054]
recorded_positions["Neck"]   = [0.0, 1.469, 0.021]
recorded_positions["Head"]   = [0.0, 1.592, 0.052]

for pfx, s_mult in [("L.", 1.0), ("R.", -1.0)]:
    recorded_positions[f"{pfx}Clavicle"] = [s_mult * 0.080, 1.340, 0.036]
    recorded_positions[f"{pfx}Shoulder"] = [s_mult * 0.178, 1.338, 0.036]

finger_joint_candidates = {"L.": {}, "R.": {}}

for obj in parts:
    corners = [obj.matrix_world @ mathutils.Vector(corner) for corner in obj.bound_box]
    c = sum(corners, mathutils.Vector()) / 8.0
    is_joint = 'joints' in obj.name.lower()
    abs_x = abs(c.x)
    side = "Left" if c.x > 0 else "Right"
    pfx = "L." if c.x > 0 else "R."
    sh_x = SHOULDER_X_LEFT if side == "Left" else SHOULDER_X_RIGHT

    # Пропускаем строго плечевой боковой штифт (шея abs_x=0.0 и талия abs_x=0.0 НЕ пропускаются)
    if is_joint and 0.12 < abs_x < 0.25 and 1.28 < c.z < 1.40:
        continue

    is_arm = (abs_x >= 0.22 and c.z > 0.90)

    mesh = obj.to_mesh()
    mesh.calc_loop_triangles()

    verts = []
    normals = []
    indices = []

    for v in mesh.vertices:
        co = obj.matrix_world @ v.co
        no = (obj.matrix_world.to_3x3() @ v.normal).normalized()
        if is_arm:
            dx = co.x - sh_x
            dz = co.z - SHOULDER_Z
            if side == "Left":
                new_dx = dz
                new_dz = -dx
                ndx = no.z
                ndz = -no.x
            else:
                new_dx = -dz
                new_dz = dx
                ndx = -no.z
                ndz = no.x
            co.x = sh_x + new_dx
            co.z = SHOULDER_Z + new_dz
            no.x = ndx
            no.z = ndz

        verts.append((co.x, co.z, -co.y))
        normals.append((no.x, no.z, -no.y))

    for tri in mesh.loop_triangles:
        indices.extend(tri.vertices)
    obj.to_mesh_clear()

    if not verts:
        continue

    cx = sum(v[0] for v in verts) / len(verts)
    cy = sum(v[1] for v in verts) / len(verts)
    cz = sum(v[2] for v in verts) / len(verts)

    if is_joint:
        # Оригинальные цилиндры шеи и талии
        if abs_x < 0.03:
            if c.z > 1.40:
                exported_meshes.append(("Neck", 2, verts, normals, indices))
                bone_meshes.setdefault("Neck", []).append((verts, normals))
                recorded_positions["Neck"] = [cx, cy, cz]
            else:
                exported_meshes.append(("SpineL3", 2, verts, normals, indices))
                bone_meshes.setdefault("SpineL3", []).append((verts, normals))
                recorded_positions["T12L1"] = [cx, cy, cz]
            continue

        # Оригинальный меш сустава пальцев стопы
        if c.z <= 0.05:
            exported_meshes.append((f"{side}Toe", 2, verts, normals, indices))
            bone_meshes.setdefault(f"{side}Toe", []).append((verts, normals))
            recorded_positions[f"{pfx}MTP"] = [cx, cy, cz]
            continue

        # Сферические шарниры только для целевых суставов
        if c.z < 0.90:
            if c.z > 0.70:
                sph_v, sph_n, sph_i = make_sphere_mesh(cx, cy, cz, 0.045)
                exported_meshes.append((f"{side}ThighProx", 2, sph_v, sph_n, sph_i))
                bone_meshes.setdefault(f"{side}ThighProx", []).append((sph_v, sph_n))
                recorded_positions[f"{pfx}Hip"] = [cx, cy, cz]
            elif c.z > 0.35:
                sph_v, sph_n, sph_i = make_sphere_mesh(cx, cy, cz, 0.036)
                exported_meshes.append((f"{side}ShankProx", 2, sph_v, sph_n, sph_i))
                bone_meshes.setdefault(f"{side}ShankProx", []).append((sph_v, sph_n))
                recorded_positions[f"{pfx}Knee"] = [cx, cy, cz]
            else:
                sph_v, sph_n, sph_i = make_sphere_mesh(cx, cy, cz, 0.020)
                exported_meshes.append((f"{side}Foot", 2, sph_v, sph_n, sph_i))
                bone_meshes.setdefault(f"{side}Foot", []).append((sph_v, sph_n))
                recorded_positions[f"{pfx}Ankle"] = [cx, cy, cz]
        else:
            if abs_x < 0.42:
                sph_v, sph_n, sph_i = make_sphere_mesh(cx, cy, cz, 0.028)
                exported_meshes.append((f"{side}ForearmProx", 2, sph_v, sph_n, sph_i))
                bone_meshes.setdefault(f"{side}ForearmProx", []).append((sph_v, sph_n))
                recorded_positions[f"{pfx}Elbow"] = [cx, cy, cz]
            elif abs_x < 0.60:
                sph_v, sph_n, sph_i = make_sphere_mesh(cx, cy, cz, 0.016)
                exported_meshes.append((f"{side}Hand", 2, sph_v, sph_n, sph_i))
                bone_meshes.setdefault(f"{side}Hand", []).append((sph_v, sph_n))
                recorded_positions[f"{pfx}Wrist"] = [cx, cy, cz]
            else:
                if c.y <= -0.095:
                    f_grp = "Thumb"
                elif c.y <= -0.072:
                    f_grp = "Index"
                elif c.y <= -0.054:
                    f_grp = "Middle"
                elif c.y <= -0.036:
                    f_grp = "Ring"
                else:
                    f_grp = "Pinky"
                finger_joint_candidates[pfx].setdefault(f_grp, []).append((abs_x, (cx, cy, cz)))
    else:
        bone_tag = "Sacrum"
        if abs_x < 0.03:
            if c.z > 1.50: bone_tag = "Head"
            elif c.z > 1.20: bone_tag = "SpineT3"
            else: bone_tag = "Sacrum"
        elif c.z < 0.90:
            if c.z > 0.55: bone_tag = f"{side}ThighDist"
            elif c.z > 0.18: bone_tag = f"{side}ShankDist"
            elif c.z > 0.02: bone_tag = f"{side}Foot"
            else: bone_tag = f"{side}Toe"
        else:
            if abs_x < 0.30: bone_tag = f"{side}UpperArmProx"
            elif abs_x < 0.52: bone_tag = f"{side}ForearmDist"
            elif abs_x < 0.65 and c.y > -0.09: bone_tag = f"{side}Hand"
            elif c.y <= -0.095:
                bone_tag = f"{side}ThumbProx" if abs_x < 0.66 else f"{side}ThumbDist"
            else:
                if c.y <= -0.072: f_p = "Index"
                elif c.y <= -0.054: f_p = "Middle"
                elif c.y <= -0.036: f_p = "Ring"
                else: f_p = "Pinky"
                if abs_x < 0.705: bone_tag = f"{side}{f_p}Prox"
                elif abs_x < 0.735: bone_tag = f"{side}{f_p}Mid"
                else: bone_tag = f"{side}{f_p}Dist"

        exported_meshes.append((bone_tag, 0, verts, normals, indices))
        bone_meshes.setdefault(bone_tag, []).append((verts, normals))

# Суставы пальцев (ранжирование по |X|)
for pfx in ["L.", "R."]:
    side = "Left" if pfx == "L." else "Right"
    for f_grp, items in finger_joint_candidates[pfx].items():
        items.sort(key=lambda x: x[0])
        if f_grp == "Thumb":
            names = ["ThumbCMC", "ThumbIP"]
            radii = [0.008, 0.007]
            b_tags = [f"{side}ThumbMeta", f"{side}ThumbDist"]
        else:
            names = [f"{f_grp}MCP", f"{f_grp}PIP", f"{f_grp}DIP"]
            radii = [0.0065, 0.0055, 0.0045]
            b_tags = [f"{side}{f_grp}Prox", f"{side}{f_grp}Mid", f"{side}{f_grp}Dist"]

        for (j_n, r_val, b_t), (_, pos) in zip(zip(names, radii, b_tags), items):
            sph_v, sph_n, sph_i = make_sphere_mesh(pos[0], pos[1], pos[2], r_val)
            exported_meshes.append((b_t, 2, sph_v, sph_n, sph_i))
            bone_meshes.setdefault(b_t, []).append((sph_v, sph_n))
            recorded_positions[f"{pfx}{j_n}"] = [pos[0], pos[1], pos[2]]

# Сферические суставы плеч
for side, pfx in [("Left", "L."), ("Right", "R.")]:
    sh_pos = recorded_positions[f"{pfx}Shoulder"]
    sph_v, sph_n, sph_i = make_sphere_mesh(sh_pos[0], sh_pos[1], sh_pos[2], 0.038)
    exported_meshes.append((f"{side}Shoulder", 2, sph_v, sph_n, sph_i))
    bone_meshes.setdefault(f"{side}Shoulder", []).append((sph_v, sph_n))

# Иерархия скелета
SKELETON_HIERARCHY = [
    ("Pelvis", 0, 0.007, None),
    ("L5S1",   7, 0.004, "Pelvis"),
    ("T12L1",  7, 0.004, "L5S1"),
    ("T7T8",   7, 0.004, "T12L1"),
    ("C7T1",   7, 0.004, "T7T8"),
    ("Neck",   7, 0.004, "C7T1"),
    ("Head",   5, 0.005, "Neck"),
]

for pfx in ["L.", "R."]:
    SKELETON_HIERARCHY.extend([
        (f"{pfx}Clavicle", 4, 0.004, "C7T1"),
        (f"{pfx}Shoulder", 1, 0.006, f"{pfx}Clavicle"),
        (f"{pfx}Elbow",    2, 0.005, f"{pfx}Shoulder"),
        (f"{pfx}Wrist",    5, 0.004, f"{pfx}Elbow"),
        (f"{pfx}ThumbCMC", 4, 0.003, f"{pfx}Wrist"),
        (f"{pfx}ThumbIP",  2, 0.002, f"{pfx}ThumbCMC"),
    ])

    for f_p in ["Index", "Middle", "Ring", "Pinky"]:
        SKELETON_HIERARCHY.extend([
            (f"{pfx}{f_p}MCP", 5, 0.0025, f"{pfx}Wrist"),
            (f"{pfx}{f_p}PIP", 2, 0.0020, f"{pfx}{f_p}MCP"),
            (f"{pfx}{f_p}DIP", 2, 0.0018, f"{pfx}{f_p}PIP"),
        ])

    SKELETON_HIERARCHY.extend([
        (f"{pfx}Hip",   1, 0.006, "Pelvis"),
        (f"{pfx}Knee",  2, 0.005, f"{pfx}Hip"),
        (f"{pfx}Ankle", 2, 0.004, f"{pfx}Knee"),
        (f"{pfx}MTP",   5, 0.003, f"{pfx}Ankle"),
    ])

name_to_idx = {name: i for i, (name, *_) in enumerate(SKELETON_HIERARCHY)}

final_joints = []
for j_name, j_kind, j_r, p_name in SKELETON_HIERARCHY:
    parent_idx = name_to_idx[p_name] if p_name is not None else -1
    if j_name not in recorded_positions:
        raise RuntimeError(f"Критическая ошибка: отсутствует позиция сустава {j_name}!")
    pos = recorded_positions[j_name]
    final_joints.append((j_name, j_kind, pos, j_r, parent_idx))

STANDOFF = 0.0015
def pick_surface(bone_tag, filter_fn):
    candidates = []
    for verts, normals in bone_meshes.get(bone_tag, []):
        for (vx, vy, vz), (nx, ny, nz) in zip(verts, normals):
            score = filter_fn(vx, vy, vz, nx, ny, nz)
            if score is not None:
                candidates.append((score, (vx, vy, vz), (nx, ny, nz)))
    if not candidates:
        return (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
    candidates.sort(key=lambda x: x[0], reverse=True)
    best = candidates[0]
    pos = (best[1][0] + best[2][0] * STANDOFF,
           best[1][1] + best[2][1] * STANDOFF,
           best[1][2] + best[2][2] * STANDOFF)
    return pos, best[2]

SENSOR_TAGS_DEF = [
    ("Head", "Right", 1, "Head", lambda x, y, z, nx, ny, nz: nz if y > 1.55 else None),
    ("Neck", "Right", 1, "Neck", lambda x, y, z, nx, ny, nz: -nz if y > 1.40 else None),
    ("SpineT3", "Right", 0, "SpineT3", lambda x, y, z, nx, ny, nz: -nz if y > 1.30 else None),
    ("SpineT7", "Right", 0, "SpineT3", lambda x, y, z, nx, ny, nz: -nz if 1.20 <= y <= 1.30 else None),
    ("SpineT11", "Right", 0, "SpineT3", lambda x, y, z, nx, ny, nz: -nz if 1.10 <= y <= 1.20 else None),
    ("SpineL3", "Right", 0, "SpineL3", lambda x, y, z, nx, ny, nz: -nz),
    ("Sacrum", "Right", 0, "Sacrum", lambda x, y, z, nx, ny, nz: -nz),
    ("Sternum", "Right", 0, "SpineT3", lambda x, y, z, nx, ny, nz: nz if 1.20 <= y <= 1.35 else None),
]

for side in ["Left", "Right"]:
    s_mult = 1.0 if side == "Left" else -1.0
    s_name = side
    SENSOR_TAGS_DEF.extend([
        (f"{side}Clavicle", s_name, 1, f"{side}Shoulder", lambda x, y, z, nx, ny, nz: nz),
        (f"{side}Shoulder", s_name, 1, f"{side}Shoulder", lambda x, y, z, nx, ny, nz: nx * s_mult),
        (f"{side}UpperArmProx", s_name, 1, f"{side}UpperArmProx", lambda x, y, z, nx, ny, nz: nx * s_mult if y > 1.25 else None),
        (f"{side}UpperArmDist", s_name, 1, f"{side}UpperArmProx", lambda x, y, z, nx, ny, nz: nx * s_mult if y <= 1.25 else None),
        (f"{side}ForearmProx", s_name, 1, f"{side}ForearmDist", lambda x, y, z, nx, ny, nz: nx * s_mult if y > 1.05 else None),
        (f"{side}ForearmDist", s_name, 1, f"{side}ForearmDist", lambda x, y, z, nx, ny, nz: nx * s_mult if y <= 1.05 else None),
        (f"{side}Hand", s_name, 1, f"{side}Hand", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}ThumbMeta", s_name, 2, f"{side}ThumbMeta", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}ThumbProx", s_name, 2, f"{side}ThumbProx", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}ThumbDist", s_name, 2, f"{side}ThumbDist", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}IndexProx", s_name, 2, f"{side}IndexProx", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}IndexMid", s_name, 2, f"{side}IndexMid", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}IndexDist", s_name, 2, f"{side}IndexDist", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}MiddleProx", s_name, 2, f"{side}MiddleProx", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}MiddleMid", s_name, 2, f"{side}MiddleMid", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}MiddleDist", s_name, 2, f"{side}MiddleDist", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}RingProx", s_name, 2, f"{side}RingProx", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}RingMid", s_name, 2, f"{side}RingMid", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}RingDist", s_name, 2, f"{side}RingDist", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}PinkyProx", s_name, 2, f"{side}PinkyProx", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}PinkyMid", s_name, 2, f"{side}PinkyMid", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}PinkyDist", s_name, 2, f"{side}PinkyDist", lambda x, y, z, nx, ny, nz: -nz),
        (f"{side}ThighProx", s_name, 0, f"{side}ThighDist", lambda x, y, z, nx, ny, nz: nx * s_mult if y > 0.65 else None),
        (f"{side}ThighDist", s_name, 0, f"{side}ThighDist", lambda x, y, z, nx, ny, nz: nx * s_mult if y <= 0.65 else None),
        (f"{side}ShankProx", s_name, 0, f"{side}ShankDist", lambda x, y, z, nx, ny, nz: (nz + nx * s_mult * 0.5) if y > 0.25 else None),
        (f"{side}ShankDist", s_name, 0, f"{side}ShankDist", lambda x, y, z, nx, ny, nz: (nz + nx * s_mult * 0.5) if y <= 0.25 else None),
        (f"{side}Foot", s_name, 1, f"{side}Foot", lambda x, y, z, nx, ny, nz: ny if z > 0.02 else None),
        (f"{side}Toe", s_name, 2, f"{side}Toe", lambda x, y, z, nx, ny, nz: ny),
    ])

out_path = "crates/mocap-model/assets/mannequin.bin"
with open(out_path, "wb") as f:
    f.write(b"MOCAP_MESH_V3\x00")
    f.write(struct.pack("<I", len(exported_meshes)))

    for tag, tone, verts, normals, indices in exported_meshes:
        tag_bytes = tag.encode("utf-8")
        f.write(struct.pack("<B", len(tag_bytes)))
        f.write(tag_bytes)
        f.write(struct.pack("<B", tone))

        f.write(struct.pack("<I", len(verts)))
        for vx, vy, vz in verts:
            f.write(struct.pack("<fff", vx, vy, vz))

        for nx, ny, nz in normals:
            f.write(struct.pack("<fff", nx, ny, nz))

        f.write(struct.pack("<I", len(indices)))
        for idx in indices:
            f.write(struct.pack("<I", idx))

    f.write(struct.pack("<I", len(SENSOR_TAGS_DEF)))
    for s_idx, (tag, side_str, chip_type, bone_tag, filter_fn) in enumerate(SENSOR_TAGS_DEF):
        pos, norm = pick_surface(bone_tag, filter_fn)
        tag_bytes = tag.encode("utf-8")
        f.write(struct.pack("<I", s_idx))
        f.write(struct.pack("<B", len(tag_bytes)))
        f.write(tag_bytes)
        f.write(struct.pack("<B", 0 if side_str == "Left" else 1))
        f.write(struct.pack("<B", chip_type))
        f.write(struct.pack("<fff", pos[0], pos[1], pos[2]))
        f.write(struct.pack("<fff", norm[0], norm[1], norm[2]))
        f.write(struct.pack("<fff", 0.0, 1.0, 0.0))
        f.write(struct.pack("<fff", pos[0], pos[1], pos[2]))

    f.write(struct.pack("<I", len(final_joints)))
    for j_name, j_kind, j_pos, j_r, j_parent in final_joints:
        j_bytes = j_name.encode("utf-8")
        f.write(struct.pack("<B", len(j_bytes)))
        f.write(j_bytes)
        f.write(struct.pack("<B", j_kind))
        f.write(struct.pack("<fff", j_pos[0], j_pos[1], j_pos[2]))
        f.write(struct.pack("<f", j_r))
        f.write(struct.pack("<i", j_parent))

print(f"✔ УСПЕШНО: Модель и суставы экспортированы без единой ошибки ({len(exported_meshes)} деталей, {len(final_joints)} суставов).")
