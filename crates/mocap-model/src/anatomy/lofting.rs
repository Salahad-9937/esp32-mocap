//! # Геометрический лофтинг сечений и генерация триангулированных мешей (`lofting`)

use super::super::math::{cross, lerp, norm, sub, V3};
use super::super::skeleton::{Prim, Tone};
use super::{cr, Builder};
use std::f32::consts::TAU;

pub const RING_N: usize = 24;

#[derive(Clone, Copy)]
pub struct Ring {
    pub c: V3,
    pub rx: f32,
    pub rz: f32,
    pub ex: f32,
}

#[must_use]
pub fn ring_pts(r: &Ring, k: f32, dy: f32) -> Vec<V3> {
    (0..RING_N)
        .map(|i| {
            let ang = TAU * i as f32 / RING_N as f32;
            let (sa, ca) = ang.sin_cos();
            let e = 2.0 / r.ex;
            let x = r.rx * k * ca.signum() * ca.abs().powf(e);
            let z = r.rz * k * sa.signum() * sa.abs().powf(e);
            [r.c[0] + x, r.c[1] + dy, r.c[2] + z]
        })
        .collect()
}

#[must_use]
pub fn refine(rings: &[Ring]) -> Vec<Ring> {
    const SUB: usize = 3;
    if rings.len() < 2 {
        return rings.to_vec();
    }
    let rx: Vec<f32> = rings.iter().map(|q| q.rx).collect();
    let rz: Vec<f32> = rings.iter().map(|q| q.rz).collect();
    let mut out = Vec::new();
    for i in 0..rings.len() - 1 {
        for j in 0..SUB {
            let f = j as f32 / SUB as f32;
            out.push(Ring {
                c: lerp(rings[i].c, rings[i + 1].c, f),
                rx: cr(&rx, i, f),
                rz: cr(&rz, i, f),
                ex: rings[i].ex + (rings[i + 1].ex - rings[i].ex) * f,
            });
        }
    }
    out.push(rings[rings.len() - 1]);
    out
}

#[must_use]
pub fn mesh_normals(v: &[V3], idx: &[u32]) -> Vec<V3> {
    let mut n = vec![[0.0_f32; 3]; v.len()];
    for t in idx.chunks_exact(3) {
        let (i0, i1, i2) = (t[0] as usize, t[1] as usize, t[2] as usize);
        let f = cross(sub(v[i1], v[i0]), sub(v[i2], v[i0]));
        for i in [i0, i1, i2] {
            n[i] = super::super::math::add(n[i], f);
        }
    }
    n.into_iter().map(norm).collect()
}

pub fn loft(b: &mut Builder, rings: &[Ring], bot: Option<f32>, top: Option<f32>, tone: Tone) {
    let refined = refine(rings);
    let rings = &refined[..];
    let n = RING_N;
    let mut layers: Vec<Vec<V3>> = Vec::new();
    let angs = [75.0_f32, 50.0, 25.0];
    if let Some(h) = bot {
        for th in angs {
            let (s, c) = th.to_radians().sin_cos();
            layers.push(ring_pts(&rings[0], c, -h * s));
        }
    }
    for r in rings {
        layers.push(ring_pts(r, 1.0, 0.0));
    }
    if let Some(h) = top {
        let last = &rings[rings.len() - 1];
        for th in angs.iter().rev() {
            let (s, c) = th.to_radians().sin_cos();
            layers.push(ring_pts(last, c, h * s));
        }
    }
    let l = layers.len();
    let first = &rings[0];
    let last = &rings[rings.len() - 1];
    let mut verts: Vec<V3> = layers.into_iter().flatten().collect();
    let apex_bot = verts.len() as u32;
    verts.push(match bot {
        Some(h) => [first.c[0], first.c[1] - h, first.c[2]],
        None => first.c,
    });
    let apex_top = verts.len() as u32;
    verts.push(match top {
        Some(h) => [last.c[0], last.c[1] + h, last.c[2]],
        None => last.c,
    });

    let mut idx: Vec<u32> = Vec::new();
    for i in 0..l - 1 {
        for k in 0..n {
            let nk = (k + 1) % n;
            let a0 = (i * n + k) as u32;
            let a1 = (i * n + nk) as u32;
            let b0 = ((i + 1) * n + k) as u32;
            let b1 = ((i + 1) * n + nk) as u32;
            idx.extend([a0, b0, a1, a1, b0, b1]);
        }
    }
    for k in 0..n {
        let nk = (k + 1) % n;
        idx.extend([k as u32, nk as u32, apex_bot]);
        idx.extend([((l - 1) * n + k) as u32, apex_top, ((l - 1) * n + nk) as u32]);
    }
    let normals = mesh_normals(&verts, &idx);
    b.prims.push(Prim::Mesh { vertices: verts, normals, indices: idx, tone });
    b.prim_bones.push(b.current_bone);
}

#[allow(clippy::too_many_arguments)]
pub fn limb(
    b: &mut Builder,
    key: Option<String>,
    a: V3,
    e: V3,
    prof: &[(f32, f32)],
    aspect: f32,
    n0: V3,
    n90: V3,
    round_a: bool,
    round_e: bool,
) {
    let mut rings: Vec<Ring> = prof
        .iter()
        .map(|&(t, r)| Ring { c: lerp(a, e, t), rx: r, rz: r * aspect, ex: 2.0 })
        .collect();
    let (mut bot_round, mut top_round) = (round_a, round_e);
    if a[1] > e[1] {
        rings.reverse();
        std::mem::swap(&mut bot_round, &mut top_round);
    }
    let dome = |r: &Ring| 0.55 * r.rx.min(r.rz);
    let bot = bot_round.then(|| dome(&rings[0]));
    let top = top_round.then(|| dome(&rings[rings.len() - 1]));
    loft(b, &rings, bot, top, Tone::Cortical);
    if let Some(k) = key {
        b.segs.insert(k, super::Seg { a, b: e, prof: prof.to_vec(), n0, n90 });
    }
}