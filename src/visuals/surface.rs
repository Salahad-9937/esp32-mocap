//! Трассировка лучей по поверхности манекена: приклеивание меток к телу и выбор мышью.
//!
//! Тело состоит из мешей (торс, голова, конечности, стопы), сфер (суставные шарики)
//! и капсул (пальцы, трапеция). Капсулы заменяются цепочкой плотных сфер — для
//! привязки меток этого достаточно (отклонение < 1 мм).

use super::skeleton::{skeleton, Prim};
use super::vec3::{add, cross, dist, dot, lerp, norm, scale, sub, V3};
use std::sync::OnceLock;

#[derive(Debug, Clone, Copy)]
pub struct Hit {
    /// Расстояние вдоль луча.
    pub t: f32,
    pub pos: V3,
    /// Единичная нормаль поверхности (наружу, к лучу).
    pub normal: V3,
}

enum Shape {
    Sphere { c: V3, r: f32 },
    Mesh { v: &'static [V3], n: &'static [V3], i: &'static [u32] },
}

struct Collider {
    lo: V3,
    hi: V3,
    shape: Shape,
}

fn sphere_collider(c: V3, r: f32) -> Collider {
    Collider {
        lo: [c[0] - r, c[1] - r, c[2] - r],
        hi: [c[0] + r, c[1] + r, c[2] + r],
        shape: Shape::Sphere { c, r },
    }
}

fn colliders() -> &'static Vec<Collider> {
    static COLLIDERS: OnceLock<Vec<Collider>> = OnceLock::new();
    COLLIDERS.get_or_init(|| {
        let mut out = Vec::new();
        for p in &skeleton().prims {
            match p {
                Prim::Sphere { c, r, .. } => out.push(sphere_collider(*c, *r)),
                Prim::Capsule { a, b, ra, rb, .. } => {
                    let rmin = ra.min(*rb).max(0.002);
                    let n = ((dist(*a, *b) / (rmin * 0.5)).ceil() as usize).clamp(2, 64);
                    for k in 0..=n {
                        let t = k as f32 / n as f32;
                        out.push(sphere_collider(lerp(*a, *b, t), ra + (rb - ra) * t));
                    }
                }
                Prim::Mesh { vertices, normals, indices, .. } => {
                    let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
                    for v in vertices {
                        for k in 0..3 {
                            lo[k] = lo[k].min(v[k]);
                            hi[k] = hi[k].max(v[k]);
                        }
                    }
                    out.push(Collider {
                        lo,
                        hi,
                        shape: Shape::Mesh { v: vertices.as_slice(), n: normals.as_slice(), i: indices.as_slice() },
                    });
                }
                Prim::Tri { .. } => {}
            }
        }
        out
    })
}

fn hits_aabb(o: V3, d: V3, lo: V3, hi: V3, tmax: f32) -> bool {
    let (mut t0, mut t1) = (0.0_f32, tmax);
    for k in 0..3 {
        if d[k].abs() < 1e-9 {
            if o[k] < lo[k] || o[k] > hi[k] {
                return false;
            }
            continue;
        }
        let inv = 1.0 / d[k];
        let (mut a, mut b) = ((lo[k] - o[k]) * inv, (hi[k] - o[k]) * inv);
        if a > b {
            std::mem::swap(&mut a, &mut b);
        }
        t0 = t0.max(a);
        t1 = t1.min(b);
        if t0 > t1 {
            return false;
        }
    }
    true
}

/// Ближайшее пересечение луча (origin, dir — единичный) с телом на расстоянии до `max_t`.
#[must_use]
pub fn raycast(o: V3, d: V3, max_t: f32) -> Option<Hit> {
    let mut best: Option<Hit> = None;
    let mut best_t = max_t;
    for col in colliders() {
        if !hits_aabb(o, d, col.lo, col.hi, best_t) {
            continue;
        }
        match &col.shape {
            Shape::Sphere { c, r } => {
                let oc = sub(o, *c);
                let b = dot(oc, d);
                let disc = b * b - (dot(oc, oc) - r * r);
                if disc < 0.0 {
                    continue;
                }
                let t = -b - disc.sqrt();
                if t >= 0.0 && t < best_t {
                    let pos = add(o, scale(d, t));
                    best_t = t;
                    best = Some(Hit { t, pos, normal: norm(sub(pos, *c)) });
                }
            }
            Shape::Mesh { v, n, i } => {
                for tri in i.chunks_exact(3) {
                    let (i0, i1, i2) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
                    let e1 = sub(v[i1], v[i0]);
                    let e2 = sub(v[i2], v[i0]);
                    let p = cross(d, e2);
                    let det = dot(e1, p);
                    if det.abs() < 1e-12 {
                        continue;
                    }
                    let inv = 1.0 / det;
                    let tv = sub(o, v[i0]);
                    let u = dot(tv, p) * inv;
                    if !(0.0..=1.0).contains(&u) {
                        continue;
                    }
                    let q = cross(tv, e1);
                    let w = dot(d, q) * inv;
                    if w < 0.0 || u + w > 1.0 {
                        continue;
                    }
                    let t = dot(e2, q) * inv;
                    if t >= 0.0 && t < best_t {
                        let k0 = 1.0 - u - w;
                        let mut nn = norm([
                            n[i0][0] * k0 + n[i1][0] * u + n[i2][0] * w,
                            n[i0][1] * k0 + n[i1][1] * u + n[i2][1] * w,
                            n[i0][2] * k0 + n[i1][2] * u + n[i2][2] * w,
                        ]);
                        if dot(nn, d) > 0.0 {
                            nn = scale(nn, -1.0);
                        }
                        best_t = t;
                        best = Some(Hit { t, pos: add(o, scale(d, t)), normal: nn });
                    }
                }
            }
        }
    }
    best
}
