//! # Генерация вертексной геометрии базовых примитивов и сетки (`geometry`)

use super::pipeline::GlVertex;
use mocap_model::math::{add, cross, len, madd, norm, scale, sub, V3};
use mocap_model::skeleton::Tone;

#[must_use]
pub fn tone_rgba(t: Tone) -> [f32; 4] {
    match t {
        Tone::Cortical | Tone::Plate | Tone::Trabecular => [0.72, 0.74, 0.78, 1.0],
        Tone::Cartilage => [0.55, 0.62, 0.72, 1.0],
        Tone::Tooth => [0.92, 0.92, 0.90, 1.0],
        Tone::Dark => [0.18, 0.21, 0.28, 1.0],
    }
}

fn create_ortho(axis: V3) -> (V3, V3) {
    let ref_dir = if axis[1].abs() < 0.88 { [0.0, 1.0, 0.0] } else { [1.0, 0.0, 0.0] };
    let u = norm(cross(axis, ref_dir));
    let v = cross(axis, u);
    (u, v)
}

pub fn add_capsule_geo(v_out: &mut Vec<GlVertex>, i_out: &mut Vec<u32>, a: V3, b: V3, ra: f32, rb: f32, tone: Tone) {
    let axis = sub(b, a);
    let l = len(axis);
    if l < 1e-4 {
        add_sphere_geo(v_out, i_out, a, ra.max(rb), tone);
        return;
    }

    let w = scale(axis, 1.0 / l);
    let (u, v) = create_ortho(w);
    let color = tone_rgba(tone);

    const SECTORS: usize = 20;
    let mut cos_sin = [(0.0_f32, 0.0_f32); SECTORS];
    for k in 0..SECTORS {
        let angle = (k as f32) * (std::f32::consts::TAU / SECTORS as f32);
        cos_sin[k] = (angle.cos(), angle.sin());
    }

    const DOME_STACKS: usize = 4;
    let base_idx = v_out.len() as u32;

    let pole_a = madd(a, w, -ra);
    v_out.push(GlVertex { pos: pole_a, normal: scale(w, -1.0), color, ao: 0.70 });

    for s in 1..=DOME_STACKS {
        let phi = -std::f32::consts::FRAC_PI_2 + (s as f32) * (std::f32::consts::FRAC_PI_2 / DOME_STACKS as f32);
        let (sin_p, cos_p) = (phi.sin(), phi.cos());
        let ring_center = madd(a, w, ra * sin_p);
        let ring_r = ra * cos_p;
        let ao = 0.70 + 0.20 * (s as f32 / DOME_STACKS as f32);

        for &(ca, sa) in &cos_sin {
            let radial = add(scale(u, ca), scale(v, sa));
            let pos = madd(ring_center, radial, ring_r);
            let nrm = norm(add(scale(radial, cos_p), scale(w, sin_p)));
            v_out.push(GlVertex { pos, normal: nrm, color, ao });
        }
    }

    let tan_alpha = (ra - rb) / l;
    let cos_alpha = 1.0 / (1.0 + tan_alpha * tan_alpha).sqrt();
    let sin_alpha = tan_alpha * cos_alpha;

    for &(ca, sa) in &cos_sin {
        let radial = add(scale(u, ca), scale(v, sa));
        let pos = madd(b, radial, rb);
        let nrm = norm(add(scale(radial, cos_alpha), scale(w, sin_alpha)));
        v_out.push(GlVertex { pos, normal: nrm, color, ao: 0.90 });
    }

    for s in 1..DOME_STACKS {
        let phi = (s as f32) * (std::f32::consts::FRAC_PI_2 / DOME_STACKS as f32);
        let (sin_p, cos_p) = (phi.sin(), phi.cos());
        let ring_center = madd(b, w, rb * sin_p);
        let ring_r = rb * cos_p;
        let ao = 0.90 - 0.20 * (s as f32 / DOME_STACKS as f32);

        for &(ca, sa) in &cos_sin {
            let radial = add(scale(u, ca), scale(v, sa));
            let pos = madd(ring_center, radial, ring_r);
            let nrm = norm(add(scale(radial, cos_p), scale(w, sin_p)));
            v_out.push(GlVertex { pos, normal: nrm, color, ao });
        }
    }

    let pole_b = madd(b, w, rb);
    let pole_b_idx = v_out.len() as u32;
    v_out.push(GlVertex { pos: pole_b, normal: w, color, ao: 0.70 });

    for k in 0..SECTORS {
        let nk = (k + 1) % SECTORS;
        i_out.extend([base_idx, base_idx + 1 + nk as u32, base_idx + 1 + k as u32]);
    }

    let total_rings = DOME_STACKS + 1 + (DOME_STACKS - 1);
    for ring in 0..(total_rings - 1) {
        let r0 = base_idx + 1 + (ring * SECTORS) as u32;
        let r1 = base_idx + 1 + ((ring + 1) * SECTORS) as u32;
        for k in 0..SECTORS {
            let nk = (k + 1) % SECTORS;
            let (a0, a1, b0, b1) = (r0 + k as u32, r0 + nk as u32, r1 + k as u32, r1 + nk as u32);
            i_out.extend([a0, a1, b0, a1, b1, b0]);
        }
    }

    let last_ring = base_idx + 1 + ((total_rings - 1) * SECTORS) as u32;
    for k in 0..SECTORS {
        let nk = (k + 1) % SECTORS;
        i_out.extend([pole_b_idx, last_ring + k as u32, last_ring + nk as u32]);
    }
}

pub fn add_sphere_geo(v_out: &mut Vec<GlVertex>, i_out: &mut Vec<u32>, c: V3, r: f32, tone: Tone) {
    const SLICES: usize = 20;
    const STACKS: usize = 10;
    let color = tone_rgba(tone);
    let base_idx = v_out.len() as u32;

    v_out.push(GlVertex { pos: [c[0], c[1] - r, c[2]], normal: [0.0, -1.0, 0.0], color, ao: 1.0 });

    for s in 1..STACKS {
        let lat = -std::f32::consts::FRAC_PI_2 + (s as f32) * (std::f32::consts::PI / STACKS as f32);
        let (y, r_lat) = (r * lat.sin(), r * lat.cos());

        for k in 0..SLICES {
            let lon = (k as f32) * (std::f32::consts::TAU / SLICES as f32);
            let radial = [r_lat * lon.cos(), y, r_lat * lon.sin()];
            v_out.push(GlVertex { pos: add(c, radial), normal: norm(radial), color, ao: 1.0 });
        }
    }

    let pole_n_idx = v_out.len() as u32;
    v_out.push(GlVertex { pos: [c[0], c[1] + r, c[2]], normal: [0.0, 1.0, 0.0], color, ao: 1.0 });

    for k in 0..SLICES {
        let nk = (k + 1) % SLICES;
        i_out.extend([base_idx, base_idx + 1 + nk as u32, base_idx + 1 + k as u32]);
    }

    for s in 0..(STACKS - 2) {
        let r0 = base_idx + 1 + (s * SLICES) as u32;
        let r1 = base_idx + 1 + ((s + 1) * SLICES) as u32;
        for k in 0..SLICES {
            let nk = (k + 1) % SLICES;
            let (a0, a1, b0, b1) = (r0 + k as u32, r0 + nk as u32, r1 + k as u32, r1 + nk as u32);
            i_out.extend([a0, a1, b0, a1, b1, b0]);
        }
    }

    let last_ring = base_idx + 1 + ((STACKS - 2) * SLICES) as u32;
    for k in 0..SLICES {
        let nk = (k + 1) % SLICES;
        i_out.extend([pole_n_idx, last_ring + k as u32, last_ring + nk as u32]);
    }
}

pub fn add_cage_and_grid_lines(v_out: &mut Vec<GlVertex>, i_out: &mut Vec<u32>) {
    let half = 5.625_f32;
    let height = 5.775_f32;
    let step = 0.75_f32;

    let mut add_l = |p1: [f32; 3], p2: [f32; 3], color: [f32; 4]| {
        let b = v_out.len() as u32;
        v_out.push(GlVertex { pos: p1, normal: [0.0, 1.0, 0.0], color, ao: 1.0 });
        v_out.push(GlVertex { pos: p2, normal: [0.0, 1.0, 0.0], color, ao: 1.0 });
        i_out.extend([b, b + 1]);
    };

    let col_grid = [0.20, 0.25, 0.35, 0.65];
    let col_x = [0.75, 0.20, 0.20, 0.85];
    let col_z = [0.20, 0.45, 0.85, 0.85];
    let col_perim = [0.28, 0.62, 0.92, 0.85];
    let col_vert = [0.22, 0.48, 0.75, 0.45];
    let col_top = [0.18, 0.36, 0.58, 0.35];

    let num_steps = (half / step).floor() as i32;
    for i in -num_steps..=num_steps {
        let x = i as f32 * step;
        add_l([x, 0.0, -half], [x, 0.0, half], if i == 0 { col_z } else { col_grid });

        let z = i as f32 * step;
        add_l([-half, 0.0, z], [half, 0.0, z], if i == 0 { col_x } else { col_grid });
    }

    add_l([-half, 0.0, -half], [half, 0.0, -half], col_perim);
    add_l([half, 0.0, -half], [half, 0.0, half], col_perim);
    add_l([half, 0.0, half], [-half, 0.0, half], col_perim);
    add_l([-half, 0.0, half], [-half, 0.0, -half], col_perim);

    add_l([-half, 0.0, -half], [-half, height, -half], col_vert);
    add_l([half, 0.0, -half], [half, height, -half], col_vert);
    add_l([-half, 0.0, half], [-half, height, half], col_vert);
    add_l([half, 0.0, half], [half, height, half], col_vert);

    add_l([-half, height, -half], [half, height, -half], col_top);
    add_l([half, height, -half], [half, height, half], col_top);
    add_l([half, height, half], [-half, height, half], col_top);
    add_l([-half, height, half], [-half, height, -half], col_top);
}