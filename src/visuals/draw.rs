//! Рисование скелета: все примитивы проецируются, сортируются по глубине
//! (от дальних к ближним) и выводятся одним списком.

use super::axes::draw_axes_gizmo;
use super::skeleton::{Joint, Prim, SensorSite, Skeleton, Tone};
use super::vec3::{add, cross, dot, madd, norm, scale, V3};
use super::viewport::OrbitalCamera;
use egui::{Color32, Painter, Pos2, Rect, Shape, Stroke, Vec2};

fn mul(c: Color32, k: f32) -> Color32 {
    let f = |v: u8| (f32::from(v) * k).clamp(0.0, 255.0) as u8;
    Color32::from_rgb(f(c.r()), f(c.g()), f(c.b()))
}

fn tone_color(t: Tone) -> Color32 {
    match t {
        Tone::Cortical | Tone::Trabecular | Tone::Plate => Color32::from_rgb(172, 179, 193),
        Tone::Cartilage => Color32::from_rgb(120, 160, 190),
        Tone::Tooth => Color32::from_rgb(248, 248, 242),
        Tone::Dark => Color32::from_rgb(22, 24, 32),
    }
}

struct View<'a> {
    cam: &'a OrbitalCamera,
    rect: Rect,
    k: f32,
    fwd: V3,
}

type Items = Vec<(f32, Shape)>;

/// Тело рисуется тремя слоями без сортировки: общий контур, общая заливка одним цветом
/// (без бликов). Поэтому перекрывающиеся части сливаются в единую фигуру.
struct Passes {
    outline: Vec<Shape>,
    fill: Vec<Shape>,
}

impl View<'_> {
    fn proj(&self, p: V3) -> Option<(Pos2, f32)> {
        self.cam.project_depth(p, self.rect)
    }

    /// Дальние элементы темнее — глубина читается лучше.
    fn shade(&self, depth: f32) -> f32 {
        1.0 - 0.40 * ((depth - (self.cam.distance - 0.8)) / 1.6).clamp(0.0, 1.0)
    }

    fn capsule(&self, ps: &mut Passes, a: V3, b: V3, ra: f32, rb: f32, tone: Tone) {
        let (Some((pa, da)), Some((pb, db))) = (self.proj(a), self.proj(b)) else { return };
        let (ra, rb) = (ra * self.k / da, rb * self.k / db);
        let fill = tone_color(tone);
        let edge = mul(fill, 0.28);
        if ra.max(rb) < 1.7 {
            let w = (ra + rb).max(1.0);
            ps.outline.push(Shape::line_segment([pa, pb], Stroke::new(w + 2.0, edge)));
            ps.fill.push(Shape::line_segment([pa, pb], Stroke::new(w, fill)));
            return;
        }
        let d = pb - pa;
        let l = d.length();
        let n = if l > 0.5 { Vec2::new(-d.y, d.x) / l } else { Vec2::new(0.0, 1.0) };
        for (grow, col, dst) in [(1.3_f32, edge, &mut ps.outline), (0.0, fill, &mut ps.fill)] {
            let (r1, r2) = (ra + grow, rb + grow);
            dst.push(Shape::circle_filled(pa, r1, col));
            dst.push(Shape::circle_filled(pb, r2, col));
            if l > 0.5 {
                dst.push(Shape::convex_polygon(
                    vec![pa + n * r1, pb + n * r2, pb - n * r2, pa - n * r1],
                    col,
                    Stroke::NONE,
                ));
            }
        }
    }

    fn sphere(&self, ps: &mut Passes, c: V3, r: f32, tone: Tone) {
        let Some((p, d)) = self.proj(c) else { return };
        let r = r * self.k / d;
        let fill = tone_color(tone);
        ps.outline.push(Shape::circle_filled(p, r + 1.3, mul(fill, 0.28)));
        ps.fill.push(Shape::circle_filled(p, r, fill));
    }

    fn tri(&self, ps: &mut Passes, p: [V3; 3], tone: Tone) {
        let (Some(a), Some(b), Some(c)) = (self.proj(p[0]), self.proj(p[1]), self.proj(p[2])) else { return };
        ps.fill.push(Shape::convex_polygon(vec![a.0, b.0, c.0], tone_color(tone), Stroke::NONE));
    }

    /// Только крупные суставы, единым стилем: тёмная точка со светлым кольцом.
    fn joint(&self, out: &mut Items, j: &Joint) {
        if j.marker_r < 0.012 {
            return;
        }
        let Some((p, d)) = self.proj(j.pos) else { return };
        let r = (j.marker_r * self.k / d * 0.7).clamp(2.5, 6.0);
        let s = vec![
            Shape::circle_filled(p, r + 1.2, Color32::from_rgb(150, 205, 255)),
            Shape::circle_filled(p, r, Color32::from_rgb(28, 40, 70)),
        ];
        out.push((d - 0.02, Shape::Vec(s)));
    }

    /// Плата датчика: жёлтая — свободное место, зелёная — активный датчик.
    fn sensor(&self, out: &mut Items, st: &SensorSite, active: bool) {
        let Some((_, dc)) = self.proj(st.pos) else { return };
        let facing = dot(st.normal, self.fwd) < 0.0;
        let half = st.chip.half();
        let k = (2.8 / (half * self.k / dc)).max(1.0); // слишком мелкие платы увеличиваем до 2.8 px
        let u = st.tangent;
        let v = norm(cross(st.normal, u));
        let (hu, hv) = (half * k, half * 0.75 * k);
        let corners = [
            madd(madd(st.pos, u, hu), v, hv),
            madd(madd(st.pos, u, hu), v, -hv),
            madd(madd(st.pos, u, -hu), v, -hv),
            madd(madd(st.pos, u, -hu), v, hv),
        ];
        let mut pts = Vec::with_capacity(4);
        for c in corners {
            let Some((p, _)) = self.proj(c) else { return };
            pts.push(p);
        }
        let op = if facing { 1.0 } else { 0.35 };
        let (fill, border, dot_c, w) = if active {
            (Color32::from_rgb(18, 70, 36), Color32::from_rgb(40, 240, 90), Color32::WHITE, 1.8)
        } else {
            (Color32::from_rgb(110, 90, 16), Color32::from_rgb(255, 215, 30), Color32::from_rgb(255, 235, 90), 1.3)
        };
        let center = self.proj(st.pos).map_or(pts[0], |c| c.0);
        let bs = Stroke::new(w, border.gamma_multiply(op));
        let mut s = vec![Shape::convex_polygon(pts.clone(), fill.gamma_multiply(op), Stroke::NONE)];
        for i in 0..4 {
            s.push(Shape::line_segment([pts[i], pts[(i + 1) % 4]], bs));
        }
        if (pts[0] - pts[2]).length() > 9.0 {
            s.push(Shape::circle_filled(center, 1.5, dot_c.gamma_multiply(op)));
        }
        let sd = self.proj(st.support).map_or(dc, |p| p.1);
        out.push((sd + if facing { -0.015 } else { 0.015 }, Shape::Vec(s)));
    }
}

/// Рисует скелет, суставы и датчики; `quats[id]` — кватернион активного датчика.
pub fn draw_skeleton(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    skel: &Skeleton,
    quats: &[Option<[f32; 4]>],
) {
    let v = View {
        cam: camera,
        rect,
        k: camera.pixels_per_meter_at_unit_depth(rect),
        fwd: camera.forward(),
    };
    let mut ps = Passes { outline: Vec::new(), fill: Vec::new() };
    for p in &skel.prims {
        match *p {
            Prim::Capsule { a, b, ra, rb, tone } => v.capsule(&mut ps, a, b, ra, rb, tone),
            Prim::Sphere { c, r, tone } => v.sphere(&mut ps, c, r, tone),
            Prim::Tri { p, tone } => v.tri(&mut ps, p, tone),
        }
    }
    painter.extend(ps.outline);
    painter.extend(ps.fill);
    // Суставы и датчики — поверх тела; обратная сторона приглушена
    let mut items: Items = Vec::with_capacity(128);
    for j in &skel.joints {
        v.joint(&mut items, j);
    }
    for (st, q) in skel.sites.iter().zip(quats) {
        v.sensor(&mut items, st, q.is_some());
    }
    items.sort_by(|a, b| b.0.total_cmp(&a.0));
    painter.extend(items.into_iter().map(|(_, s)| s));

    // Оси активных датчиков поверх всего (на обратной стороне — приглушены)
    for (st, q) in skel.sites.iter().zip(quats) {
        if let Some(q) = q {
            let facing = dot(st.normal, v.fwd) < 0.0;
            let base = add(st.pos, scale(st.normal, 0.0));
            draw_axes_gizmo(painter, camera, rect, base, *q, st.chip.axis_len(), if facing { 1.0 } else { 0.35 });
        }
    }
}

/// Горизонтальная координатная сетка пола при Y = 0.
pub fn draw_floor_grid(painter: &Painter, camera: &OrbitalCamera, rect: Rect, size: f32, step: f32) {
    let grid_stroke = Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(50, 65, 85, 90));
    let axis_stroke_x = Stroke::new(1.4_f32, Color32::from_rgba_unmultiplied(180, 50, 50, 130));
    let axis_stroke_z = Stroke::new(1.4_f32, Color32::from_rgba_unmultiplied(50, 100, 200, 130));

    let half = size * 0.5;
    let mut x = -half;
    while x <= half + 0.001 {
        let stroke = if x.abs() < 0.001 { axis_stroke_z } else { grid_stroke };
        if let (Some(p1), Some(p2)) = (camera.project([x, 0.0, -half], rect), camera.project([x, 0.0, half], rect)) {
            painter.line_segment([p1, p2], stroke);
        }
        x += step;
    }
    let mut z = -half;
    while z <= half + 0.001 {
        let stroke = if z.abs() < 0.001 { axis_stroke_x } else { grid_stroke };
        if let (Some(p1), Some(p2)) = (camera.project([-half, 0.0, z], rect), camera.project([half, 0.0, z], rect)) {
            painter.line_segment([p1, p2], stroke);
        }
        z += step;
    }
}