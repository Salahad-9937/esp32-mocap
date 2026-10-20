//! # Аппаратный 3D-растеризатор манекена (`draw`)
//!
//! Исполняет рендеринг полигонального анатомического скелета, суставов, двусторонних чипов
//! сенсоров, напольной сетки и объемного каркаса рабочей зоны через конвейер OpenGL Glow.

use super::axes::rotate_vector_by_quat;
use super::placement::{self, PlacedChip};
use super::skeleton::{Prim, Skeleton, Tone};
use super::vec3::{add, cross, madd, norm, scale, sub, V3};
use super::viewport::OrbitalCamera;
use eframe::egui_glow;
use eframe::glow::{self, HasContext};
use egui::{Painter, Rect};
use std::sync::{Arc, Mutex, OnceLock};

/// Формат 3D-вершины для аппаратного конвейера OpenGL (stride = 44 байта).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct GlVertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
    pub ao: f32,
}

const VERTEX_SHADER_SRC: &str = r#"
in vec3 a_pos;
in vec3 a_normal;
in vec4 a_color;
in float a_ao;

uniform mat4 u_mvp;
uniform vec3 u_light_key;
uniform vec3 u_light_fill;
uniform vec3 u_eye_pos;
uniform int u_is_line;

out vec4 v_color;

void main() {
    gl_Position = u_mvp * vec4(a_pos, 1.0);

    if (u_is_line == 1) {
        v_color = a_color;
    } else {
        vec3 n = normalize(a_normal);
        vec3 lk = normalize(u_light_key);
        vec3 lf = normalize(u_light_fill);

        float n_dot_k = clamp(dot(n, lk) * 0.5 + 0.5, 0.0, 1.0);
        float key = n_dot_k * n_dot_k;

        float fill = max(dot(n, lf), 0.0) * 0.32;

        vec3 eye_dir = normalize(u_eye_pos - a_pos);
        float n_dot_v = clamp(1.0 - abs(dot(n, eye_dir)), 0.0, 1.0);
        float rim = pow(n_dot_v, 3.0) * 0.28;

        vec3 warm = vec3(1.10, 1.06, 0.96);
        vec3 cool = vec3(0.82, 0.90, 1.12);
        vec3 base = a_color.rgb;
        vec3 blended = base * mix(cool, warm, key);

        float ambient = 0.24;
        vec3 lit = (blended * (ambient + key * 0.68 + fill) * a_ao) + vec3(rim);

        v_color = vec4(clamp(lit, 0.0, 1.0), a_color.a);
    }
}
"#;

const FRAGMENT_SHADER_SRC: &str = r#"
in vec4 v_color;
out vec4 FragColor;

void main() {
    FragColor = v_color;
}
"#;

unsafe fn compile_shader(
    gl: &glow::Context,
    shader_type: u32,
    extra_header: &str,
    source: &str,
) -> Result<glow::Shader, String> {
    let shader = gl.create_shader(shader_type)?;
    let version = egui_glow::ShaderVersion::get(gl);
    let full_src = format!("{}\n{}{}", version.version_declaration(), extra_header, source);
    gl.shader_source(shader, &full_src);
    gl.compile_shader(shader);
    if !gl.get_shader_compile_status(shader) {
        let log = gl.get_shader_info_log(shader);
        gl.delete_shader(shader);
        return Err(log);
    }
    Ok(shader)
}

unsafe fn create_program(gl: &glow::Context) -> Result<glow::Program, String> {
    let shader_version = egui_glow::ShaderVersion::get(gl);
    let fs_header = if shader_version.is_embedded() {
        "precision mediump float;\n"
    } else {
        ""
    };

    let vs = compile_shader(gl, glow::VERTEX_SHADER, "", VERTEX_SHADER_SRC)?;
    let fs = compile_shader(gl, glow::FRAGMENT_SHADER, fs_header, FRAGMENT_SHADER_SRC)?;
    let program = gl.create_program()?;
    gl.attach_shader(program, vs);
    gl.attach_shader(program, fs);

    gl.bind_attrib_location(program, 0, "a_pos");
    gl.bind_attrib_location(program, 1, "a_normal");
    gl.bind_attrib_location(program, 2, "a_color");
    gl.bind_attrib_location(program, 3, "a_ao");

    gl.link_program(program);
    gl.delete_shader(vs);
    gl.delete_shader(fs);
    if !gl.get_program_link_status(program) {
        let log = gl.get_program_info_log(program);
        gl.delete_program(program);
        return Err(log);
    }
    Ok(program)
}

pub struct GlSceneRenderer {
    program: glow::Program,
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    ebo: glow::Buffer,
}

impl GlSceneRenderer {
    pub fn new(gl: &glow::Context) -> Result<Self, String> {
        unsafe {
            let program = create_program(gl)?;
            let vao = gl.create_vertex_array()?;
            let vbo = gl.create_buffer()?;
            let ebo = gl.create_buffer()?;

            gl.bind_vertex_array(Some(vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ebo));

            let stride = std::mem::size_of::<GlVertex>() as i32;

            gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, stride, 0);
            gl.enable_vertex_attrib_array(0);

            gl.vertex_attrib_pointer_f32(1, 3, glow::FLOAT, false, stride, 12);
            gl.enable_vertex_attrib_array(1);

            gl.vertex_attrib_pointer_f32(2, 4, glow::FLOAT, false, stride, 24);
            gl.enable_vertex_attrib_array(2);

            gl.vertex_attrib_pointer_f32(3, 1, glow::FLOAT, false, stride, 40);
            gl.enable_vertex_attrib_array(3);

            gl.bind_vertex_array(None);
            Ok(Self { program, vao, vbo, ebo })
        }
    }

    pub fn paint(
        &mut self,
        gl: &glow::Context,
        mvp: &[f32; 16],
        eye_pos: [f32; 3],
        tri_vertices: &[GlVertex],
        tri_indices: &[u32],
        line_vertices: &[GlVertex],
        line_indices: &[u32],
    ) {
        unsafe {
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LEQUAL);
            gl.depth_mask(true);

            gl.clear(glow::DEPTH_BUFFER_BIT);

            gl.use_program(Some(self.program));

            if let Some(loc) = gl.get_uniform_location(self.program, "u_mvp") {
                gl.uniform_matrix_4_f32_slice(Some(&loc), false, mvp);
            }
            if let Some(loc) = gl.get_uniform_location(self.program, "u_light_key") {
                gl.uniform_3_f32(Some(&loc), 0.48, 0.72, 0.50);
            }
            if let Some(loc) = gl.get_uniform_location(self.program, "u_light_fill") {
                gl.uniform_3_f32(Some(&loc), -0.45, -0.30, -0.45);
            }
            if let Some(loc) = gl.get_uniform_location(self.program, "u_eye_pos") {
                gl.uniform_3_f32(Some(&loc), eye_pos[0], eye_pos[1], eye_pos[2]);
            }

            let u_is_line_loc = gl.get_uniform_location(self.program, "u_is_line");

            gl.bind_vertex_array(Some(self.vao));

            // 1. Отрисовка сплошной 3D-геометрии (манекен, суставы, датчики)
            if !tri_indices.is_empty() {
                if let Some(ref loc) = u_is_line_loc {
                    gl.uniform_1_i32(Some(loc), 0);
                }

                gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.vbo));
                let v_bytes = std::slice::from_raw_parts(
                    tri_vertices.as_ptr().cast::<u8>(),
                    tri_vertices.len() * std::mem::size_of::<GlVertex>(),
                );
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, v_bytes, glow::DYNAMIC_DRAW);

                gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(self.ebo));
                let i_bytes = std::slice::from_raw_parts(
                    tri_indices.as_ptr().cast::<u8>(),
                    tri_indices.len() * std::mem::size_of::<u32>(),
                );
                gl.buffer_data_u8_slice(glow::ELEMENT_ARRAY_BUFFER, i_bytes, glow::DYNAMIC_DRAW);

                gl.enable(glow::CULL_FACE);
                gl.cull_face(glow::BACK);
                gl.front_face(glow::CCW);

                gl.draw_elements(glow::TRIANGLES, tri_indices.len() as i32, glow::UNSIGNED_INT, 0);

                gl.disable(glow::CULL_FACE);
            }

            // 2. Отрисовка линий пола, объемного каркаса рабочей зоны и осей
            if !line_indices.is_empty() {
                if let Some(ref loc) = u_is_line_loc {
                    gl.uniform_1_i32(Some(loc), 1);
                }

                gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.vbo));
                let v_bytes = std::slice::from_raw_parts(
                    line_vertices.as_ptr().cast::<u8>(),
                    line_vertices.len() * std::mem::size_of::<GlVertex>(),
                );
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, v_bytes, glow::DYNAMIC_DRAW);

                gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(self.ebo));
                let i_bytes = std::slice::from_raw_parts(
                    line_indices.as_ptr().cast::<u8>(),
                    line_indices.len() * std::mem::size_of::<u32>(),
                );
                gl.buffer_data_u8_slice(glow::ELEMENT_ARRAY_BUFFER, i_bytes, glow::DYNAMIC_DRAW);

                gl.draw_elements(glow::LINES, line_indices.len() as i32, glow::UNSIGNED_INT, 0);
            }

            gl.bind_vertex_array(None);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, None);
            gl.use_program(None);
            gl.disable(glow::DEPTH_TEST);
        }
    }
}

static GL_RENDERER: OnceLock<Mutex<GlSceneRenderer>> = OnceLock::new();

fn tone_rgba(t: Tone) -> [f32; 4] {
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

fn add_capsule_geo(
    v_out: &mut Vec<GlVertex>,
    i_out: &mut Vec<u32>,
    a: V3,
    b: V3,
    ra: f32,
    rb: f32,
    tone: Tone,
) {
    let axis = sub(b, a);
    let len = super::vec3::len(axis);
    if len < 1e-4 {
        add_sphere_geo(v_out, i_out, a, ra.max(rb), tone);
        return;
    }

    let w = scale(axis, 1.0 / len);
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
        let sin_p = phi.sin();
        let cos_p = phi.cos();
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

    let tan_alpha = (ra - rb) / len;
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
        let sin_p = phi.sin();
        let cos_p = phi.cos();
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
        let r1_k = base_idx + 1 + k as u32;
        let r1_nk = base_idx + 1 + nk as u32;
        i_out.extend([base_idx, r1_nk, r1_k]);
    }

    let total_rings = DOME_STACKS + 1 + (DOME_STACKS - 1);

    for ring in 0..(total_rings - 1) {
        let r0_start = base_idx + 1 + (ring * SECTORS) as u32;
        let r1_start = base_idx + 1 + ((ring + 1) * SECTORS) as u32;

        for k in 0..SECTORS {
            let nk = (k + 1) % SECTORS;

            let a0 = r0_start + k as u32;
            let a1 = r0_start + nk as u32;
            let b0 = r1_start + k as u32;
            let b1 = r1_start + nk as u32;

            i_out.extend([a0, a1, b0]);
            i_out.extend([a1, b1, b0]);
        }
    }

    let last_ring_start = base_idx + 1 + ((total_rings - 1) * SECTORS) as u32;
    for k in 0..SECTORS {
        let nk = (k + 1) % SECTORS;
        let r_k = last_ring_start + k as u32;
        let r_nk = last_ring_start + nk as u32;
        i_out.extend([pole_b_idx, r_k, r_nk]);
    }
}

fn add_sphere_geo(
    v_out: &mut Vec<GlVertex>,
    i_out: &mut Vec<u32>,
    c: V3,
    r: f32,
    tone: Tone,
) {
    const SLICES: usize = 20;
    const STACKS: usize = 10;
    let color = tone_rgba(tone);
    let base_idx = v_out.len() as u32;

    let pole_s = [c[0], c[1] - r, c[2]];
    v_out.push(GlVertex { pos: pole_s, normal: [0.0, -1.0, 0.0], color, ao: 1.0 });

    for s in 1..STACKS {
        let lat = -std::f32::consts::FRAC_PI_2 + (s as f32) * (std::f32::consts::PI / STACKS as f32);
        let y = r * lat.sin();
        let r_lat = r * lat.cos();

        for k in 0..SLICES {
            let lon = (k as f32) * (std::f32::consts::TAU / SLICES as f32);
            let radial = [r_lat * lon.cos(), y, r_lat * lon.sin()];
            v_out.push(GlVertex {
                pos: add(c, radial),
                normal: norm(radial),
                color,
                ao: 1.0,
            });
        }
    }

    let pole_n = [c[0], c[1] + r, c[2]];
    let pole_n_idx = v_out.len() as u32;
    v_out.push(GlVertex { pos: pole_n, normal: [0.0, 1.0, 0.0], color, ao: 1.0 });

    for k in 0..SLICES {
        let nk = (k + 1) % SLICES;
        let r1_k = base_idx + 1 + k as u32;
        let r1_nk = base_idx + 1 + nk as u32;
        i_out.extend([base_idx, r1_nk, r1_k]);
    }

    for s in 0..(STACKS - 2) {
        let r0_start = base_idx + 1 + (s * SLICES) as u32;
        let r1_start = base_idx + 1 + ((s + 1) * SLICES) as u32;

        for k in 0..SLICES {
            let nk = (k + 1) % SLICES;

            let a0 = r0_start + k as u32;
            let a1 = r0_start + nk as u32;
            let b0 = r1_start + k as u32;
            let b1 = r1_start + nk as u32;

            i_out.extend([a0, a1, b0]);
            i_out.extend([a1, b1, b0]);
        }
    }

    let last_ring_start = base_idx + 1 + ((STACKS - 2) * SLICES) as u32;
    for k in 0..SLICES {
        let nk = (k + 1) % SLICES;
        let r_k = last_ring_start + k as u32;
        let r_nk = last_ring_start + nk as u32;
        i_out.extend([pole_n_idx, r_k, r_nk]);
    }
}

pub fn draw_scene_3d(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    skel: &Skeleton,
    chips: &[PlacedChip],
    quats: &[Option<[f32; 4]>],
) {
    let mut tri_vertices: Vec<GlVertex> = Vec::with_capacity(8192);
    let mut tri_indices: Vec<u32> = Vec::with_capacity(16384);

    let mut line_vertices: Vec<GlVertex> = Vec::with_capacity(1024);
    let mut line_indices: Vec<u32> = Vec::with_capacity(1024);

    // 1. Геометрия тела манекена
    for pr in &skel.prims {
        match pr {
            Prim::Capsule { a, b, ra, rb, tone } => {
                add_capsule_geo(&mut tri_vertices, &mut tri_indices, *a, *b, *ra, *rb, *tone);
            }
            Prim::Sphere { c, r, tone } => {
                add_sphere_geo(&mut tri_vertices, &mut tri_indices, *c, *r, *tone);
            }
            Prim::Tri { p, tone } => {
                let edge1 = sub(p[1], p[0]);
                let edge2 = sub(p[2], p[0]);
                let normal = norm(cross(edge1, edge2));
                let color = tone_rgba(*tone);
                let base = tri_vertices.len() as u32;
                for &pos in p {
                    tri_vertices.push(GlVertex { pos, normal, color, ao: 1.0 });
                }
                tri_indices.extend([base, base + 1, base + 2]);
            }
            Prim::Mesh { vertices, normals, indices, tone } => {
                let color = tone_rgba(*tone);
                let base = tri_vertices.len() as u32;
                for i in 0..vertices.len() {
                    tri_vertices.push(GlVertex {
                        pos: vertices[i],
                        normal: normals[i],
                        color,
                        ao: 1.0,
                    });
                }
                for &idx in indices {
                    tri_indices.push(base + idx);
                }
            }
        }
    }

    // 2. Геометрия суставов
    for j in &skel.joints {
        let r = if j.marker_r > 0.010 { 0.007 } else { 0.005 };
        add_sphere_geo(&mut tri_vertices, &mut tri_indices, j.pos, r, Tone::Dark);
    }

    // 3. Метки датчиков: двусторонняя сетка 3×3, видимая с любых сторон
    let highlighted = placement::highlight();
    for (st, q) in chips.iter().zip(quats) {
        let color = if highlighted == Some(st.id) {
            [0.45, 0.85, 1.0, 1.0]
        } else if q.is_some() {
            [0.15, 0.95, 0.35, 1.0]
        } else {
            [1.0, 0.84, 0.12, 1.0]
        };
        let base = tri_vertices.len() as u32;
        for k in 0..9 {
            tri_vertices.push(GlVertex { pos: st.patch[k], normal: st.patch_n[k], color, ao: 1.0 });
        }
        for row in 0..2u32 {
            for col in 0..2u32 {
                let v00 = base + row * 3 + col;
                let (v01, v10, v11) = (v00 + 1, v00 + 3, v00 + 4);
                // Лицевая сторона (исправленный CCW обход)
                tri_indices.extend([v00, v11, v01, v00, v10, v11]);
                // Обратная сторона (для двусторонней видимости без отсечения)
                tri_indices.extend([v00, v01, v11, v00, v11, v10]);
            }
        }
    }

    // 4. Линии сетки горизонта (Y = 0) и объемные пространственные границы рабочей зоны (11.25x11.25x5.78 м)
    let half = 5.625_f32;
    let height = 5.775_f32;
    let step = 0.75_f32;

    let mut add_line = |p1: [f32; 3], p2: [f32; 3], color: [f32; 4]| {
        let b = line_vertices.len() as u32;
        line_vertices.push(GlVertex { pos: p1, normal: [0.0, 1.0, 0.0], color, ao: 1.0 });
        line_vertices.push(GlVertex { pos: p2, normal: [0.0, 1.0, 0.0], color, ao: 1.0 });
        line_indices.extend([b, b + 1]);
    };

    let col_grid = [0.20, 0.25, 0.35, 0.65];
    let col_x = [0.75, 0.20, 0.20, 0.85];
    let col_z = [0.20, 0.45, 0.85, 0.85];
    let col_perimeter = [0.28, 0.62, 0.92, 0.85];
    let col_cage_vert = [0.22, 0.48, 0.75, 0.45];
    let col_cage_top = [0.18, 0.36, 0.58, 0.35];

    // Симметричный обход сетки от центра к краям (осевые линии x=0 и z=0 выделены)
    let num_steps = (half / step).floor() as i32;
    for i in -num_steps..=num_steps {
        let x = i as f32 * step;
        let col = if i == 0 { col_z } else { col_grid };
        add_line([x, 0.0, -half], [x, 0.0, half], col);

        let z = i as f32 * step;
        let col_row = if i == 0 { col_x } else { col_grid };
        add_line([-half, 0.0, z], [half, 0.0, z], col_row);
    }

    // Внешняя контурная рамка пола
    add_line([-half, 0.0, -half], [half, 0.0, -half], col_perimeter);
    add_line([half, 0.0, -half], [half, 0.0, half], col_perimeter);
    add_line([half, 0.0, half], [-half, 0.0, half], col_perimeter);
    add_line([-half, 0.0, half], [-half, 0.0, -half], col_perimeter);

    // 4 угловых вертикальных пилона рабочей зоны (Y: 0.0 -> 5.78)
    add_line([-half, 0.0, -half], [-half, height, -half], col_cage_vert);
    add_line([half, 0.0, -half], [half, height, -half], col_cage_vert);
    add_line([-half, 0.0, half], [-half, height, half], col_cage_vert);
    add_line([half, 0.0, half], [half, height, half], col_cage_vert);

    // Верхняя рамка периметра рабочей зоны (Y = 5.78)
    add_line([-half, height, -half], [half, height, -half], col_cage_top);
    add_line([half, height, -half], [half, height, half], col_cage_top);
    add_line([half, height, half], [-half, height, half], col_cage_top);
    add_line([-half, height, half], [-half, height, -half], col_cage_top);

    // 5. Оси ориентации активных датчиков
    for (st, q) in chips.iter().zip(quats) {
        if let Some(quat) = q {
            let axis_len = st.chip.axis_len();
            let axes = [
                ([axis_len, 0.0, 0.0], [0.95, 0.20, 0.20, 1.0]),
                ([0.0, axis_len, 0.0], [0.20, 0.90, 0.30, 1.0]),
                ([0.0, 0.0, axis_len], [0.20, 0.50, 1.0, 1.0]),
            ];
            for (local, col) in axes {
                let rot = rotate_vector_by_quat(local, *quat);
                let tip = add(st.pos, rot);
                add_line(st.pos, tip, col);
            }
        }
    }

    let aspect = rect.width() / rect.height().max(1.0);
    let mvp = camera.calc_mvp(aspect);
    let eye_pos = camera.eye_position();

    let callback = egui::PaintCallback {
        rect,
        callback: Arc::new(egui_glow::CallbackFn::new(move |_info, painter_gl| {
            let gl = painter_gl.gl();
            let renderer_lock = GL_RENDERER.get_or_init(|| {
                Mutex::new(GlSceneRenderer::new(gl).expect("Сбой инициализации 3D GL-рендерера"))
            });
            if let Ok(mut renderer) = renderer_lock.lock() {
                renderer.paint(
                    gl,
                    &mvp,
                    eye_pos,
                    &tri_vertices,
                    &tri_indices,
                    &line_vertices,
                    &line_indices,
                );
            }
        })),
    };

    painter.add(callback);
}