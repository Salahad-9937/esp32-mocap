//! # Конвейер OpenGL шейдеров и вертексных буферов Glow (`pipeline`)

use eframe::egui_glow;
use eframe::glow::{self, HasContext};

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
        float fill = max(dot(n, lf), 0.0) * 0.20;

        vec3 eye_dir = normalize(u_eye_pos - a_pos);
        float eye_facing = max(dot(n, eye_dir), 0.0);
        float n_dot_v = clamp(1.0 - eye_facing, 0.0, 1.0);
        float rim = pow(n_dot_v, 4.0) * 0.12;

        vec3 warm = vec3(1.04, 1.02, 0.98);
        vec3 cool = vec3(0.92, 0.95, 1.05);
        vec3 base = a_color.rgb;
        vec3 blended = base * mix(cool, warm, key);

        float ambient = 0.20;
        vec3 lit = (blended * (ambient + key * 0.55 + fill) * a_ao) + vec3(rim);

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
    let fs_header = if shader_version.is_embedded() { "precision mediump float;\n" } else { "" };

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

            if !tri_indices.is_empty() {
                if let Some(ref loc) = u_is_line_loc { gl.uniform_1_i32(Some(loc), 0); }

                gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.vbo));
                let v_bytes = std::slice::from_raw_parts(tri_vertices.as_ptr().cast::<u8>(), tri_vertices.len() * std::mem::size_of::<GlVertex>());
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, v_bytes, glow::DYNAMIC_DRAW);

                gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(self.ebo));
                let i_bytes = std::slice::from_raw_parts(tri_indices.as_ptr().cast::<u8>(), tri_indices.len() * std::mem::size_of::<u32>());
                gl.buffer_data_u8_slice(glow::ELEMENT_ARRAY_BUFFER, i_bytes, glow::DYNAMIC_DRAW);

                gl.enable(glow::CULL_FACE);
                gl.cull_face(glow::BACK);
                gl.front_face(glow::CCW);
                gl.draw_elements(glow::TRIANGLES, tri_indices.len() as i32, glow::UNSIGNED_INT, 0);
                gl.disable(glow::CULL_FACE);
            }

            if !line_indices.is_empty() {
                if let Some(ref loc) = u_is_line_loc { gl.uniform_1_i32(Some(loc), 1); }

                gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.vbo));
                let v_bytes = std::slice::from_raw_parts(line_vertices.as_ptr().cast::<u8>(), line_vertices.len() * std::mem::size_of::<GlVertex>());
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, v_bytes, glow::DYNAMIC_DRAW);

                gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(self.ebo));
                let i_bytes = std::slice::from_raw_parts(line_indices.as_ptr().cast::<u8>(), line_indices.len() * std::mem::size_of::<u32>());
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