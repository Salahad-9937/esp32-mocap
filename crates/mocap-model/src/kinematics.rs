//! # Процессор прямой кинематики анатомического скелета (`kinematics`)

use super::math::{add, cross, norm, scale, sub, V3};
use std::collections::HashMap;
use std::sync::OnceLock;

#[inline]
#[must_use]
pub fn rotate_vec_by_quat(v: V3, q: [f32; 4]) -> V3 {
    let q_xyz = [q[1], q[2], q[3]];
    let t = scale(cross(q_xyz, v), 2.0);
    add(v, add(scale(t, q[0]), cross(q_xyz, t)))
}

/// Узел сегмента скелета с явным анатомическим суставом вращения.
#[derive(Debug, Clone)]
pub struct SegmentBone {
    pub tag: &'static str,
    pub parent_tag: Option<&'static str>,
    pub rest_joint: V3,
}

/// Вычисленная мировая пространственная трансформация сегмента.
#[derive(Debug, Clone, Copy)]
pub struct BoneTransform {
    pub world_joint: V3,
    pub world_rot: [f32; 4],
    pub rest_joint: V3,
}

impl BoneTransform {
    #[inline]
    #[must_use]
    pub fn transform_point(&self, p: V3) -> V3 {
        let local_p = sub(p, self.rest_joint);
        add(self.world_joint, rotate_vec_by_quat(local_p, self.world_rot))
    }

    #[inline]
    #[must_use]
    pub fn transform_normal(&self, n: V3) -> V3 {
        norm(rotate_vec_by_quat(n, self.world_rot))
    }
}

pub struct KinematicTree {
    pub bones: Vec<SegmentBone>,
    pub tag_indices: HashMap<&'static str, usize>,
}

impl KinematicTree {
    #[must_use]
    pub fn build() -> Self {
        let s_l = 1.0_f32;
        let s_r = -1.0_f32;

        let bones = vec![
            // ── 1. Осевой скелет (8) ────────────────────────────────────────
            SegmentBone { tag: "Sacrum", parent_tag: None, rest_joint: [0.0, 0.94, -0.015] },
            SegmentBone { tag: "SpineL3", parent_tag: Some("Sacrum"), rest_joint: [0.0, 0.99, -0.020] },
            SegmentBone { tag: "SpineT11", parent_tag: Some("SpineL3"), rest_joint: [0.0, 1.08, -0.020] },
            SegmentBone { tag: "SpineT7", parent_tag: Some("SpineT11"), rest_joint: [0.0, 1.18, -0.020] },
            SegmentBone { tag: "SpineT3", parent_tag: Some("SpineT7"), rest_joint: [0.0, 1.32, -0.020] },
            SegmentBone { tag: "Sternum", parent_tag: Some("SpineT3"), rest_joint: [0.0, 1.36, 0.080] },
            SegmentBone { tag: "Neck", parent_tag: Some("SpineT3"), rest_joint: [0.0, 1.46, -0.015] },
            SegmentBone { tag: "Head", parent_tag: Some("Neck"), rest_joint: [0.0, 1.61, 0.010] },

            // ── 2. Левая рука и пальцы (22) ─────────────────────────────────
            SegmentBone { tag: "LeftClavicle", parent_tag: Some("SpineT3"), rest_joint: [s_l * 0.038, 1.436, 0.014] },
            SegmentBone { tag: "LeftShoulder", parent_tag: Some("LeftClavicle"), rest_joint: [s_l * 0.168, 1.408, -0.014] },
            SegmentBone { tag: "LeftUpperArmProx", parent_tag: Some("LeftShoulder"), rest_joint: [s_l * 0.178, 1.390, -0.012] },
            SegmentBone { tag: "LeftUpperArmDist", parent_tag: Some("LeftUpperArmProx"), rest_joint: [s_l * 0.190, 1.220, -0.012] },
            SegmentBone { tag: "LeftForearmProx", parent_tag: Some("LeftUpperArmProx"), rest_joint: [s_l * 0.198, 1.108, -0.012] },
            SegmentBone { tag: "LeftForearmDist", parent_tag: Some("LeftForearmProx"), rest_joint: [s_l * 0.209, 0.980, -0.009] },
            SegmentBone { tag: "LeftHand", parent_tag: Some("LeftForearmProx"), rest_joint: [s_l * 0.216, 0.845, -0.006] },
            SegmentBone { tag: "LeftThumbMeta", parent_tag: Some("LeftHand"), rest_joint: [s_l * 0.211, 0.823, 0.008] },
            SegmentBone { tag: "LeftThumbProx", parent_tag: Some("LeftThumbMeta"), rest_joint: [s_l * 0.208, 0.803, 0.015] },
            SegmentBone { tag: "LeftThumbDist", parent_tag: Some("LeftThumbProx"), rest_joint: [s_l * 0.205, 0.782, 0.029] },
            SegmentBone { tag: "LeftIndexProx", parent_tag: Some("LeftHand"), rest_joint: [s_l * 0.216, 0.782, 0.007] },
            SegmentBone { tag: "LeftIndexMid", parent_tag: Some("LeftIndexProx"), rest_joint: [s_l * 0.213, 0.753, 0.008] },
            SegmentBone { tag: "LeftIndexDist", parent_tag: Some("LeftIndexMid"), rest_joint: [s_l * 0.210, 0.735, 0.009] },
            SegmentBone { tag: "LeftMiddleProx", parent_tag: Some("LeftHand"), rest_joint: [s_l * 0.217, 0.778, -0.002] },
            SegmentBone { tag: "LeftMiddleMid", parent_tag: Some("LeftMiddleProx"), rest_joint: [s_l * 0.214, 0.745, -0.002] },
            SegmentBone { tag: "LeftMiddleDist", parent_tag: Some("LeftMiddleMid"), rest_joint: [s_l * 0.210, 0.724, -0.002] },
            SegmentBone { tag: "LeftRingProx", parent_tag: Some("LeftHand"), rest_joint: [s_l * 0.216, 0.781, -0.011] },
            SegmentBone { tag: "LeftRingMid", parent_tag: Some("LeftRingProx"), rest_joint: [s_l * 0.213, 0.751, -0.012] },
            SegmentBone { tag: "LeftRingDist", parent_tag: Some("LeftRingMid"), rest_joint: [s_l * 0.210, 0.732, -0.013] },
            SegmentBone { tag: "LeftPinkyProx", parent_tag: Some("LeftHand"), rest_joint: [s_l * 0.215, 0.786, -0.019] },
            SegmentBone { tag: "LeftPinkyMid", parent_tag: Some("LeftPinkyProx"), rest_joint: [s_l * 0.213, 0.764, -0.020] },
            SegmentBone { tag: "LeftPinkyDist", parent_tag: Some("LeftPinkyMid"), rest_joint: [s_l * 0.210, 0.750, -0.021] },

            // ── 3. Правая рука и пальцы (22) ────────────────────────────────
            SegmentBone { tag: "RightClavicle", parent_tag: Some("SpineT3"), rest_joint: [s_r * 0.038, 1.436, 0.014] },
            SegmentBone { tag: "RightShoulder", parent_tag: Some("RightClavicle"), rest_joint: [s_r * 0.168, 1.408, -0.014] },
            SegmentBone { tag: "RightUpperArmProx", parent_tag: Some("RightShoulder"), rest_joint: [s_r * 0.178, 1.390, -0.012] },
            SegmentBone { tag: "RightUpperArmDist", parent_tag: Some("RightUpperArmProx"), rest_joint: [s_r * 0.190, 1.220, -0.012] },
            SegmentBone { tag: "RightForearmProx", parent_tag: Some("RightUpperArmProx"), rest_joint: [s_r * 0.198, 1.108, -0.012] },
            SegmentBone { tag: "RightForearmDist", parent_tag: Some("RightForearmProx"), rest_joint: [s_r * 0.209, 0.980, -0.009] },
            SegmentBone { tag: "RightHand", parent_tag: Some("RightForearmProx"), rest_joint: [s_r * 0.216, 0.845, -0.006] },
            SegmentBone { tag: "RightThumbMeta", parent_tag: Some("RightHand"), rest_joint: [s_r * 0.211, 0.823, 0.008] },
            SegmentBone { tag: "RightThumbProx", parent_tag: Some("RightThumbMeta"), rest_joint: [s_r * 0.208, 0.803, 0.015] },
            SegmentBone { tag: "RightThumbDist", parent_tag: Some("RightThumbProx"), rest_joint: [s_r * 0.205, 0.782, 0.029] },
            SegmentBone { tag: "RightIndexProx", parent_tag: Some("RightHand"), rest_joint: [s_r * 0.216, 0.782, 0.007] },
            SegmentBone { tag: "RightIndexMid", parent_tag: Some("RightIndexProx"), rest_joint: [s_r * 0.213, 0.753, 0.008] },
            SegmentBone { tag: "RightIndexDist", parent_tag: Some("RightIndexMid"), rest_joint: [s_r * 0.210, 0.735, 0.009] },
            SegmentBone { tag: "RightMiddleProx", parent_tag: Some("RightHand"), rest_joint: [s_r * 0.217, 0.778, -0.002] },
            SegmentBone { tag: "RightMiddleMid", parent_tag: Some("RightMiddleProx"), rest_joint: [s_r * 0.214, 0.745, -0.002] },
            SegmentBone { tag: "RightMiddleDist", parent_tag: Some("RightMiddleMid"), rest_joint: [s_r * 0.210, 0.724, -0.002] },
            SegmentBone { tag: "RightRingProx", parent_tag: Some("RightHand"), rest_joint: [s_r * 0.216, 0.781, -0.011] },
            SegmentBone { tag: "RightRingMid", parent_tag: Some("RightRingProx"), rest_joint: [s_r * 0.213, 0.751, -0.012] },
            SegmentBone { tag: "RightRingDist", parent_tag: Some("RightRingMid"), rest_joint: [s_r * 0.210, 0.732, -0.013] },
            SegmentBone { tag: "RightPinkyProx", parent_tag: Some("RightHand"), rest_joint: [s_r * 0.215, 0.786, -0.019] },
            SegmentBone { tag: "RightPinkyMid", parent_tag: Some("RightPinkyProx"), rest_joint: [s_r * 0.213, 0.764, -0.020] },
            SegmentBone { tag: "RightPinkyDist", parent_tag: Some("RightPinkyMid"), rest_joint: [s_r * 0.210, 0.750, -0.021] },

            // ── 4. Левая нога (6) ───────────────────────────────────────────
            SegmentBone { tag: "LeftThighProx", parent_tag: Some("Sacrum"), rest_joint: [s_l * 0.096, 0.905, -0.005] },
            SegmentBone { tag: "LeftThighDist", parent_tag: Some("LeftThighProx"), rest_joint: [s_l * 0.089, 0.700, -0.002] },
            SegmentBone { tag: "LeftShankProx", parent_tag: Some("LeftThighProx"), rest_joint: [s_l * 0.082, 0.495, 0.000] },
            SegmentBone { tag: "LeftShankDist", parent_tag: Some("LeftShankProx"), rest_joint: [s_l * 0.082, 0.280, -0.002] },
            // Голеностопный сустав
            SegmentBone { tag: "LeftFoot", parent_tag: Some("LeftShankProx"), rest_joint: [s_l * 0.082, 0.056, 0.000] },
            // Сустав MTP1 (скошенная дуга переката по зеленой линии)
            SegmentBone { tag: "LeftToe", parent_tag: Some("LeftFoot"), rest_joint: [s_l * 0.067, 0.010, 0.114] },

            // ── 5. Правая нога (6) ──────────────────────────────────────────
            SegmentBone { tag: "RightThighProx", parent_tag: Some("Sacrum"), rest_joint: [s_r * 0.096, 0.905, -0.005] },
            SegmentBone { tag: "RightThighDist", parent_tag: Some("RightThighProx"), rest_joint: [s_r * 0.089, 0.700, -0.002] },
            SegmentBone { tag: "RightShankProx", parent_tag: Some("RightThighProx"), rest_joint: [s_r * 0.082, 0.495, 0.000] },
            SegmentBone { tag: "RightShankDist", parent_tag: Some("RightShankProx"), rest_joint: [s_r * 0.082, 0.280, -0.002] },
            // Голеностопный сустав
            SegmentBone { tag: "RightFoot", parent_tag: Some("RightShankProx"), rest_joint: [s_r * 0.082, 0.056, 0.000] },
            // Сустав MTP1 (скошенная дуга переката по зеленой линии)
            SegmentBone { tag: "RightToe", parent_tag: Some("RightFoot"), rest_joint: [s_r * 0.067, 0.010, 0.114] },
        ];

        let mut tag_indices = HashMap::new();
        for (i, b) in bones.iter().enumerate() {
            tag_indices.insert(b.tag, i);
        }

        Self { bones, tag_indices }
    }

    #[must_use]
    pub fn evaluate(&self, orientations: &HashMap<&str, [f32; 4]>) -> HashMap<&'static str, BoneTransform> {
        let mut transforms = HashMap::with_capacity(self.bones.len());
        for b in &self.bones {
            self.evaluate_bone(b.tag, orientations, &mut transforms);
        }
        transforms
    }

    fn evaluate_bone(
        &self,
        tag: &'static str,
        orientations: &HashMap<&str, [f32; 4]>,
        transforms: &mut HashMap<&'static str, BoneTransform>,
    ) -> BoneTransform {
        if let Some(&tf) = transforms.get(tag) {
            return tf;
        }

        let idx = self.tag_indices[tag];
        let b = &self.bones[idx];
        let local_measured_q = orientations.get(tag).copied();

        let tf = match b.parent_tag {
            None => {
                let world_rot = local_measured_q.unwrap_or([1.0, 0.0, 0.0, 0.0]);
                BoneTransform {
                    world_joint: b.rest_joint,
                    world_rot,
                    rest_joint: b.rest_joint,
                }
            }
            Some(p_tag) => {
                let p_tf = self.evaluate_bone(p_tag, orientations, transforms);
                let delta_rest = sub(b.rest_joint, p_tf.rest_joint);
                let delta_world = rotate_vec_by_quat(delta_rest, p_tf.world_rot);
                let world_joint = add(p_tf.world_joint, delta_world);
                let world_rot = local_measured_q.unwrap_or(p_tf.world_rot);

                BoneTransform {
                    world_joint,
                    world_rot,
                    rest_joint: b.rest_joint,
                }
            }
        };

        transforms.insert(tag, tf);
        tf
    }
}

#[must_use]
pub fn kinematic_tree() -> &'static KinematicTree {
    static TREE: OnceLock<KinematicTree> = OnceLock::new();
    TREE.get_or_init(KinematicTree::build)
}