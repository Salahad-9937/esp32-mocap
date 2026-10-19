//! Модель скелета: суставы, кости и реестр ровно 64 мест установки датчиков.
//!
//! Геометрия строится один раз в [`super::anatomy`] (поза N, рост ≈ 1.75 м) и затем
//! только рисуется. Список датчиков [`SENSOR_DEFS`] — единственный источник истины:
//! из него же получается [`TARGET_BONES`] для вкладки маппинга.
//!
//! Система координат проекта: Y — вверх, Z — вперёд (лицо смотрит в +Z),
//! X — вправо от человека (левая сторона имеет отрицательный X).

use super::anatomy;
use super::vec3::V3;
use std::sync::OnceLock;

/// Сколько всего мест под датчики (предел платы: 8 мультиплексоров × 8 каналов).
pub const SENSOR_COUNT: usize = 64;

/// Зазор между поверхностью кости и платой датчика, м.
pub const STANDOFF: f32 = 0.004;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    #[must_use]
    pub const fn sign(self) -> f32 {
        match self {
            Side::Right => 1.0,
            Side::Left => -1.0,
        }
    }

    #[must_use]
    pub const fn prefix(self) -> &'static str {
        match self {
            Side::Right => "R.",
            Side::Left => "L.",
        }
    }
}

/// Тип сустава по биомеханической классификации (определяет вид маркера).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointKind {
    /// Корень скелета (таз), 6 степеней свободы.
    Root,
    /// Шаровидный: тазобедренный, плечевой (3 вращательных DoF).
    Ball,
    /// Блоковидный: локоть, колено, голеностоп, межфаланговые (1 DoF).
    Hinge,
    /// Цилиндрический: C1–C2, лучелоктевой (вращение вокруг оси кости).
    Pivot,
    /// Седловидный: грудино-ключичный, запястно-пястный большого пальца.
    Saddle,
    /// Мыщелковый: лучезапястный, пястно-фаланговые, атланто-затылочный (2 DoF).
    Condyloid,
    /// Плоский (скольжение): акромиально-ключичный, крестцово-подвздошный, подтаранный.
    Gliding,
    /// Межпозвонковый диск (малые вращения во всех осях).
    Spinal,
}

/// Материал кости — определяет цвет.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tone {
    Cortical,
    Trabecular,
    Plate,
    Cartilage,
    Tooth,
    Dark,
}

/// Графический примитив скелета (рисуется с сортировкой по глубине).
#[derive(Debug, Clone, Copy)]
pub enum Prim {
    Capsule { a: V3, b: V3, ra: f32, rb: f32, tone: Tone },
    Sphere { c: V3, r: f32, tone: Tone },
    Tri { p: [V3; 3], tone: Tone },
}

/// Сустав кинематического дерева. Поля `name`, `parent`, `axis` нужны для будущей
/// прямой кинематики (поворот костей по кватернионам датчиков).
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct Joint {
    pub name: String,
    pub kind: JointKind,
    pub pos: V3,
    pub parent: Option<usize>,
    /// Ось вращения для блоковидных суставов.
    pub axis: Option<V3>,
    /// Радиус маркера, м.
    pub marker_r: f32,
}

/// Размер платы датчика.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    /// Плата ~28 мм: туловище, бедро, голень.
    Large,
    /// Плата ~20 мм: голова, шея, плечо, предплечье, кисть, стопа.
    Medium,
    /// Плата ~11 мм: фаланги пальцев, большой палец стопы.
    Small,
}

impl Chip {
    /// Половина стороны платы, м.
    #[must_use]
    pub const fn half(self) -> f32 {
        match self {
            Chip::Large => 0.014,
            Chip::Medium => 0.010,
            Chip::Small => 0.0055,
        }
    }

    /// Длина осей-гизмо активного датчика, м.
    #[must_use]
    pub const fn axis_len(self) -> f32 {
        match self {
            Chip::Large => 0.10,
            Chip::Medium => 0.065,
            Chip::Small => 0.024,
        }
    }
}

/// Как найти место датчика на теле.
#[derive(Debug, Clone, Copy)]
pub enum Anchor {
    /// Точка на поверхности сегмента кости: имя сегмента (без префикса стороны),
    /// доля длины от проксимального конца `t` и угол вокруг оси `deg`
    /// (0° — «нулевая» нормаль сегмента, +90° — вторая нормаль; см. `Seg` в anatomy).
    Seg(&'static str, f32, f32),
    /// Кончик остистого отростка позвонка («C5», «T7», «L3»…).
    Spinous(&'static str),
    /// Лобная кость (налобная повязка).
    Forehead,
    /// Тело грудины.
    Sternum,
    /// Задняя поверхность крестца.
    Sacrum,
}

/// Описание одного из 64 датчиков.
#[derive(Debug, Clone, Copy)]
pub struct SensorDef {
    pub tag: &'static str,
    pub side: Side,
    pub anchor: Anchor,
    pub chip: Chip,
}

const fn d(tag: &'static str, side: Side, anchor: Anchor, chip: Chip) -> SensorDef {
    SensorDef { tag, side, anchor, chip }
}

use Anchor::{Forehead, Seg, Sacrum, Spinous, Sternum};
use Chip::{Large as L, Medium as M, Small as S};
use Side::{Left as Lt, Right as Rt};

/// 64 датчика. Принцип: один датчик на жёсткий сегмент; на длинных сегментах
/// (плечо, предплечье, бедро, голень) по два (проксимальный/дистальный) — для учёта
/// скручивания и смещения мягких тканей; пальцы — по датчику на каждую фалангу;
/// позвоночник — по датчику на каждый отдел.
///
/// Индексы: 0–7 ось, 8–11 плечевой пояс, 12–16 левая рука, 17–31 левые пальцы,
/// 32–36 правая рука, 37–51 правые пальцы, 52–57 левая нога, 58–63 правая нога.
pub const SENSOR_DEFS: [SensorDef; SENSOR_COUNT] = [
    // ── Голова и ось тела (8) ──────────────────────────────────────────────
    d("Head", Rt, Forehead, M),            // лобная кость
    d("Neck", Rt, Spinous("C5"), M),       // остистый отросток C5
    d("SpineT3", Rt, Spinous("T3"), L),    // верх грудного отдела
    d("SpineT7", Rt, Spinous("T7"), L),    // середина грудного отдела
    d("SpineT11", Rt, Spinous("T11"), L),  // низ грудного отдела
    d("SpineL3", Rt, Spinous("L3"), L),    // поясничный отдел
    d("Sacrum", Rt, Sacrum, L),            // таз (крестец)
    d("Sternum", Rt, Sternum, L),          // грудная клетка спереди
    // ── Плечевой пояс (4) ──────────────────────────────────────────────────
    d("LeftClavicle", Lt, Seg("Clavicle", 0.5, 0.0), M),
    d("RightClavicle", Rt, Seg("Clavicle", 0.5, 0.0), M),
    d("LeftShoulder", Lt, Seg("Acromion", 0.85, 0.0), M), // акромион / ость лопатки
    d("RightShoulder", Rt, Seg("Acromion", 0.85, 0.0), M),
    // ── Левая рука (5) ─────────────────────────────────────────────────────
    d("LeftUpperArmProx", Lt, Seg("UpperArm", 0.30, 100.0), M),
    d("LeftUpperArmDist", Lt, Seg("UpperArm", 0.72, 100.0), M),
    d("LeftForearmProx", Lt, Seg("Ulna", 0.30, 110.0), M),
    d("LeftForearmDist", Lt, Seg("Radius", 0.72, 80.0), M),
    d("LeftHand", Lt, Seg("MC3", 0.50, 0.0), M),         // тыл кисти
    // ── Левая кисть: 15 фаланг ─────────────────────────────────────────────
    d("LeftThumbMeta", Lt, Seg("ThumbMC", 0.5, 0.0), S),
    d("LeftThumbProx", Lt, Seg("ThumbPP", 0.5, 0.0), S),
    d("LeftThumbDist", Lt, Seg("ThumbDP", 0.5, 0.0), S),
    d("LeftIndexProx", Lt, Seg("IndexPP", 0.5, 0.0), S),
    d("LeftIndexMid", Lt, Seg("IndexMP", 0.5, 0.0), S),
    d("LeftIndexDist", Lt, Seg("IndexDP", 0.5, 0.0), S),
    d("LeftMiddleProx", Lt, Seg("MiddlePP", 0.5, 0.0), S),
    d("LeftMiddleMid", Lt, Seg("MiddleMP", 0.5, 0.0), S),
    d("LeftMiddleDist", Lt, Seg("MiddleDP", 0.5, 0.0), S),
    d("LeftRingProx", Lt, Seg("RingPP", 0.5, 0.0), S),
    d("LeftRingMid", Lt, Seg("RingMP", 0.5, 0.0), S),
    d("LeftRingDist", Lt, Seg("RingDP", 0.5, 0.0), S),
    d("LeftPinkyProx", Lt, Seg("PinkyPP", 0.5, 0.0), S),
    d("LeftPinkyMid", Lt, Seg("PinkyMP", 0.5, 0.0), S),
    d("LeftPinkyDist", Lt, Seg("PinkyDP", 0.5, 0.0), S),
    // ── Правая рука (5) ────────────────────────────────────────────────────
    d("RightUpperArmProx", Rt, Seg("UpperArm", 0.30, 100.0), M),
    d("RightUpperArmDist", Rt, Seg("UpperArm", 0.72, 100.0), M),
    d("RightForearmProx", Rt, Seg("Ulna", 0.30, 110.0), M),
    d("RightForearmDist", Rt, Seg("Radius", 0.72, 80.0), M),
    d("RightHand", Rt, Seg("MC3", 0.50, 0.0), M),
    // ── Правая кисть: 15 фаланг ────────────────────────────────────────────
    d("RightThumbMeta", Rt, Seg("ThumbMC", 0.5, 0.0), S),
    d("RightThumbProx", Rt, Seg("ThumbPP", 0.5, 0.0), S),
    d("RightThumbDist", Rt, Seg("ThumbDP", 0.5, 0.0), S),
    d("RightIndexProx", Rt, Seg("IndexPP", 0.5, 0.0), S),
    d("RightIndexMid", Rt, Seg("IndexMP", 0.5, 0.0), S),
    d("RightIndexDist", Rt, Seg("IndexDP", 0.5, 0.0), S),
    d("RightMiddleProx", Rt, Seg("MiddlePP", 0.5, 0.0), S),
    d("RightMiddleMid", Rt, Seg("MiddleMP", 0.5, 0.0), S),
    d("RightMiddleDist", Rt, Seg("MiddleDP", 0.5, 0.0), S),
    d("RightRingProx", Rt, Seg("RingPP", 0.5, 0.0), S),
    d("RightRingMid", Rt, Seg("RingMP", 0.5, 0.0), S),
    d("RightRingDist", Rt, Seg("RingDP", 0.5, 0.0), S),
    d("RightPinkyProx", Rt, Seg("PinkyPP", 0.5, 0.0), S),
    d("RightPinkyMid", Rt, Seg("PinkyMP", 0.5, 0.0), S),
    d("RightPinkyDist", Rt, Seg("PinkyDP", 0.5, 0.0), S),
    // ── Левая нога (6) ─────────────────────────────────────────────────────
    d("LeftThighProx", Lt, Seg("Femur", 0.30, 90.0), L),  // латеральная поверхность бедра
    d("LeftThighDist", Lt, Seg("Femur", 0.72, 90.0), L),
    d("LeftShankProx", Lt, Seg("Tibia", 0.28, -45.0), L), // переднемедиальная грань большеберцовой
    d("LeftShankDist", Lt, Seg("Tibia", 0.70, -45.0), L),
    d("LeftFoot", Lt, Seg("MT3", 0.50, 0.0), M),          // тыл стопы
    d("LeftToe", Lt, Seg("HalluxPP", 0.50, 0.0), S),      // большой палец стопы
    // ── Правая нога (6) ────────────────────────────────────────────────────
    d("RightThighProx", Rt, Seg("Femur", 0.30, 90.0), L),
    d("RightThighDist", Rt, Seg("Femur", 0.72, 90.0), L),
    d("RightShankProx", Rt, Seg("Tibia", 0.28, -45.0), L),
    d("RightShankDist", Rt, Seg("Tibia", 0.70, -45.0), L),
    d("RightFoot", Rt, Seg("MT3", 0.50, 0.0), M),
    d("RightToe", Rt, Seg("HalluxPP", 0.50, 0.0), S),
];

/// Имена 64 целевых сегментов в порядке `SENSOR_DEFS` (для вкладки маппинга).
pub const TARGET_BONES: [&str; SENSOR_COUNT] = {
    let mut out = [""; SENSOR_COUNT];
    let mut i = 0;
    while i < SENSOR_COUNT {
        out[i] = SENSOR_DEFS[i].tag;
        i += 1;
    }
    out
};

/// Рассчитанное место установки датчика на скелете.
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct SensorSite {
    pub id: usize,
    pub tag: &'static str,
    pub side: Side,
    /// Центр платы (на поверхности кости + `STANDOFF`).
    pub pos: V3,
    /// Внешняя нормаль поверхности (куда «смотрит» плата).
    pub normal: V3,
    /// Направление длинной стороны платы (вдоль кости).
    pub tangent: V3,
    /// Точка внутри опорной кости — для корректной сортировки по глубине.
    pub support: V3,
    pub chip: Chip,
}

/// Готовый скелет: суставы, примитивы для отрисовки и 64 места датчиков.
pub struct Skeleton {
    pub joints: Vec<Joint>,
    pub prims: Vec<Prim>,
    pub sites: Vec<SensorSite>,
}

/// Скелет строится один раз при первом обращении.
pub fn skeleton() -> &'static Skeleton {
    static SKELETON: OnceLock<Skeleton> = OnceLock::new();
    SKELETON.get_or_init(anatomy::build)
}
