//! # Анатомический скелет, суставы и реестр 64 сенсорных позиций (`skeleton`)

use super::anatomy;
use super::math::V3;
use std::sync::OnceLock;

/// Полное количество аппаратно поддерживаемых сенсорных каналов
pub const SENSOR_COUNT: usize = 64;
/// Возвышение сенсорной площадки над поверхностью кости (метры)
pub const STANDOFF: f32 = 0.004;

/// Анатомическая сторона тела
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    /// Левая сторона тела
    Left,
    /// Правая сторона тела
    Right,
}

impl Side {
    /// Знак направления по оси X (-1.0 для правой, 1.0 для левой)
    #[must_use]
    pub const fn sign(self) -> f32 {
        match self {
            Side::Right => -1.0,
            Side::Left => 1.0,
        }
    }

    /// Префикс строкового идентификатора сустава или сенсора
    #[must_use]
    pub const fn prefix(self) -> &'static str {
        match self {
            Side::Right => "R.",
            Side::Left => "L.",
        }
    }
}

/// Биомеханический тип анатомического сочленения
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointKind {
    /// Корневой сустав (таз)
    Root,
    /// Шаровидный сустав с тремя вращательными степенями свободы
    Ball,
    /// Одноосевой цилиндрический шарнир (блоковидный)
    Hinge,
    /// Вращательный цилиндрический сустав вокруг оси кости
    Pivot,
    /// Двухосевой седловидный сустав
    Saddle,
    /// Мыщелковый эллипсоидный сустав
    Condyloid,
    /// Плоский скользящий сустав
    Gliding,
    /// Межпозвонковый диск
    Spinal,
}

/// Материал анатомического сегмента для визуального оттенения
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tone {
    /// Кортикальная кость
    Cortical,
    /// Трабекулярная губчатая кость
    Trabecular,
    /// Костная пластина
    Plate,
    /// Хрящевая ткань
    Cartilage,
    /// Зубная ткань
    Tooth,
    /// Темная контрастная зона
    Dark,
}

/// Геометрический примитив для построения костей
#[derive(Debug, Clone)]
pub enum Prim {
    /// Сфероцилиндрическая капсула с радиусами `ra` и `rb` на концах
    Capsule { a: V3, b: V3, ra: f32, rb: f32, tone: Tone },
    /// Сфера с центром `c` и радиусом `r`
    Sphere { c: V3, r: f32, tone: Tone },
    /// Одиночный треугольник
    Tri { p: [V3; 3], tone: Tone },
    /// Произвольный триангулированный меш с нормалями
    Mesh {
        vertices: Vec<V3>,
        normals: Vec<V3>,
        indices: Vec<u32>,
        tone: Tone,
    },
}

/// Анатомический сустав кинематического дерева
#[derive(Debug, Clone)]
pub struct Joint {
    /// Название сочленения
    pub name: String,
    /// Тип кинематической пары
    pub kind: JointKind,
    /// 3D-координаты центра вращения
    pub pos: V3,
    /// Индекс родительского сочленения
    pub parent: Option<usize>,
    /// Основная ось вращения шарнира
    pub axis: Option<V3>,
    /// Радиус суставного маркера
    pub marker_r: f32,
}

/// Габаритный типоразмер сенсорной платы
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    /// Большая плата (~28 мм) для торса и бедер
    Large,
    /// Средняя плата (~20 мм) для конечностей и головы
    Medium,
    /// Малая плата (~11 мм) для фаланг пальцев
    Small,
}

impl Chip {
    /// Половина стороны сенсорной платы в метрах
    #[must_use]
    pub const fn half(self) -> f32 {
        match self {
            Chip::Large => 0.014,
            Chip::Medium => 0.010,
            Chip::Small => 0.0055,
        }
    }

    /// Длина визуальных осей ориентации в метрах
    #[must_use]
    pub const fn axis_len(self) -> f32 {
        match self {
            Chip::Large => 0.10,
            Chip::Medium => 0.065,
            Chip::Small => 0.024,
        }
    }
}

/// Способ геометрической привязки сенсора к телу
#[derive(Debug, Clone, Copy)]
pub enum Anchor {
    /// На сегменте кости (имя сегмента, параметр длины 0..1, угол в градусах)
    Seg(&'static str, f32, f32),
    /// На остистом отростке позвонка
    Spinous(&'static str),
    /// Лобная кость
    Forehead,
    /// Грудина
    Sternum,
    /// Крестец
    Sacrum,
}

/// Статическое описание установки сенсора
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

use Anchor::{Forehead, Sacrum, Seg, Spinous, Sternum};
use Chip::{Large as L, Medium as M, Small as S};
use Side::{Left as Lt, Right as Rt};

/// Реестр всех 64 сенсорных позиций костюма захвата движений
pub const SENSOR_DEFS: [SensorDef; SENSOR_COUNT] = [
    d("Head", Rt, Forehead, M),
    d("Neck", Rt, Spinous("C5"), M),
    d("SpineT3", Rt, Spinous("T3"), L),
    d("SpineT7", Rt, Spinous("T7"), L),
    d("SpineT11", Rt, Spinous("T11"), L),
    d("SpineL3", Rt, Spinous("L3"), L),
    d("Sacrum", Rt, Sacrum, L),
    d("Sternum", Rt, Sternum, L),
    d("LeftClavicle", Lt, Seg("Clavicle", 0.5, 0.0), M),
    d("RightClavicle", Rt, Seg("Clavicle", 0.5, 0.0), M),
    d("LeftShoulder", Lt, Seg("Acromion", 0.85, 0.0), M),
    d("RightShoulder", Rt, Seg("Acromion", 0.85, 0.0), M),
    d("LeftUpperArmProx", Lt, Seg("UpperArm", 0.30, 100.0), M),
    d("LeftUpperArmDist", Lt, Seg("UpperArm", 0.72, 100.0), M),
    d("LeftForearmProx", Lt, Seg("Ulna", 0.30, 110.0), M),
    d("LeftForearmDist", Lt, Seg("Radius", 0.72, 80.0), M),
    d("LeftHand", Lt, Seg("MC3", 0.50, 0.0), M),
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
    d("RightUpperArmProx", Rt, Seg("UpperArm", 0.30, 100.0), M),
    d("RightUpperArmDist", Rt, Seg("UpperArm", 0.72, 100.0), M),
    d("RightForearmProx", Rt, Seg("Ulna", 0.30, 110.0), M),
    d("RightForearmDist", Rt, Seg("Radius", 0.72, 80.0), M),
    d("RightHand", Rt, Seg("MC3", 0.50, 0.0), M),
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
    d("LeftThighProx", Lt, Seg("Femur", 0.30, 90.0), L),
    d("LeftThighDist", Lt, Seg("Femur", 0.72, 90.0), L),
    d("LeftShankProx", Lt, Seg("Tibia", 0.28, -45.0), L),
    d("LeftShankDist", Lt, Seg("Tibia", 0.70, -45.0), L),
    d("LeftFoot", Lt, Seg("MT3", 0.50, 0.0), M),
    d("LeftToe", Lt, Seg("HalluxPP", 0.50, 0.0), S),
    d("RightThighProx", Rt, Seg("Femur", 0.30, 90.0), L),
    d("RightThighDist", Rt, Seg("Femur", 0.72, 90.0), L),
    d("RightShankProx", Rt, Seg("Tibia", 0.28, -45.0), L),
    d("RightShankDist", Rt, Seg("Tibia", 0.70, -45.0), L),
    d("RightFoot", Rt, Seg("MT3", 0.50, 0.0), M),
    d("RightToe", Rt, Seg("HalluxPP", 0.50, 0.0), S),
];

/// Имена целевых сегментов маппинга в каноническом порядке
pub const TARGET_BONES: [&str; SENSOR_COUNT] = {
    let mut out = [""; SENSOR_COUNT];
    let mut i = 0;
    while i < SENSOR_COUNT {
        out[i] = SENSOR_DEFS[i].tag;
        i += 1;
    }
    out
};

/// Рассчитанная сенсорная площадка на поверхности тела
#[derive(Debug, Clone)]
pub struct SensorSite {
    pub id: usize,
    pub tag: &'static str,
    pub side: Side,
    pub pos: V3,
    pub normal: V3,
    pub tangent: V3,
    pub support: V3,
    pub chip: Chip,
}

/// Единый контейнер биомеханического скелета
pub struct Skeleton {
    pub joints: Vec<Joint>,
    pub prims: Vec<Prim>,
    pub sites: Vec<SensorSite>,
}

/// Инициализирует и возвращает разделяемую ссылку на неизменяемый анатомический скелет.
#[must_use]
pub fn skeleton() -> &'static Skeleton {
    static SKELETON: OnceLock<Skeleton> = OnceLock::new();
    SKELETON.get_or_init(anatomy::build)
}