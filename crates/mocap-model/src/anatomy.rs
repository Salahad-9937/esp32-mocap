//! # Загрузчик эталонной 3D-модели манекена и координатор анатомии (`anatomy`)

pub mod loader;
pub mod solver;
pub mod specs;

use crate::skeleton::Skeleton;

/// Выполняет сборку скелета манекена, чтение мешей и калибровку сенсорных позиций.
#[must_use]
pub fn build() -> Skeleton {
    let (prims, prim_bones, mut sites, joints) = loader::load_raw_model();
    specs::apply_anatomical_specs(&prims, &prim_bones, &mut sites);

    Skeleton {
        joints,
        prims,
        prim_bones,
        sites,
    }
}