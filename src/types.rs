#[cfg(feature="dim3")]
pub mod dim3{
    use crate::nalgebra::RealField;
    use crate::nalgebra;
    use crate::nphysics;

    pub type Vector<N> = nalgebra::Vector3<N>;
    pub type Isometry<N> = nalgebra::Isometry3<N>;
    pub type Point<N> = nalgebra::Point3<N>;

    pub type AngularInertiaTensor<N> = nalgebra::Matrix3<N>;
    pub type Force<N> = nphysics::algebra::Force3<N>;
    pub type Velocity<N> = nphysics::algebra::Velocity3<N>;

    #[inline]
    pub fn angular_inertia_zero<N: RealField>() -> AngularInertiaTensor<N>{
        AngularInertiaTensor::<N>::zeros()
    }

    #[inline]
    pub fn to_isometry3<N: RealField>(value: &Isometry<N>) -> nalgebra::Isometry3<N>{
        value.clone()
    }

    #[inline]
    pub fn from_isometry3<N: RealField>(value: &nalgebra::Isometry3<N>) -> Isometry<N>{
        value.clone()
    }
}

#[cfg(feature="dim3")]
pub use dim3::*;

#[cfg(feature="dim2")]
pub mod dim2{
    use crate::nalgebra::RealField;
    use crate::nalgebra;
    use crate::nphysics;

    pub type Vector<N> = nalgebra::Vector2<N>;
    pub type Isometry<N> = nalgebra::Isometry2<N>;
    pub type Point<N> = nalgebra::Point2<N>;

    pub type AngularInertiaTensor<N> = N;
    pub type Force<N> = nphysics::algebra::Force2<N>;
    pub type Velocity<N> = nphysics::algebra::Velocity2<N>;

    #[inline]
    pub fn angular_inertia_zero<N: RealField>() -> AngularInertiaTensor<N>{
        N::zero()
    }

    #[inline]
    pub fn to_isometry3<N: RealField>(value: &Isometry<N>) -> nalgebra::Isometry3<N>{
        // TODO
        unimplemented!()
    }

    #[inline]
    pub fn from_isometry3<N: RealField>(value: &nalgebra::Isometry3<N>) -> Isometry<N>{
        // TODO
        unimplemented!()
    }
}


#[cfg(feature="dim2")]
pub use dim2::*;

