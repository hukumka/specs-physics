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
    use nalgebra::{UnitQuaternion, UnitComplex, Quaternion, Vector2, Vector3};

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
        let pos = &value.translation.vector;
        let pos = Vector3::new(pos[0], pos[1], N::zero());

        let rot = &value.rotation;
        let rot = UnitQuaternion::new_unchecked(Quaternion::new(rot.cos_angle(), N::zero(), N::zero(), rot.sin_angle()));

        nalgebra::Isometry3::from_parts(pos.into(), rot)
    }

    #[inline]
    pub fn from_isometry3<N: RealField>(value: &nalgebra::Isometry3<N>) -> Isometry<N>{
        // TODO: create a way to allow arbitrary Z coordinate
        let pos = &value.translation.vector;
        assert_eq!(pos[2], N::zero());
        let pos = Vector2::new(pos[0], pos[1]);
        // TODO: Is panic correct behaviour for 2d object having rotations outside of XY plane?
        let rot = &value.rotation.as_ref().coords;
        assert_eq!((rot[1], rot[2]), (N::zero(), N::zero()), "2D objects must not have rotations outside of XY plane.");
        let rot = UnitComplex::from_cos_sin_unchecked(rot[0], rot[3]);
        nalgebra::Isometry::from_parts(pos.into(), rot)
    }
}


#[cfg(feature="dim2")]
pub use dim2::*;

