
pub trait Vector {
    type V: Vector;

    fn to_slice(&self) -> &mut [f64];

    fn add(ours: &mut f64, theirs: f64);

    fn add_all(&self, other: f64) {
        for field in self.to_slice() {
            Self::add(field, other);
        }
    }

    fn vector_add(&self, other: &Self::V) {
        let theirs: &mut [f64] = other.to_slice();
        let mut i = 0;
        for field in self.to_slice() {
            Self::add(field, theirs[i]);
            i += 1
        }
    }

}

#[derive(Default)]
pub struct vect3{x:f64,y:f64,z:f64}
impl Vector for vect3 {
    type V = vect3;

    fn to_slice(&self) -> &mut [f64] {
        &mut [self.x,self.y,self.z]
    }
    fn add(ours: &mut f64, theirs: f64) {
        *ours += theirs;
    }
}
#[derive(Default)]
pub struct angvect3{roll:f64,pitch:f64,yaw:f64}
impl Vector for angvect3 {
    type V = angvect3;

    fn to_slice(&self) -> &mut [f64] {
        &mut [self.roll,self.pitch,self.yaw]
    }
    fn add(ours: &mut f64, theirs: f64) {
        *ours = f64::abs(*ours + theirs) % 360.0;
    }
}

pub trait Interaction {
    fn interact(&self, start:f64, dt:f64) -> (vect3, angvect3);
}

pub struct ThrustInteraction {
    thruster: Box<Mover>,
    parent: Box<Mover>,

    effective_translation: vect3,
    effective_rotation: angvect3,
    flip: bool,
}

impl ThrustInteraction {
    fn new(thruster: Box<Mover> , flip: Option<bool>) -> ThrustInteraction {
        let flip = flip.unwrap_or(true);

        let mut parent: Box<Mover> = thruster;
        let mut effective_translation: vect3 = thruster.translation;
        let mut effective_rotation: angvect3 = thruster.rotation;

        if flip == true {
            effective_rotation.add_all(-180.);
        }

        let mut recipient: Box<Mover> = thruster;
        while recipient.parent.is_none() {
            recipient = recipient.parent.unwrap();

            effective_translation.vector_add(&recipient.translation);
            effective_rotation.vector_add(&recipient.rotation);
        }
        parent = recipient;

        ThrustInteraction{
            thruster,
            parent, 
            effective_translation, 
            effective_rotation, 
            flip
        }
    }
}

pub struct Mover {
    parent: Option<Box<Mover>>,

    translation: vect3,
    rotation: angvect3,

    interactions: Vec<Box<dyn Interaction>>
}

fn main() {
    println!("Hello, world!");
}
