use std::any::Any;
use std::any::TypeId;
use std::ops::Index;
use std::ops::Add;
use std::rc::Rc;

const X: usize = 0;
const Y: usize = 1;
const Z: usize = 2;

const ROLL: usize = 0;
const PITCH: usize = 1;
const YAW: usize = 2;

const DEFAULT_COSMIC_SPEED_LIMIT: f64 = 299_792_458.0;
const DEFAULT_SPEED_OF_LIGHT: f64 = DEFAULT_COSMIC_SPEED_LIMIT;

static C_S_L: f64 = DEFAULT_COSMIC_SPEED_LIMIT;
static C_V: f64 = DEFAULT_SPEED_OF_LIGHT;

macro_rules! vec_index {
    ($type:ty) => {
        impl Index<usize> for $type {
            type Output = f64;

            fn index(&self, index: usize) -> &Self::Output {
                return &self.arr[index];
            }
        }
    };
}
macro_rules! as_any {
    () => {
        fn as_any(&self) -> &dyn Any {
            self
        }
    }
}
/*
macro_rules! vec_new {
    ($type:ty) => {
        fn new_empty() -> Self {
            return $type{arr:[0.,0.,0.]};
        }
    };
}*/

pub trait ConstVector<const S: usize> {

}

pub trait Vector {
    const S: usize;
    type V: Vector;

    fn new_empty() -> Self;
    fn get(&self) -> &[f64; 3];
    fn get_mut(&mut self) -> &mut [f64; 3];

    fn add(ours: &mut f64, theirs: f64);

    fn add_all(&mut self, other: f64) {
        for field in self.get_mut() {
            Self::add(field, other);
        }
    }

    fn vector_add(&mut self, other: &Self::V) {
        let theirs: &[f64;3] = other.get();
        for (i, field) in self.get_mut().iter_mut().enumerate() {
            Self::add(field, theirs[i]);
        }
    }

    fn mult(ours: &mut f64, theirs: f64);

    fn mult_all(&mut self, other: f64) {
        for field in self.get_mut() {
            Self::mult(field, other);
        }
    }

    fn magnitude(&self) -> f64 {
        let a = self.get();
        let x = a.into_iter();
        let y = x.map(|&dim| dim.powf(2.));
        let z = y.sum();
        f64::sqrt(
            z
        )
    }


}

#[derive(Default)]
pub struct vect3{arr: [f64; 3]}
impl Vector for vect3 {
    type V = vect3;
    const S: usize = 3;

    fn new_empty() -> Self {
        vect3{arr:[0.,0.,0.,]}
    }
    
    fn get_mut(&mut self) -> &mut [f64;3] {
        return &mut self.arr;
    }
    fn get(&self) -> &[f64; 3] {
        return &self.arr;
    }

    fn add(ours: &mut f64, theirs: f64) {
        *ours += theirs;
    }

    fn mult(ours: &mut f64, theirs: f64) {
        *ours *= theirs;
    }
    

}
impl Clone for vect3 {
    fn clone(&self) -> Self {
        return vect3{arr: self.arr.clone()}
    }
}
impl vect3 {
    fn get_dist(a: vect3, b: vect3) -> f64 {

        f64::sqrt(
            (a[X] - b[X]).powf(2.) + (a[Y] - b[Y]).powf(2.) + (a[Z] - b[Z]).powf(2.)
        )
    }
}
/*
impl Index<usize> for vect3 {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        return &self.arr[index];
    }
}*/
vec_index!(vect3);



#[derive(Default)]
pub struct angvect3{arr: [f64; 3]}
impl Vector for angvect3 {
    type V = angvect3;
    const S: usize = 3;

    fn new_empty() -> Self {
        angvect3{arr:[0.,0.,0.,]}
    }
    fn get_mut(&mut self) -> &mut [f64;3] {
        return &mut self.arr;
    }
    fn get(&self) -> &[f64; 3] {
        return &self.arr;
    }
    fn add(ours: &mut f64, theirs: f64) {
        *ours = f64::abs(*ours + theirs) % 360.0;
    }//TODO: this is very wrong for anything other than current angular position

    fn mult(ours: &mut f64, theirs: f64) {
        *ours = f64::abs(*ours * theirs) % 360.0;
    }
}
impl Clone for angvect3 {
    fn clone(&self) -> Self {
        return angvect3{arr: self.arr.clone()}
    }
}
vec_index!(angvect3);

pub trait Interaction {
    fn interact(&self, start:f64, dt:f64) -> (Rc<Mover>, vect3, angvect3);
    fn as_any(&self) -> &dyn Any;
}


pub struct ThrustInteraction {
    thruster: Rc<Mover>,
    parent: Rc<Mover>,

    effective_translation: vect3,
    effective_rotation: angvect3,
    flip: bool,
}

impl ThrustInteraction {
    fn new(thruster: &mut Rc<Mover> , flip: Option<bool>) -> ThrustInteraction {
        let flip = flip.unwrap_or(true);

        let mut parent: Rc<Mover> = thruster.clone();
        let mut effective_translation: vect3 = thruster.translation.clone();
        let mut effective_rotation: angvect3 = thruster.rotation.clone();

        if flip == true {
            effective_rotation.add_all(-180.);
        }

        let mut recipient: &Rc<Mover> = &thruster;
        while recipient.parent.is_some() {
            match &recipient.parent {
                Some(val) => {
                    recipient = &val;
                }
                None => break
            }

            effective_translation.vector_add(&recipient.translation);
            effective_rotation.vector_add(&recipient.rotation);
        }
        parent = recipient.clone();

        let ret = ThrustInteraction{
            thruster: thruster.clone(),
            parent: parent.clone(), 
            effective_translation, 
            effective_rotation, 
            flip
        };
        thruster.owned_interactions.push(Rc::new(ret));

        ret
    }
    
}

impl Interaction for ThrustInteraction {
    fn interact(&self, start:f64, dt:f64) -> (Rc<Mover>, vect3, angvect3) {
        let mut thrust_center: vect3 = vect3{arr: [0.,0.,0.]};
        let mut thrust_rotation: angvect3 = angvect3{arr: [0.,0.,0.]};
        if std::ptr::eq(&self.thruster as *const _, &self.parent as *const _) {
            thrust_center = self.parent.translation.clone();
            thrust_rotation = self.parent.rotation.clone();

            if self.flip {
                thrust_rotation.add_all(-180.0);
            }
        } else {
            thrust_center = self.effective_translation.clone();
            thrust_rotation = self.effective_rotation.clone();
        }

        let thrust: f64 = self.thruster.current_thrust;
        
        let F: vect3 = vect3{arr: [
            thrust * thrust_rotation[ROLL], 
            thrust * thrust_rotation[PITCH], 
            thrust * thrust_rotation[YAW]]
        };

        let torque: angvect3 = angvect3 { arr: [
            thrust_center[Y] * F[Z] - F[Y] * thrust_center[Z],
            thrust_center[Z] * F[X] - F[Z] * thrust_center[X],
            thrust_center[X] * F[Y] - F[X] * thrust_center[Y]
        ] 
        };

        (self.parent.clone(), F, torque)
    }
    as_any!();
}

pub struct Mover {
    parent: Option<Rc<Mover>>,

    translation: vect3,
    rotation: angvect3,

    velocity: vect3,
    angular_velocity: angvect3,

    interactions: Vec<Rc<dyn Interaction>>,
    owned_interactions: Vec<Rc<dyn Interaction>>,

    rest_mass: f64,
    rel_mass: f64,

    radius: f64,

    health: f64,
    is_alive: bool,

    current_thrust: f64,
    name: String,

}
impl Mover {
    fn set_thrust(&mut self, new_thrust: f64) {
        self.current_thrust = new_thrust;
        let mut existing_thrust_interaction: Option<&ThrustInteraction> = None;
        /*self.owned_interactions
            .iter()
            .any(|e| e.is::<ThrustInteraction>());//e.type_id() == TypeId::of::<ThrustInteraction>()
        */

        for i in 0..self.owned_interactions.len() {
            let e = &self.owned_interactions[i];
            match e.as_any().downcast_ref::<ThrustInteraction>() {
                Some(m) => {
                    existing_thrust_interaction = Some(m);
                    break;
                }
                None => {
                    continue;
                }
            };
        }

        match existing_thrust_interaction {
            Some(ti) => {
                
            }
            None => {

            }
        }
    }
    
    fn new(parent: Option<Rc<Mover>>, translation: Option<vect3>, rotation: Option<angvect3>) -> Self {
        let translation = translation.unwrap_or(vect3::new_empty());
        let rotation = rotation.unwrap_or(angvect3::new_empty());

        Mover{
            parent,
            translation,
            rotation,

            velocity: vect3::new_empty(),
            angular_velocity: angvect3::new_empty(),
            interactions: vec![],
            owned_interactions: vec![],
            current_thrust: 0.,
            rest_mass: 1.,
            rel_mass: 1.,
            radius: 1.,
            health: 1.,
            is_alive: true,
            name: "unnamed".to_string()
        }
    }

    fn set_rest_mass(&mut self, new_rest_mass: f64, vel_magnitude: f64) {
        self.rest_mass = new_rest_mass;
        self.set_rel_mass(vel_magnitude)
    }
    fn set_rel_mass(&mut self, vel_magnitude: f64) {
        self.rel_mass = self.rest_mass/f64::sqrt(1. - vel_magnitude.powf(2.) / C_S_L.powf(2.));
    }

    #[inline(always)]
    fn mass(&self) -> f64 {
        self.rest_mass
    }

    fn add_health(&mut self, addition: f64) {
        self.set_health(self.health + addition);
    }
    fn set_health(&mut self, new_health: f64) {
        self.health = new_health;
        self.on_set_health();
    }
    fn on_set_health(&mut self) {
        match(self.is_alive, self.health <= 0.) {
            (true, true) => self.is_alive = false,
            (false, false) => self.is_alive = true,
            (_,_) => ()
        }
    } 

    fn get_moment_of_inertia(&self) -> f64 {
        (2./5.) * self.mass() * self.radius.powf(2.)
    }

}

fn get_dist(mover1: &Rc<Mover>, mover2: &Rc<Mover>) -> f64 {
    let center1: &vect3 = &mover1.translation;
    let center2: &vect3 = &mover2.translation;

    f64::sqrt(
        (center1[X] - center2[X]).powf(2.) + (center1[Y] - center2[Y]).powf(2.) + (center1[Z] - center2[Z]).powf(2.)
    )
}

enum Colliding {
    Now,
    Later{dt:f64},
    No
}

fn translate_FD(mut base: vect3, dt: f64, mut b_d_dt: vect3, mut b_d2_dt2: Option<vect3>) -> vect3 {
    let mut b_d2_dt2 = b_d2_dt2.unwrap_or(vect3::new_empty());

    b_d_dt.mult_all(dt);
    base.vector_add(&b_d_dt);

    b_d2_dt2.mult_all(dt.powf(2.));
    base.vector_add(&b_d2_dt2);

    base
}

fn check_collision(mover1: (Rc<Mover>, &Deltas), mover2: (Rc<Mover>, &Deltas), dt: f64) -> Colliding {
    let radius1: f64 = mover1.0.radius;
    let radius2: f64 = mover2.0.radius;

    let distance = get_dist(&mover1.0, &mover2.0);

    //let mover1_distance_delta = vect3::get_dist(mover1.0.translation, mover1.1.new_positions.lin);


    let mut collides = (distance <= radius1) || (distance <= radius2);
    if collides {
        return Colliding::Now
    }
    else {
        return Colliding::No
    }

    /*
    let deltas1 = mover1.1;
    let deltas2 = mover2.1;

    if collides {
        Colliding::Later{dt}
    } else {
        Colliding::No
    }
    */
}

fn find_collisions(movers: &Vec<(Rc<Mover>, Deltas)>, dt: f64) {
    for i in 0..movers.len() {
        for j in i..movers.len() {
            let collides = check_collision((movers[i].0.clone(), &movers[i].1), (movers[j].0.clone(), &movers[i].1), dt);
            match collides {
                Colliding::Now => {
                    println!("collision!")
                }
                _ => continue
            }
        }
    }
}

fn create_movers(movers: &mut Vec<(Rc<Mover>, Deltas)>) {

}

struct LinAng {
    lin: vect3,
    ang: angvect3
}

struct Deltas {
    forces: (vect3, vect3),
    accelerations: (vect3, vect3),
    new_velocities: (vect3, vect3),
    new_positions: (vect3, angvect3)
}

fn main() {
    let mut movers: Vec<(Rc<Mover>, Deltas)> = vec![];
    println!("Hello, world!");
}
