use std::any::Any;
use std::any::TypeId;
use std::cell::RefCell;
use std::ops::Index;
use std::ops::Add;
use std::ops::Mul;
use std::rc::Rc;

use nalgebra::Matrix;
use nalgebra::Rotation3;
use nalgebra::Vector3;

const X: usize = 0;
const Y: usize = 1;
const Z: usize = 2;

const ROLL: usize = 0;
const PITCH: usize = 1;
const YAW: usize = 2;

const DEFAULT_COSMIC_SPEED_LIMIT: f64 = 299_792_458.0;
const DEFAULT_SPEED_OF_LIGHT: f64 = DEFAULT_COSMIC_SPEED_LIMIT;
const PI: f64 = std::f64::consts::PI;

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
macro_rules! Vect3_new {
    () => {
        Vect3::new(0.,0.,0.)
    };
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

pub type Vect3 = Vector3<f64>;

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

pub struct Mover {
    parent: Option<Rc<Mover>>,
    children: Vec<Rc<Mover>>,

    ///relative to the origin of the space it is operating in, outer space for top level Mover's and the center of parent for recursive Mover's
    translation: RefCell<Vect3>,
    rotation: RefCell<Vect3>,//figure out how to modulo it later

    velocity: RefCell<Vect3>,
    angular_velocity: RefCell<Vect3>,

    rest_mass: f64,
    rel_mass: f64,

    radius: f64,

    health: f64,
    is_alive: bool,

    current_thrust: f64,
    thrust_rotation: Vect3,
    F_thrust: Vect3,

    name: String,

}



impl Mover {
    pub fn set_thrust(&mut self, new_thrust: f64) {
        self.current_thrust = new_thrust; 
        self.set_F_thrust();
    }

    pub fn rotate_thrust(&mut self, rotate_by: Vect3) {
        self.thrust_rotation += rotate_by;
        self.set_F_thrust();
    }

    pub fn set_thrust_rotation(&mut self, new_rotation: Vect3) {
        self.thrust_rotation = new_rotation;
        self.set_F_thrust();
    }

    fn set_F_thrust(&mut self) {
        let rot_matrix = Rotation3::from_euler_angles(self.thrust_rotation.x, self.thrust_rotation.y, self.thrust_rotation.z);
        self.F_thrust = rot_matrix * Vect3::new(self.current_thrust, 0., 0.);
    }
    
    pub fn new(parent: Option<Rc<Mover>>, translation: Option<Vect3>, rotation: Option<Vect3>) -> Self {
        let translation = translation.unwrap_or(Vect3_new!());
        let rotation = rotation.unwrap_or(Vect3_new!());

        Mover{
            parent,
            children: vec![],
            translation: RefCell::new(translation),
            rotation: RefCell::new(rotation),

            velocity: RefCell::new(Vect3_new!()),
            angular_velocity: RefCell::new(Vect3_new!()),
            
            rest_mass: 1.,
            rel_mass: 1.,
            radius: 1.,
            health: 1.,
            is_alive: true,
            current_thrust: 0.,
            thrust_rotation: Vect3::new(-PI, 0., 0.),
            F_thrust: Vect3_new!(),
            name: "unnamed".to_string()
        }
    }

    pub fn set_rest_mass(&mut self, new_rest_mass: f64, vel_magnitude: f64) {
        self.rest_mass = new_rest_mass;
        self.set_rel_mass(vel_magnitude)
    }
    fn set_rel_mass(&mut self, vel_magnitude: f64) {
        self.rel_mass = self.rest_mass/f64::sqrt(1. - vel_magnitude.powf(2.) / C_S_L.powf(2.));
    }

    #[inline(always)]
    pub fn mass(&self) -> f64 {
        self.rest_mass
    }

    pub fn add_health(&mut self, addition: f64) {
        self.set_health(self.health + addition);
    }
    pub fn set_health(&mut self, new_health: f64) {
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

    pub fn get_moment_of_inertia(&self) -> f64 {
        (2./5.) * self.mass() * self.radius.powf(2.)
    }

}

fn get_dist(mover1: &Rc<Mover>, mover2: &Rc<Mover>) -> f64 {
    let center1 = &mover1.translation.borrow();
    let center2 = &mover2.translation.borrow();

    f64::sqrt(
        (center1.x - center2.x).powf(2.) + (center1.y - center2.y).powf(2.) + (center1.z - center2.z).powf(2.)
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
    let mut first = Mover::new(None, None, None);
    first.set_thrust(1.);
    first.name = "test".to_string();

    movers.push((Rc::new(first), Deltas::new()));
}

struct LinAng {
    lin: vect3,
    ang: angvect3
}

struct Deltas {
    forces: (Vect3, Vect3),
    accelerations: (Vect3, Vect3),
    new_velocities: (Vect3, Vect3),
    new_positions: (Vect3, Vect3)
}
impl Deltas {
    pub fn new() -> Self {
        Deltas {
            forces: (Vect3_new!(), Vect3_new!()),
            accelerations: (Vect3_new!(), Vect3_new!()),
            new_velocities: (Vect3_new!(), Vect3_new!()),
            new_positions: (Vect3_new!(), Vect3_new!())
        }
    }
}

fn is_done(movers: &Vec<(Rc<Mover>, Deltas)>, time_elapsed: f64) -> bool {
    movers.iter().all(|t| t.0.clone().is_alive == false) || time_elapsed > 100.
}

///(linear, torque)
fn sum_thrust_forces_for(mover: Rc<Mover>, dt: f64) -> (Vect3, Vect3) {
    let mut F: Matrix<f64, nalgebra::Const<3>, nalgebra::Const<1>, nalgebra::ArrayStorage<f64, 3, 1>> = mover.F_thrust;
    let mut torque: Matrix<f64, nalgebra::Const<3>, nalgebra::Const<1>, nalgebra::ArrayStorage<f64, 3, 1>> = Vect3_new!();

    for child in mover.children.iter() {
        let mut child_forces = sum_thrust_forces_for(child.clone(), dt);
        let child_F = &mut child_forces.0;
        let child_torque = &mut child_forces.1;

        //the torque on us is the cross product of the net linear force vector on the child's point in us and 
        //the displacement vector of the child relative to our origin

        let torque_on_us = child_F.cross(&child.translation.borrow());
        torque += torque_on_us;
        F += *child_F;
    }

    (F, torque)
}

fn sum_thrust_forces(movers: &mut Vec<(Rc<Mover>, Deltas)>, dt: f64) {
    for (mover, delta) in movers.iter_mut() {
        let sum_lin_and_torque = sum_thrust_forces_for(mover.clone(), dt);
        delta.forces.0 += sum_lin_and_torque.0;
        delta.forces.1 += sum_lin_and_torque.1;
    }
}

fn calculate_gravity_forces(movers: &mut Vec<(Rc<Mover>, Deltas)>, dt: f64) {

}

fn sum_forces(movers: &mut Vec<(Rc<Mover>, Deltas)>, dt: f64) {
    sum_thrust_forces(movers, dt);
    calculate_gravity_forces(movers, dt);
}

fn move_movers(movers: &mut Vec<(Rc<Mover>, Deltas)>, dt: f64) {

    for (mover, delta) in movers.iter_mut() {
        delta.accelerations.0 = delta.forces.0 / mover.mass();
        delta.accelerations.1 = delta.forces.1 / mover.get_moment_of_inertia();

        //let mut translation = *mover.translation.borrow_mut();


        *mover.velocity.borrow_mut() += delta.accelerations.0 * dt;

        //let mut rotation = *mover.rotation.borrow_mut();
        *mover.angular_velocity.borrow_mut() += delta.accelerations.1 * dt;

        *mover.translation.borrow_mut() += *mover.velocity.borrow() * dt;
        *mover.rotation.borrow_mut() += *mover.angular_velocity.borrow() * dt;
    }
}

struct RunResult;

fn run(initial_dt: f64, movers: &mut Vec<(Rc<Mover>, Deltas)>) -> RunResult {
    let mut time: f64 = 0.;
    let mut dt: f64 = initial_dt;

    loop {
        for e in movers.iter_mut() {
            e.1 = Deltas::new();
        }
        sum_forces(movers, dt);
        move_movers(movers, dt);

        for mover in movers.iter() {
            println!("mover {}, force: {}N, acceleration: {} m/s/s, velocity: {} m/s, position: {} m", mover.0.name, mover.1.forces.0, mover.1.accelerations.0, *mover.0.velocity.borrow(), *mover.0.translation.borrow());
        }

        if is_done(movers, time) {
            break;
        }
        time += dt;
    }

    println!("done!");

    RunResult
}

fn main() {

    let mut movers: Vec<(Rc<Mover>, Deltas)> = vec![];
    create_movers(&mut movers);

    let dt: f64 = 1.;

    run(dt, &mut movers);
    
}
