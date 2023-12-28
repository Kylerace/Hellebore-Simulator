use std::any::Any;
use std::any::TypeId;
use std::borrow::BorrowMut;
use std::cell::RefCell;
use std::ops::Index;
use std::ops::Add;
use std::ops::Mul;
use std::rc::Rc;
use std::thread;
use std::time;

use nalgebra::Matrix;
use nalgebra::Rotation3;
use nalgebra::Unit;
use nalgebra::Vector3;
use nalgebra::Quaternion;
use nalgebra::UnitQuaternion;

const X: usize = 0;
const Y: usize = 1;
const Z: usize = 2;

const ROLL: usize = 0;
const PITCH: usize = 1;
const YAW: usize = 2;

const DEFAULT_COSMIC_SPEED_LIMIT: f64 = 299_792_458.0;
const DEFAULT_SPEED_OF_LIGHT: f64 = DEFAULT_COSMIC_SPEED_LIMIT;
const PI: f64 = std::f64::consts::PI;
const MAX_DURATION: f64 = 5.;

static C_S_L: f64 = DEFAULT_COSMIC_SPEED_LIMIT;
static C_V: f64 = DEFAULT_SPEED_OF_LIGHT;

static mut end_prints: Vec<String> = vec![];
static mut outs_to_use: Vec<String> = vec![];


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
    ($x:expr, $y:expr, $z:expr) => {
        Vect3::new($x,$y,$z)
    };
}

macro_rules! stringify_vector {
    ($vector:expr) => {
        format!("[{}, {}, {}]", $vector.x, $vector.y, $vector.z)
    };
    ($vector:expr, $precision:literal) => {
        format!("[{:.3$}, {:.3$}, {:.3$}]", $vector.x, $vector.y, $vector.z, $precision)
    };
}

macro_rules! flag_print {
    () => {
        println!()
    };
    ($feature_flag:literal, $($arg:tt)*) => {{
        #[cfg(feature = "print_flag_print")]
        println!("flag_print:");
        
        #[cfg(feature = $feature_flag)]
        {
            println!($($arg)*);
        }
    }};
}

macro_rules! push_end_print {
    ($($arg:tt)*) => {
        unsafe {
            end_prints.push(format!($($arg)*));
        }
    };
    ($feature_flag:literal, $($arg:tt)*) => {
        #[cfg(feature = $feature_flag)]
        unsafe {
            end_prints.push(format!($($arg)*));
        }
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

type MoversList = Vec<(Rc<RefCell<Mover>>, Deltas)>;

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
    children: Vec<Rc<RefCell<Mover>>>,

    ///relative to the origin of the space it is operating in, outer space for top level Mover's and the center of parent for recursive Mover's
    translation: RefCell<Vect3>,
    rotation: RefCell<UnitQuaternion<f64>>,//figure out how to modulo it later

    velocity: RefCell<Vect3>,
    angular_velocity: RefCell<Vect3>,

    rest_mass: f64,
    rel_mass: f64,

    radius: f64,

    health: f64,
    is_alive: bool,

    current_thrust: f64,
    thrust_rotation: UnitQuaternion<f64>,
    /// the force of thrust vector with rotation vector of <0,0,0>, you need to multiply this by a rotation matrix of rotation's euler angles to get 
    /// the true force of thrust vector in body coordinates.
    /// 
    /// linear F = self's rotation matrix * F_thrust of self + sigma(child's rotation matrix * child's F_thrust).
    /// 
    /// torque = sigma(torque of each child + child's translation.cross(child's F)).
    F_thrust: Vect3,

    name: String,

}



impl Mover {
    pub fn new(translation: Option<Vect3>, rotation: Option<UnitQuaternion<f64>>) -> Self {
        let translation = translation.unwrap_or(Vect3_new!());
        let _original = rotation.clone();
        let mut rotation = rotation.unwrap_or(UnitQuaternion::<f64>::from_axis_angle(&-Vector3::<f64>::z_axis(), 0. * PI));//

        Mover{
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
            thrust_rotation: UnitQuaternion::<f64>::new(Vect3::z() * -PI),
            F_thrust: Vect3_new!(),
            name: "unnamed".to_string()
        }
    }

    pub fn set_rotation(&mut self, new_rotation: UnitQuaternion<f64>) {
        self.rotation.replace(new_rotation);
    }

    pub fn rotate_by(&mut self, rotate_by: Vect3) {
        let rotate_by_matrix = Rotation3::from_euler_angles(rotate_by.x, rotate_by.y, rotate_by.z);
        let rotation_quat = UnitQuaternion::from_euler_angles(rotate_by.x, rotate_by.y, rotate_by.z);

        let pre = self.rotation.clone().into_inner();

        //let _test = **self.rotation.borrow();
        //self.rotation.replace(rotate_by_matrix * (*self.rotation.borrow()).clone());// = rotate_by_matrix * *self.rotation.borrow_mut();
        self.rotation.replace_with(|&mut rot| rot * rotation_quat);//rotation_quat * rot * rotation_quat.conjugate());

        #[cfg(feature = "print_rotate_by")]
        {
            let pre_rotation_str = format!("Unit Quaternion angle: {:.4}π - axis: ({}, {}, {})", pre.angle() / PI, pre.axis().unwrap().x, pre.axis().unwrap().y, pre.axis().unwrap().z);
            let post = self.rotation.clone().into_inner();
            let post_rotation_str = format!("Unit Quaternion angle: {}π - axis: ({}, {}, {})", post.angle() / PI, post.axis().unwrap().x, post.axis().unwrap().y, post.axis().unwrap().z);

            let rotation_vector_str = format!("[{:.3}π, {:.3}π, {:.3}π]", rotate_by.x/PI, rotate_by.y/PI, rotate_by.z/PI);

            let rotation_quaternion_str = format!("Unit Quaternion angle: {:.4}π - axis: ({}, {}, {})", 
                rotation_quat.angle() / PI, rotation_quat.axis().unwrap().x, rotation_quat.axis().unwrap().y, rotation_quat.axis().unwrap().z);

            println!("rotate_by(): \nself.rotation before: {} \n+ rotation vector: {} -> rotation quaternion: {} \n= after: {} \nrotation_matrix: {}", 
            pre_rotation_str, rotation_vector_str, rotation_quaternion_str, post_rotation_str, rotate_by_matrix);
        }
            
        //println!("self.rotation before rotate_by: {} after: {}", pre, self.rotation.clone().into_inner());
    }

    pub fn set_thrust(&mut self, new_thrust: f64) {
        self.current_thrust = new_thrust; 
        self.set_F_thrust();
    }

    pub fn rotate_thrust(&mut self, rotate_by: UnitQuaternion<f64>) {
        self.thrust_rotation *= rotate_by;
        self.set_F_thrust();
    }

    pub fn set_thrust_rotation(&mut self, new_rotation: UnitQuaternion<f64>) {
        self.thrust_rotation = new_rotation;
        self.set_F_thrust();
    }

    fn set_F_thrust(&mut self) {
        self.F_thrust = self.thrust_rotation.transform_vector(&Vect3::new(self.current_thrust, 0., 0.));
        flag_print!("print_set_F_thrust", "{}.set_F_thrust() = {}, thrust_rotation = {}, which would rotate [0,1,0] to: {}", 
            self.name, stringify_vector!(self.F_thrust), self.thrust_rotation, stringify_vector!(self.thrust_rotation.transform_vector(&Vect3::new(0.,1.,0.))));
    }
    
    

    pub fn add_child(&mut self, new_child: Rc<RefCell<Mover>>) -> Result<(), &'static str> {
        if self.children.iter().any(|c| std::ptr::eq(&*c.borrow(), &*new_child.borrow())) {
            return Err("child already in!");
        }
        if new_child.borrow().children.iter().any(|c| std::ptr::eq(&*c.borrow(), self)) {
            return Err("child is parent of self!");
        }

        self.children.push(new_child.clone());

        let vel_magnitude = self.velocity.borrow().magnitude(); 
        self.set_rest_mass(self.rest_mass + new_child.borrow().rest_mass, vel_magnitude);

        Ok(())
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

fn get_dist(mover1: &Mover, mover2: &Mover) -> f64 {
    let center1 = mover1.translation.borrow();
    let center2 = mover2.translation.borrow();

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

fn check_collision(mover1: (Rc<RefCell<Mover>>, &Deltas), mover2: (Rc<RefCell<Mover>>, &Deltas), dt: f64) -> Colliding {
    let mover_1 = mover1.0.borrow();
    let mover_2 = mover2.0.borrow();

    let radius1: f64 = mover_1.radius;
    let radius2: f64 = mover_2.radius;

    let distance = get_dist(&mover1.0.borrow(), &mover2.0.borrow());

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

fn find_collisions(movers: &MoversList, dt: f64) {
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

fn is_done(movers: &MoversList, time_elapsed: f64) -> bool {
    movers.iter().all(|t| t.0.borrow().is_alive == false) || time_elapsed > MAX_DURATION
}

///(linear, torque) in parent coordinates (world coords for top level movers, parent body coords for child movers)
fn sum_thrust_forces_for(mover: &Mover, dt: f64) -> (Vect3, Vect3) {
    let mut F = mover.rotation.borrow().transform_vector(&mover.F_thrust);
    //println!("F_thrust: {}, rotation matrix: {}, F: {}", stringify_vector!(mover.F_thrust), our_rot_matrix, stringify_vector!(F));
    
    let mut torque: Matrix<f64, nalgebra::Const<3>, nalgebra::Const<1>, nalgebra::ArrayStorage<f64, 3, 1>> = Vect3_new!();

    let mut child_strings: Vec<String> = vec![];

    for c in mover.children.iter() {
        let child = c.borrow();
        let mut child_forces = sum_thrust_forces_for(&child, dt);
        let child_F = &mut child_forces.0;
        let child_torque = &mut child_forces.1;

        //the torque on us is the cross product of the net linear force vector on the child's point in us and 
        //the displacement vector of the child relative to our origin, added to the net torque on the child due to conservation of angular momentum
        //since torque is the derivative of angular momentum with respect to time, and angular momentum is conserved in a closed system (any node + its children)
        //the angular momentum of the parent must also change by the sum of torques in the children

        let torque_on_us = child_F.cross(&child.translation.borrow());

        torque = torque + torque_on_us + *child_torque;
        F += *child_F;

        #[cfg(feature = "print_sum_forces")]
        child_strings.push(format!("child: {}, translation: {}, orientation: {}, F_thrust: {}, rotated F: {}, torque on us: {}", 
            child.name, stringify_vector!(child.translation.borrow(), 3), child.rotation.borrow(), stringify_vector!(child.F_thrust, 3), stringify_vector!(child_F, 3), stringify_vector!(torque_on_us, 3)));
    }

    #[cfg(feature = "print_sum_forces")]
    {
        let mut child_contributions = "".to_string();

        for cs in child_strings.into_iter() {
            child_contributions = format!("{}\n\t{}", child_contributions, cs);
        }

        println!("sum_thrust_forces_for({}, {}): \n\tF_thrust: {}, orientation: {}, F: {}, torque: {},{}", 
            mover.name, dt, stringify_vector!(mover.F_thrust), *mover.rotation.borrow(), stringify_vector!(F, 3), stringify_vector!(torque, 3),
            child_contributions);
    }
    //flag_print!("print_sum_forces", "sum_thrust_forces_for({}, {}): \n\tF_thrust: {}, orientation: {}, F: {}, torque: {}", 
    //    mover.name, dt, stringify_vector!(mover.F_thrust), *mover.rotation.borrow(), stringify_vector!(F, 3), stringify_vector!(torque, 3));

    (F, torque)
}

fn sum_thrust_forces(movers: &mut MoversList, dt: f64) {
    for (mover, delta) in movers.iter_mut() {
        let sum_lin_and_torque = sum_thrust_forces_for(&mover.borrow(), dt);
        delta.forces.0 += sum_lin_and_torque.0;
        delta.forces.1 += sum_lin_and_torque.1;
    }
}

fn calculate_gravity_forces(movers: &mut MoversList, dt: f64) {

}

fn sum_forces(movers: &mut MoversList, dt: f64) {
    sum_thrust_forces(movers, dt);
    calculate_gravity_forces(movers, dt);
}

fn move_movers(movers: &mut MoversList, dt: f64) {

    for (m, delta) in movers.iter_mut() {
        let mover = m.borrow();

        let force = delta.forces.0;
        let torque = delta.forces.1;

        delta.accelerations.0 = force / mover.mass();

        //angular stuff is harder.
        //moment of inertia tensor thats invariant to time, this is in the BODY's frame of reference not the world's
        let moment_of_inertia = mover.get_moment_of_inertia();
        let inverse_MOI = 1./moment_of_inertia;
        //let rot_matrix = Rotation3::from_euler_angles(mover.thrust_rotation.x, mover.thrust_rotation.y, mover.thrust_rotation.z);

        //let MMOI = rot_matrix.matrix().scale(moment_of_inertia_body) * rot_matrix.transpose();

        let old_angular_velocity = *mover.angular_velocity.borrow();//implicit clone
        //old_angular_velocity = old_angular_velocity.cross(&old_angular_velocity.scale(moment_of_inertia));

        //angular acceleration
        delta.accelerations.1 = inverse_MOI * (torque - old_angular_velocity.cross(&old_angular_velocity.scale(moment_of_inertia)));

        flag_print!("print_new_angular_acceleration", "new angular acceleration: {}\n torque: {}\n old angular velocity: {}\n MOI/inverse: {}, {}\n cross product: {}",
            delta.accelerations.1, torque, old_angular_velocity, moment_of_inertia, inverse_MOI, old_angular_velocity.cross(&old_angular_velocity.scale(moment_of_inertia)));

        //let mut translation = *mover.translation.borrow_mut();


        *mover.velocity.borrow_mut() += delta.accelerations.0 * dt;

        //let mut rotation = *mover.rotation.borrow_mut();
        *mover.angular_velocity.borrow_mut() += delta.accelerations.1 * dt;

        *mover.translation.borrow_mut() += *mover.velocity.borrow() * dt;

        let delta_orientation = *mover.angular_velocity.borrow() * dt;
        *mover.rotation.borrow_mut() *= UnitQuaternion::<f64>::from_euler_angles(delta_orientation.x, delta_orientation.y, delta_orientation.z);
    }
}

struct RunResult;

fn run(initial_dt: f64, min_tick_time: std::time::Duration, movers: &mut MoversList) -> RunResult {
    let mut time: f64 = 0.;
    let mut dt: f64 = initial_dt;

    loop {
        let start_of_tick = time::Instant::now();

        for e in movers.iter_mut() {
            e.1 = Deltas::new();
        }
        sum_forces(movers, dt);
        move_movers(movers, dt);

        for m in movers.iter() {
            //let mover_1_force = format!("[{}, {}, {}]", mover.1.forces.0.x, mover.1.forces.0.y, mover.1.forces.0.z);

            let mover = m.0.borrow();

            #[cfg(feature = "print_run_deltas")]
            {
                println!("run deltas:");
                println!("\tmover: {}", mover.name);

               // println!("\tforce: {} N", stringify_vector!(m.1.forces.0, 4));
                println!("\ttorque: {} Nm", stringify_vector!(m.1.forces.1, 4));

                //println!("\tacceleration: {} m/s/s", stringify_vector!(m.1.accelerations.0, 4));
                println!("\tangular acceleration: {} rad/s/s:", stringify_vector!(m.1.accelerations.1, 4));

                //println!("\tvelocity: {} m/s", stringify_vector!(*mover.velocity.borrow(), 4));
                println!("\tangular velocity: {} rad/s", stringify_vector!(*mover.angular_velocity.borrow(), 4));

                //println!("\tposition: {} m", stringify_vector!(*mover.translation.borrow(), 4));
                println!("\torientation: {} rad", *mover.rotation.borrow());

                //println!("\ttime/dt: {}/{} s", time, dt);
            }

            /*
            flag_print!("print_run_deltas",
            "
            mover: {}, 
            force: {} N, 
            torque: {} Nm,
            acceleration: {} m/s/s, 
            angular acceleration: {} rad/s/s,
            velocity: {} m/s, 
            angular velocity: {} rad/s,
            position: {} m, 
            orientation: {} rad,
            time/dt: {}/{} s",

            mover.name, 
            stringify_vector!(m.1.forces.0, 4), 
            stringify_vector!(m.1.forces.1, 4),

            stringify_vector!(m.1.accelerations.0, 4), 
            stringify_vector!(m.1.accelerations.1, 4),
            
            stringify_vector!(*mover.velocity.borrow(), 4), 
            stringify_vector!(*mover.angular_velocity.borrow(), 4),

            stringify_vector!(*mover.translation.borrow(), 4),
            *mover.rotation.borrow(),

            time,
            dt,
            );
            */
            
            //println!("mover {}, force: {}N, acceleration: {} m/s/s, velocity: {} m/s, position: {} m", mover.0.name, mover.1.forces.0, mover.1.accelerations.0, *mover.0.velocity.borrow(), *mover.0.translation.borrow());
        }

        if is_done(movers, time) {
            break;
        }
        time += dt;

        let end_of_tick = time::Instant::now();

        let tick_duration = end_of_tick - start_of_tick;

        if tick_duration > min_tick_time {
            thread::sleep(tick_duration - min_tick_time);
        }

        
    }

    println!("done!");

    RunResult
}



fn create_movers(movers: &mut MoversList) {
    let mut first = RefCell::new(Mover::new(None, None));
    /*
    first.set_thrust(1.);
    first.rotate_thrust(UnitQuaternion::from_axis_angle(&Vect3::y_axis(), 0.5*PI));

    first.set_rotation(UnitQuaternion::from_axis_angle(&Vect3::y_axis(), 0.5*PI));
    first.rotate_by(Vect3_new!(0.*PI, 0.25*PI, 0.));
    */

    //first.set_rest_mass(10., 0.);
    first.borrow_mut().name = "parent".to_string();

    let mut thruster = Mover::new(Some(Vect3_new!(0., 0.25, 0.)), None);
    thruster.name = "thruster".to_string();
    thruster.set_thrust(1.);
    
    //thruster.set_rest_mass(0., 0.);
    
    //push_end_print!("thruster translation {} rotation: {}, F_thrust: {}", thruster.translation.borrow(), thruster.rotation.borrow(), thruster.F_thrust);

    let thruster_mass = thruster.mass();

    let _ = first.borrow_mut().add_child(Rc::new(RefCell::new(thruster)));

    push_end_print!("total parent mass: {} Kg, parent without child: {} Kg, child mass: {} Kg", first.borrow().mass(), first.borrow().mass() - thruster_mass, thruster_mass);
    push_end_print!("env args: {:?}", std::env::args().nth(1));

    let mut first_rc = Rc::new(first);

    movers.push((first_rc, Deltas::new()));
}

fn main() {

    let mut movers: MoversList = vec![];
    create_movers(&mut movers);

    let dt: f64 = 1.;
    let min_tick_time = time::Duration::from_millis((1000. / 60.) as u64);

    run(dt, min_tick_time, &mut movers);
    
    unsafe {
        if !end_prints.is_empty() {
            println!("\n___END PRINTS___\n");
        }
        for string_to_print in end_prints.iter() {
            println!("{}", string_to_print);
        }
    }
}
