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

#[macro_use]
mod globals;
mod mover;

use crate::mover::*;
use crate::globals::*;


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
