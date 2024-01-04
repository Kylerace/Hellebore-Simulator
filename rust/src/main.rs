#![feature(tuple_trait)]

use std::any::Any;
use std::any::TypeId;
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

pub type CollisionList = Vec<(Rc<RefCell<Mover>>, Rc<RefCell<Mover>>)>;

fn find_collisions(movers: &MoversList, time: f64, dt: f64) {
    let mut collisions: CollisionList = vec![];
    for i in 0..movers.len() {
        for j in i+1..movers.len() {
            let first = movers[i].0.clone();
            let second = movers[j].0.clone();

            let collides = check_collision((movers[i].0.clone(), &movers[i].1), (movers[j].0.clone(), &movers[i].1), dt);

            match collides {
                Colliding::Now => {
                    collisions.push((first.clone(), second.clone()));
                }
                _ => continue
            }
        }
    }
    handle_collisions(collisions, time, dt)
}

fn handle_collisions(colliders: CollisionList, time: f64, dt: f64) {
    for collision in colliders.iter() {
        flag_print!("print_handle_collisions", "mover: {} (at {} radius = {}) and mover: {} (at {} radius = {}) collided at time = {}", 
            collision.0.borrow().name, stringify_vector!(collision.0.borrow().translation.borrow(), 3), collision.0.borrow().radius, 
            collision.1.borrow().name, stringify_vector!(collision.1.borrow().translation.borrow(), 3), collision.1.borrow().radius, time + dt);
    }
}

//TODO: orientation should be represented with a quaternion
#[derive(Clone)]
pub struct Deltas {
    pub forces: (Vect3, Vect3),
    pub accelerations: (Vect3, Vect3),
    pub initial_velocities: (Vect3, Vect3),
    pub initial_positions: (Vect3, Vect3),
    pub best_velocities: (Vect3, Vect3),
    pub best_positions: (Vect3, Vect3),
}
impl Deltas {
    pub fn new() -> Self {
        Deltas {
            forces: (Vect3_new!(), Vect3_new!()),
            accelerations: (Vect3_new!(), Vect3_new!()),
            initial_velocities: (Vect3_new!(), Vect3_new!()),
            initial_positions: (Vect3_new!(), Vect3_new!()),
            best_velocities: (Vect3_new!(), Vect3_new!()),
            best_positions: (Vect3_new!(), Vect3_new!())
        }
    }
}

fn is_done(movers: &MoversList, time_elapsed: f64) -> bool {
    movers.iter().all(|t| t.0.borrow().is_alive == false) || time_elapsed > MAX_DURATION
}

///(linear, torque) in parent coordinates (world coords for top level movers, parent body coords for child movers)
fn sum_thrust_forces_for(mover: &Mover, dt: f64) -> (Vect3, Vect3) {
    let mut F = mover.rotation.borrow().transform_vector(&mover.F_thrust);
    
    let mut torque: Matrix<f64, nalgebra::Const<3>, nalgebra::Const<1>, nalgebra::ArrayStorage<f64, 3, 1>> = Vect3_new!();

    #[cfg(feature = "print_sum_forces")]
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
    for rc_t in movers.iter_mut() {
        let mover = &rc_t.0;
        let mut delta = &mut rc_t.1;

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

/**
 * before factoring in any constraints like collisions which may happen between now and now + dt, set the 
 * initial velocity and position of each mover's associated Deltas struct to what they should be
 */
fn set_initial_vectors(movers: &mut MoversList, dt: f64) {

    for (m, delta, a) in movers.iter_mut() {
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

fn poll_actors(movers: &mut MoversList, time: f64, dt: f64) {
    let mut movers_to_poll: Vec<&MoverTuple> = vec![];
    {
    for m in movers.iter() {
        if m.2.is_none() {//actor
            continue;
        }
        movers_to_poll.push(m);
        //let created_movers = m.borrow_mut().decide(m.clone(), a.as_mut().unwrap(), &*movers, time, dt);
    }

    }


    let mut all_created_movers: Vec<Mover> = vec![];

    for _m in movers_to_poll.into_iter() {
        //this code is weird because of an ant style evolutionary algorithm to develop code to get past the borrow checker
        //(fancy speak for idk what im doing)

        let m = &_m.0;
        let d = &_m.1;
        let a = &_m.2;

        let new_a = a.as_ref().unwrap().clone();
        //let next_a = new_a.borrow_mut();//.into_box();
        //let nexter_a = next_a.into_box();
        let mut created_movers = Mover::decide(m.clone(), new_a, &*movers, time, dt);

        all_created_movers.append(&mut created_movers);
        //movers.append(&created_movers.map(|m| (Rc::new(RefCell::new(mover)), Deltas::new(), None)));
    }


    for new_mover in all_created_movers.into_iter() {
        let new_tuple = (Rc::new(RefCell::new(new_mover)), Deltas::new(), Option::<ActorRepresentation>::None);
        movers.push(new_tuple);
    }

}

fn run(initial_dt: f64, min_tick_time: std::time::Duration, movers: &mut MoversList) -> RunResult {
    let mut time: f64 = 0.;
    let mut dt: f64 = initial_dt;

    loop {
        let start_of_tick = time::Instant::now();

        for e in movers.iter_mut() {
            e.1 = Deltas::new();
        }

        poll_actors(movers, time, dt);

        sum_forces(movers, dt);
        set_initial_vectors(movers, dt);

        find_collisions(movers, time, dt);

        for m in movers.iter() {
            //let mover_1_force = format!("[{}, {}, {}]", mover.1.forces.0.x, mover.1.forces.0.y, mover.1.forces.0.z);

            let mover = m.0.borrow();

            #[cfg(feature = "print_run_deltas_any")] 
            {
                println!("run deltas:");
                println!("\tmover: {}", mover.name);
                println!("\ttime/dt: {}/{} s", time, dt);
            }

            #[cfg(feature = "print_run_deltas_linear")]
            {
                println!("\tforce: {} N", stringify_vector!(m.1.forces.0, 4));
                println!("\tacceleration: {} m/s/s", stringify_vector!(m.1.accelerations.0, 4));
                println!("\tvelocity: {} m/s", stringify_vector!(*mover.velocity.borrow(), 4));
                println!("\tposition: {} m", stringify_vector!(*mover.translation.borrow(), 4));
            }

            #[cfg(feature = "print_run_deltas_angular")]
            {
                println!("\ttorque: {} Nm", stringify_vector!(m.1.forces.1, 4));
                println!("\tangular acceleration: {} rad/s/s:", stringify_vector!(m.1.accelerations.1, 4));
                println!("\tangular velocity: {} rad/s", stringify_vector!(*mover.angular_velocity.borrow(), 4));
                println!("\torientation: {} rad", *mover.rotation.borrow());
            }

            #[cfg(feature = "print_run_deltas_thrust_vectors")]
            {
                let mut lines: Vec<String> = vec![];
                
                if mover.current_thrust > 0. {
                    lines.push(format!("{} current thrust: {}, thrust rotation: {}, F_thrust: {}", mover.name, mover.current_thrust, mover.thrust_rotation, stringify_vector!(mover.F_thrust)));
                }

                for _child in mover.children.iter() {
                    let child = _child.borrow();
                    if child.current_thrust > 0. {
                        lines.push(format!("{} current thrust: {}, thrust rotation: {}, F_thrust: {}", child.name, child.current_thrust, child.thrust_rotation, stringify_vector!(child.F_thrust)));
                    }
                }

                for line in lines {
                    println!("\t\t{}",line);
                }
            }
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

pub struct Photon {

}

fn create_movers(movers: &mut MoversList) {
    let mut first = RefCell::new(Mover::new(Some(Vect3_new!(0.,0.,0.)), None));
    /*
    first.set_thrust(1.);
    first.rotate_thrust(UnitQuaternion::from_axis_angle(&Vect3::y_axis(), 0.5*PI));

    first.set_rotation(UnitQuaternion::from_axis_angle(&Vect3::y_axis(), 0.5*PI));
    first.rotate_by(Vect3_new!(0.*PI, 0.25*PI, 0.));
    */

    //first.set_rest_mass(10., 0.);
    first.borrow_mut().name = "first".to_string();
    first.borrow_mut().radius = 5.;

    first.borrow_mut().set_thrust(1.);

    push_end_print!("first's F vector: {}, thrust rotation: {}, overall rotation: {}, thrust rotation rotating [1,0,0]: {}", 
        stringify_vector!(first.borrow().F_thrust, 3), first.borrow().thrust_rotation, first.borrow().rotation.borrow(), first.borrow().thrust_rotation.transform_vector(&Vect3_new!(1.,0.,0.)));

    let mut second = RefCell::new(Mover::new(
        Some(Vect3_new!(10.,0.,0.)), 
        Some(UnitQuaternion::from_axis_angle(&Vect3::z_axis(), PI)))
    );
    second.borrow_mut().name = "second".to_string();
    second.borrow_mut().set_thrust(1.);
    second.borrow_mut().radius = 5.;

    let first_rc = Rc::new(first);

    movers.push( (first_rc, Deltas::new(), None) );
    movers.push( (Rc::new(second), Deltas::new(), None) );
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
