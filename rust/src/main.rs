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
mod actor;
mod action;
mod space;

use crate::actor::*;
use crate::mover::*;
use crate::globals::*;
use crate::action::*;
use crate::space::*;

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

    let mut collides = distance <= (radius1 + radius2) * COLLISION_RADIUS_MULTIPLIER;
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

fn find_collisions(movers: &mut MoversList, time: f64, dt: f64) {
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
    handle_collisions(collisions, movers, time, dt)
}

fn handle_collisions(colliders: CollisionList, movers: &mut MoversList, time: f64, dt: f64) {
    for collision in colliders.iter() {
        let m1 = collision.0.clone();
        let m2 = collision.1.clone();

        let v1 = m1.borrow().velocity.clone();
        let v2 = m2.borrow().velocity.clone();

        let mass1 = m1.borrow().mass();
        let mass2 = m2.borrow().mass();

        let e1 = 0.5 * mass1 * v1.borrow().magnitude().powf(2.);
        let e2 = 0.5 * mass2 * v2.borrow().magnitude().powf(2.);
        
        if f64::max(e1,e2) >= REMOVE_HIT_MOVERS_BEYOND_X_JOULES {
            m1.borrow_mut().set_health(0.);
            m2.borrow_mut().set_health(0.);
        }

        flag_print!("print_handle_collisions", "COLLISION mover: {} (at {}, velocity = {}, energy = {} J, r = {}) and mover: {} (at {}, velocity = {}, energy = {} J, r = {}) collided at time = {}", 
            collision.0.borrow().name, stringify_vector!(collision.0.borrow().translation.borrow(), 3), stringify_vector!(v1.borrow(), 3), e1, collision.0.borrow().radius, 
            collision.1.borrow().name, stringify_vector!(collision.1.borrow().translation.borrow(), 3), stringify_vector!(v2.borrow(), 3), e2, collision.1.borrow().radius, time);
    }
}

//TODO: orientation should be represented with a quaternion
#[derive(Clone, Debug)]
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
    let mut ret = movers.iter().all(|t| t.0.borrow().is_alive == false);
    if ret {
        println!("GAME DONE DUE TO ALL MOVERS DYING");
        return ret;
    }
    ret = ret || time_elapsed > MAX_DURATION;
    if ret {
        println!("GAME DONE DUE TO TIME");
        return ret;
    }

    let mut movers_by_team: Vec<(String, Vec<Rc<RefCell<Mover>>>, usize)> = vec![];
    for m_tuple in movers.iter() {
        let movers_team = m_tuple.0.borrow().team.clone();

        let mut found_existing_team = false;
        for existing_team_tuple in movers_by_team.iter_mut() {
            if movers_team == existing_team_tuple.0 {
                existing_team_tuple.1.push(m_tuple.0.clone());
                found_existing_team = true;
                if m_tuple.0.borrow().is_alive {
                    existing_team_tuple.2 += 1;
                }
                break;
            }
        }
        if !found_existing_team {
            let to_add = match m_tuple.0.borrow().is_alive {
                true => 1, 
                false => 0
            };
            movers_by_team.push((movers_team, vec![m_tuple.0.clone()], to_add));
        }
    }

    if movers_by_team.len() > 1 {
        let mut surviving_teams = 0;
        let mut last_surviving_team_name: Option<String> = None;
        for team_tuple in movers_by_team.iter() {
            if team_tuple.2 > 0 {
                surviving_teams += 1;
                last_surviving_team_name = Some(team_tuple.0.clone());
            }
        }

        if surviving_teams == 1 {
            ret = true;
            println!("GAME DONE DUE TO TEAM {} BEING THE ONLY SURVIVOR", last_surviving_team_name.unwrap_or("ERROR".to_string()));
        }
    }



    ret
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

        println!("sum_thrust_forces_for({}, {}): \n\tF_thrust: {}, orientation: {}, thrust_rotation: {}, rotated F: {}, torque: {},{}", 
            mover.name, dt, stringify_vector!(mover.F_thrust, 3), *mover.rotation.borrow(), stringify_vector!(mover.thrust_rotation.transform_vector(&Vect3::x()), 3), stringify_vector!(F, 3), stringify_vector!(torque, 3),
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
        let mover = m.borrow_mut();

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

        let lin_vel = delta.accelerations.0 * dt;
        *mover.velocity.borrow_mut() += lin_vel;
        delta.initial_velocities.0 += lin_vel;

        //let mut rotation = *mover.rotation.borrow_mut();
        *mover.angular_velocity.borrow_mut() += delta.accelerations.1 * dt;

        /* 
        
        *mover.translation.borrow_mut() += *mover.velocity.borrow() * dt;

        let delta_orientation = *mover.angular_velocity.borrow() * dt;
        *mover.rotation.borrow_mut() *= UnitQuaternion::<f64>::from_euler_angles(delta_orientation.x, delta_orientation.y, delta_orientation.z);
        */
    }
}

fn set_positions(movers: &mut MoversList, dt: f64) {
    for (m, delta, _) in movers.iter_mut() {
        let mover = m.borrow();

        *mover.translation.borrow_mut() += *mover.velocity.borrow() * dt;
        //lin_pos_change(mover.translation.borrow_mut(), mover.velocity.borrow(), dt);
        
        let delta_orientation = *mover.angular_velocity.borrow() * dt;
        *mover.rotation.borrow_mut() *= UnitQuaternion::<f64>::from_euler_angles(delta_orientation.x, delta_orientation.y, delta_orientation.x);
    }
}
fn lin_pos_change(translation: &mut Vect3, velocity: &Vect3, dt: f64) {
    *translation += *velocity * dt;
}

struct RunResult;

fn poll_actors(movers: &mut MoversList, time: f64, dt: f64) {
    let mut movers_to_poll: Vec<&MoverTuple> = vec![];
    {
    for m in movers.iter() {
        if m.2.is_none() || !m.0.borrow().is_alive {//actor
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
        let mut created_movers = Mover::decide(m.clone(), new_a, &*movers, time, dt);

        all_created_movers.append(&mut created_movers);
    }


    for new_mover in all_created_movers.into_iter() {
        let new_tuple = (Rc::new(RefCell::new(new_mover)), Deltas::new(), Option::<ActorRepresentation>::None);
        movers.push(new_tuple);
    }

    //flag_print!("poll_actors", )

}

fn decide_actor_dt(initial_dt: f64, current_dt: f64, movers: &MoversList) -> f64 {

    initial_dt
}

fn decide_movement_dt(initial_dt: f64, current_dt: f64, actor_dt: f64, movers: &MoversList) -> f64 {
    let mut living_movers: Vec<&MoverTuple> = vec![];

    for tup in movers.iter() {
        if tup.0.borrow().is_alive {
            living_movers.push(tup);
        }
    }

    let mut minimum_dt = actor_dt;

    //println!("movers len {}", movers.len());

    for i in 0..movers.len() {// (n * (n + 1))/2, if we always have less than 30 movers then this is 450 + 15.5 = 465.6 movers
        //println!("i: {}", i);
        for j in (i+1)..movers.len() {
            //println!("j: {}", j);
            let _first = movers[i].0.clone();
            let _second = movers[j].0.clone();
            let first = _first.borrow();
            let second = _second.borrow();
            if !first.is_alive && !second.is_alive {
                flag_print!("print_decide_movement_dt", "\treturned due to both being dead");
                continue;
            }
            let v1 = first.velocity.borrow();
            let v2 = second.velocity.borrow();
            let vcombined = *v1 - *v2;

            let pos1 = first.translation.borrow();
            let pos2 = second.translation.borrow();
            let diff = *pos1 - *pos2;

            let r1 = first.radius;
            let r2 = second.radius;
            let rsquared = f64::powf(r1+r2, 2.);

            let starting_distance = f64::sqrt(diff.dot(&diff));//f64::sqrt(f64::powf(pos1.x - pos2.x, 2.) + f64::powf(pos1.y - pos2.y, 2.) + f64::powf(pos1.z - pos2.z, 2.));

            if starting_distance <= r1 + r2 {
                flag_print!("print_decide_movement_dt", "\treturned due to starting_distance {} <= radii sum {} + {} = {}", starting_distance, r1, r2, r1 + r2);
                continue;
            }

            let rel_dot = Vector3::dot(&diff, &vcombined);
            let rel_v_squared = vcombined.magnitude_squared();

            if rel_dot > 0. || rel_v_squared == 0. {
                //flag_print!("print_decide_movement_dt", "\treturned due to rel_dot {} > 0 or rel_v_squared {} == 0", rel_dot, rel_v_squared);
                continue;

            }

            let min_dist_t = -rel_dot / rel_v_squared;
            let min_dist_squared = (diff + vcombined * min_dist_t).magnitude_squared();

            if min_dist_squared > rsquared {
                flag_print!("print_decide_movement_dt", "\treturned due to min_dist_squared {} >= rsquared {}", min_dist_squared, rsquared);
                continue;
            }

            let collide_dist_to_min_squared = rsquared - min_dist_squared;
            let collide_t_before_min = f64::sqrt(collide_dist_to_min_squared / rel_v_squared);

            let collision_time = min_dist_t - collide_t_before_min;

            if collision_time < minimum_dt && collision_time > 0. {
                flag_print!("print_decide_movement_dt", "decide_movement_dt() minimum set to {} from {} because movers ({} at {} moving {}) and ({} at {} moving {}) will collide sooner than normal dt. min_dist_squared = {}, min_dist_t = {}, rel_dot = {}, rel_v_squared = {}",
                    collision_time, minimum_dt, 
                    first.name, stringify_vector!(first.translation.borrow(),2), stringify_vector!(v1, 2), 
                    second.name, stringify_vector!(second.translation.borrow(),2), stringify_vector!(v2, 2),
                    min_dist_squared, min_dist_t, rel_dot, rel_v_squared);
                
                minimum_dt = collision_time;
            } else {

                flag_print!("print_decide_movement_dt", "decide_movement_dt() minimum unchanged at {} when minimum collision time was {} because movers ({} at {} moving {}) and ({} at {} moving {}) will not collide until at least dt time passes",
                    minimum_dt, collide_t_before_min, 
                    first.name, stringify_vector!(first.translation.borrow(),2), stringify_vector!(v1, 2), 
                    second.name, stringify_vector!(second.translation.borrow(),2), stringify_vector!(v2, 2));
                continue;
            }

        }
    }
    //flag_print!("print_decide_movement_dt", "minimum_dt: {}, actor: {}", minimum_dt, actor_dt);
    minimum_dt 
}

fn run(initial_dt: f64, min_tick_time: std::time::Duration, movers: &mut MoversList) -> RunResult {
    let mut time: f64 = 0.;
    let mut dt: f64 = initial_dt;

    let mut actor_dt: f64 = initial_dt;
    let mut time_until_next_actor_tick: f64 = actor_dt;

    let mut movement_dt: f64 = initial_dt;
    let mut movement_time_elapsed: f64 = 0.;

    'game: loop {
        let start_of_tick = time::Instant::now();

        actor_dt = decide_actor_dt(initial_dt, actor_dt, movers);
        time_until_next_actor_tick = actor_dt;

        poll_actors(movers, time, actor_dt);

        for e in movers.iter_mut() {
            e.1 = Deltas::new();
        }

        sum_forces(movers, actor_dt);
        set_initial_vectors(movers, actor_dt);

        movement_dt = decide_movement_dt(initial_dt, movement_dt, actor_dt, movers).min(time_until_next_actor_tick);
        
        let starting_movement_dt = movement_dt;

        movement_time_elapsed = 0.;
        
        let mut i = 0;
        'movement: loop {
            i += 1;
            if i > 1000 || movement_dt <= 0. {
                println!("----------INFINITE MOVEMENT LOOP DETECTED, QUITTING GAME----------");
                break 'game;
            }
            //println!("t={}, movement_dt = {}, actor_dt = {}, time_until_next_actor_tick = {}, starting movement_dt = {}", time, movement_dt, actor_dt, time_until_next_actor_tick, starting_movement_dt);

            set_positions(movers, movement_dt);

            find_collisions(movers, time, movement_dt);

            print_run_deltas(time, movement_dt, movers);
            
            time += movement_dt;

            if is_done(movers, time) {
                break 'game;
            }
            movement_time_elapsed += movement_dt;
            time_until_next_actor_tick -= movement_dt;


            movement_dt = movement_dt.min(time_until_next_actor_tick);

            if time_until_next_actor_tick <= 0. {
                break 'movement;
            }

            //for e in movers.iter_mut() {
            //    e.1 = Deltas::new();
            //}

        }


        let end_of_tick = time::Instant::now();

        let tick_duration = end_of_tick - start_of_tick;

        if tick_duration > min_tick_time {
            thread::sleep(tick_duration - min_tick_time);
        }

        
    }

    println!("done!");

    RunResult
}


fn print_run_deltas(time: f64, dt: f64, movers: &MoversList) {

    for m in movers.iter() {
        //let mover_1_force = format!("[{}, {}, {}]", mover.1.forces.0.x, mover.1.forces.0.y, mover.1.forces.0.z);

        let mover = m.0.borrow();

        #[cfg(feature = "print_run_deltas_any")] 
        {
            println!("run deltas:");
            let mover_ident_string = match mover.is_alive {
                true => {
                    format!("\t mover: {}", mover.name)
                },
                false => {
                    format!("\tmover: {} (DECEASED)", mover.name)
                }
            };
            println!("{}", mover_ident_string);
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
}

pub struct Photon {

}

fn create_movers_test(movers: &mut MoversList) {
    let mut first = RefCell::new(Mover::new(None, None));
    first.borrow_mut().name = "first".to_string();
    first.borrow_mut().team = "red".to_string();
    first.borrow_mut().set_thrust(1000000.);

    let mut second = RefCell::new(Mover::new(Some(Vect3_new!(300.,0.,0.)), None));
    second.borrow_mut().name = "second".to_string();
    second.borrow_mut().team = "compact".to_string();
    second.borrow_mut().set_thrust(5000.);
    second.borrow_mut().set_thrust_rotation(UnitQuaternion::<f64>::from_axis_angle(&Vect3::z_axis(), 0.5*PI));

    movers.push((Rc::new(first), Deltas::new(), None));
    movers.push((Rc::new(second), Deltas::new(), None));
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
    first.borrow_mut().team = "red".to_string();
    first.borrow_mut().radius = 5.;

    //first.borrow_mut().set_thrust(1.);

    first.borrow_mut().available_actions = Rc::new(vec![
        ActionType::ThrustAction(
            ThrustAction { rotation: None, new_thrust_force: None, new_thrust_bounds: Interval::<f64>::new(Some(0.), Some(100.)) }
        )
    ]);

    let first_ai = SimpleMissileActor{};

    push_end_print!("first's F vector: {}, thrust rotation: {}, overall rotation: {}, thrust rotation rotating [1,0,0]: {}", 
        stringify_vector!(first.borrow().F_thrust, 3), first.borrow().thrust_rotation, first.borrow().rotation.borrow(), first.borrow().thrust_rotation.transform_vector(&Vect3_new!(1.,0.,0.)));

    let second = RefCell::new(Mover::new(
        Some(Vect3_new!(100.,1050.,0.)), 
        Some(UnitQuaternion::from_axis_angle(&Vect3::z_axis(), PI)))
    );
    second.borrow_mut().name = "second".to_string();
    second.borrow_mut().team = "compact".to_string();
    //second.borrow_mut().set_thrust(1.);
    second.borrow_mut().radius = 5.;

    let first_rc = Rc::new(first);

    let third = Rc::new(RefCell::new(Mover::new(Some(Vect3_new!(-1000., -1000., -1000.)), None)));
    third.borrow_mut().team = "red".to_string();
    third.borrow_mut().name = "grace".to_string();

    movers.push( (first_rc, Deltas::new(), Some(Rc::new(RefCell::new(first_ai)))) );
    movers.push( (Rc::new(second), Deltas::new(), None) );
    movers.push((third, Deltas::new(), None));
}

fn test_shit() {
    let _x = Vect3_new!(1.,0.,0.);
    let _quat = UnitQuaternion::<f64>::from_axis_angle(&Vect3::z_axis(), 0.5*PI);
    //push_end_print!("quat {} (or rotation matrix {}) \n\tROTATES {} \n\tINTO {}", _quat, _quat.to_rotation_matrix(), stringify_vector!(_x,3), stringify_vector!(_quat * _x,3));
}

fn main() {

    let mut movers: MoversList = vec![];
    create_movers_test(&mut movers);

    let dt: f64 = STARTING_DT;
    let min_tick_time = time::Duration::from_millis((1000. / 60.) as u64);

    run(dt, min_tick_time, &mut movers);
    
    test_shit();

    unsafe {
        if !end_prints.is_empty() {
            println!("\n___END PRINTS___\n");
        }
        for string_to_print in end_prints.iter() {
            println!("{}", string_to_print);
        }
    }
}
