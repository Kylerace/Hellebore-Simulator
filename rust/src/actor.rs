

use std::{rc::Rc, cell::RefCell};

use nalgebra::{UnitQuaternion, Vector3, Rotation3, Unit};

use crate::{Vect3, mover::{Mover}, ObservationSpace, globals::{MoverTuple, PI}, action::{ActionType, ThrustAction, ThrustActionRotationChoice, Action}};


pub trait Actor { 
    fn decide<'a, 'b: 'a>(&'a mut self, observation: &ObservationSpace, actions: &Vec<(Rc<RefCell<Mover>>, Rc<Vec<ActionType>>)>) -> Vec<Mover>;
}

#[derive(Clone, Debug)]
pub struct SimpleMissileActor {

}


impl_TraitEnumToBox!(
    ActorType, Actor,
    SimpleMissileActor(SimpleMissileActor),
);

impl Actor for SimpleMissileActor {
    fn decide<'a, 'b: 'a>(&'a mut self, observation: &ObservationSpace, actions: &Vec<(Rc<RefCell<Mover>>, Rc<Vec<ActionType>>)>) -> Vec<Mover> {
        let mut output: Vec<Mover> = vec![];

        //flag_print!("print_SimpleMissileActor", "ZERO, action len: {}", actions.len());

        let mut thrust_action: Option<(Rc<RefCell<Mover>>, ThrustAction)> = None;
        'outer: for mover_actions in actions.iter() {
            for action in mover_actions.1.iter() {
                if let ActionType::ThrustAction(t) = action {
                    thrust_action = Some((mover_actions.0.clone(), t.clone()));
                    break 'outer;
                }
            }
        }

        if thrust_action.is_none() {
            return output;
        }

        //flag_print!("print_SimpleMissileActor", "ONE");

        let our_team = observation.ego.borrow().team.clone();
        let mut enemies: Vec<&MoverTuple> = vec![];
        for mover in observation.all_movers {
            if mover.0.borrow().team != our_team && mover.0.borrow().is_alive {
                enemies.push(mover);
            }
        }

        if enemies.is_empty() {
            return output;
        }

        //find closest enemy, then rotate thrust towards them and set to max thrust
        let mut closest_enemy: Option<Rc<RefCell<Mover>>> = None;
        let mut closest_enemy_distance = f64::MAX;

        let _our_pos = observation.ego.borrow();
        let our_pos = _our_pos.translation.borrow();

        for enemy in enemies {
            let enemy_mover = enemy.0.borrow();
            let enemy_pos = enemy_mover.translation.borrow();

            let distance = f64::sqrt(f64::powf(our_pos.x - enemy_pos.x, 2.) + f64::powf(our_pos.y - enemy_pos.y, 2.) + f64::powf(our_pos.z - enemy_pos.z, 2.));
            if distance < closest_enemy_distance {
                closest_enemy_distance = distance;
                closest_enemy = Some(enemy.0.clone());
            }
        }

        if closest_enemy.is_none() {
            return output;
        }

        //flag_print!("print_SimpleMissileActor", "THREE");
        
        //im not entirely sure how to make this work for constrained thrust thats only in child movers
        //for this though i think we can just find an ideal velocity vector and rotate our thrust such that it will eventually cancel out the difference between 
        //mover's current velocity and the ideal velocity and thus hit the target 

        //from us to them
        let us = observation.ego.borrow();
        let our_translation = us.translation.borrow();
        
        let closest_enemy_ref = closest_enemy.as_ref().unwrap().borrow();
        let their_translation = closest_enemy_ref.translation.borrow();

        let pos_delta = *their_translation - *our_translation;//Vector3::<f64>::new(our_translation.x - their_translation.x, our_translation.y - their_translation.y, our_translation.z - their_translation.z);

        let normed_pos_delta = pos_delta.clone().normalize();
        //heading, the rotation from this to pos_delta is the rotation that thrust_rotation needs to be set to for F_thrust to push us towards the enemy
        let missile_rotated_x = us.rotation.borrow().transform_vector(&Vect3::x()).normalize();
        
        //set rotation to the quaternion that rotates missile_rotated_x to normed_pos_delta

        let direction_to_enemy = UnitQuaternion::rotation_between(&missile_rotated_x, &normed_pos_delta);

        let new_thrust_rotation = match direction_to_enemy {//observation.ego.borrow().thrust_rotation.inverse() * to_closest;
            Some(q) => {
                q
            },
            None => {//just invert thrust_rotation

                //println!("AAAAAAAAAAAAAAAAAAAA inverse is {}", observation.ego.borrow().thrust_rotation.inverse());
                //observation.ego.borrow().thrust_rotation.inverse()

                let mut nonzero_value_main: Option<f64> = None;
                let mut other_val1 = 0.;
                let mut other_val2 = 0.;
                
                for (i, value)  in missile_rotated_x.iter().enumerate() {//this is GARBAGE BRO
                    if *value != 0. {
                        nonzero_value_main = Some(*value);
                        match i {
                            0 => {
                                other_val1 = missile_rotated_x[1];
                                other_val2 = missile_rotated_x[2];
                                break;
                            },
                            1 => {
                                other_val2 = missile_rotated_x[2];
                                break;
                            },
                            _ => {}
                        }
                    } else {
                        match i {
                            0 => {
                                other_val1 = missile_rotated_x[0];
                            },
                            1 => {
                                other_val2 = missile_rotated_x[1];
                            },
                            _ => {}
                        }
                    }
                }

                let new_axis = Vect3_new!(1.,1., -(other_val1 + other_val2)/nonzero_value_main.unwrap());
                let new_rotation = UnitQuaternion::from_axis_angle(&Unit::<Vect3>::new_normalize(new_axis), PI);

                println!("AAAAAAAAAAAAA rotated is {}, axis is {}", new_rotation, new_axis);

                new_rotation
            }
        };

        let mut return_thrust_action = thrust_action.as_ref().unwrap().1.clone();
        return_thrust_action.rotation = Some(ThrustActionRotationChoice::SetRotation(new_thrust_rotation));
        return_thrust_action.new_thrust_force = Some(return_thrust_action.new_thrust_bounds.high);

        output.append(&mut return_thrust_action.perform_on(observation.ego.clone()));
        //output.push((thrust_action.unwrap().0, vec![ActionType::ThrustAction(return_thrust_action)]));

        flag_print!("print_SimpleMissileActor", "SimpleMissileActor {} at {}, closest enemy: {} at {}, pos delta {}\n\tangle from us to them is {}\n\trotation matrix is {}", 
            observation.ego.borrow().name, stringify_vector!(our_translation, 3), closest_enemy_ref.name, stringify_vector!(their_translation, 3), stringify_vector!(pos_delta, 3), new_thrust_rotation, new_thrust_rotation.to_rotation_matrix());
        return output;
    }
}