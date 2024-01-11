use std::{rc::Rc, cell::RefCell};
use nalgebra::UnitQuaternion;

use crate::{Mover, Interval, Space, globals::{ActorRepresentation, Vect3}};

pub trait Action {
    /// create a new mover by returning one in the vector, otherwise just affect mover 
    fn perform_on(&self, mover: Rc<RefCell<Mover>>) -> Vec<Mover>;
}

/**
 * so that ThrustAction can be used to set thrust_rotation directly or to rotate it
 */
#[derive(Clone, Debug)]
pub enum ThrustActionRotationChoice {
    RotateBy(UnitQuaternion<f64>),
    SetRotation(UnitQuaternion<f64>)
}

#[derive(Clone, Debug)]
pub struct ThrustAction {
    ///rotation we apply to thrust_rotation
    pub rotation: Option<ThrustActionRotationChoice>, //TODO: figure out how to constrain rotations
    pub new_thrust_force: Option<f64>,
    ///max and min values
    pub new_thrust_bounds: Interval<f64>
}

impl ThrustAction {
    fn new(thrust_bounds: Option<Interval<f64>>) -> Self {
        let thrust_bounds = thrust_bounds.unwrap_or(Interval::<f64>::new(Some(0.), None));
        ThrustAction {
            rotation: None,
            new_thrust_force: None,
            new_thrust_bounds: thrust_bounds,
        }
    }
}

impl Action for ThrustAction {
    fn perform_on(&self, mover: Rc<RefCell<Mover>>) -> Vec<Mover> {
        
        //let bmover = mover.borrow_mut();
        #[cfg(feature = "print_ThrustAction_perform_on")]
        let mut data: Vec<String> = vec![];

        {
            if let Some(t) = self.new_thrust_force {
                #[cfg(feature = "print_ThrustAction_perform_on")]
                data.push(format!("current_thrust changed from {} to {}", mover.borrow().current_thrust, t));

                mover.borrow_mut().set_thrust(t);
            }
        }
        match self.rotation {
            Some(ThrustActionRotationChoice::RotateBy(r)) => {
                #[cfg(feature = "print_ThrustAction_perform_on")]
                let old = mover.borrow().thrust_rotation.clone();

                mover.borrow_mut().rotate_thrust(r);

                #[cfg(feature = "print_ThrustAction_perform_on")]
                data.push(format!("thrust_rotation rotated by {} from {} to {}", r, old, mover.borrow().thrust_rotation.clone()));
            }
            Some(ThrustActionRotationChoice::SetRotation(r)) => {
                #[cfg(feature = "print_ThrustAction_perform_on")]
                data.push(format!("thrust_rotation changed from {} to {}", mover.borrow().thrust_rotation, r));

                mover.borrow_mut().set_thrust_rotation(r);
            }
            None => {}
        }

        #[cfg(feature = "print_ThrustAction_perform_on")]
        {
            let mut concat_data = "".to_string();
            for data_str in data.iter() {
                concat_data.push_str(&format!(" {},", data_str));
            }
            println!("\tThrustAction::perform_on({}) did:{}", mover.borrow().name, concat_data);
        }

        vec![]
    }
}

impl Space for ThrustAction {
    type ElementType = (Option<UnitQuaternion<f64>>, Option<f64>);

    fn contains(&self, point: &Self::ElementType) -> bool {
        match point.1 {
            None => true,
            Some(t) => {
                self.new_thrust_bounds.contains(&t)
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct TestAction {
    pub x: usize
}

impl Action for TestAction {
    
    fn perform_on(&self, mover: Rc<RefCell<Mover>>) -> Vec<Mover> {
        println!("TestAction with x = {} performed on mover {}", self.x, mover.borrow().name);

        vec![]
    }
}

pub struct ShootProjectileAction {
    pub projectile: Option<Mover>,
    pub projectile_actor: Option<ActorRepresentation>,
    pub target_point: Vect3,
    /// the mover that contains our gun
    pub top_level_mover_center: Vect3,
    pub top_level_mover_radius: f64,
    ///what we shoot from, if it's not a top level mover we set its rotation to our rotation and then shoot
    pub gun_center: Vect3,
    pub gun_radius: f64,
    /// when we create the projectile we create it (radius of gun + radius of projectile) * this 
    /// distance away from the gun, if that is inside the top level mover then we dont create the mover
    pub magic_shot_distance_coefficient: f64,
    /// velocity added to that of the top level mover's
    pub initial_relative_velocity: Vect3,
}

///returns the path from our_level to target
fn search_mover_child_tree(our_level: Rc<RefCell<Mover>>, target: Rc<RefCell<Mover>>) -> Option<Vec<Rc<RefCell<Mover>>>> {//TODO: move somewhere not stupid
    if Rc::ptr_eq(&our_level, &target) {
        return Some(vec![target.clone()]);
    }
    let mut path_down: Vec<Rc<RefCell<Mover>>> = vec![];
    for child in our_level.borrow().children.iter() {
        let mut child_path = search_mover_child_tree(child.clone(), target.clone());

        if let Some(mut v) = child_path {
            path_down.push(our_level.clone());
            path_down.append(&mut v);
            return Some(path_down);
        }
    }

    if path_down.is_empty() {
        None
    } else {
        Some(path_down)
    }
}

impl ShootProjectileAction {
    fn try_create(top_level_mover: Rc<RefCell<Mover>>, gun: Rc<RefCell<Mover>>, target_point: Vect3, added_velocity: f64) -> Option<Self> {
        let gun_is_top_level = Rc::ptr_eq(&top_level_mover, &gun);
        let mut origin: Vect3 = *top_level_mover.borrow().translation.borrow();
        if !gun_is_top_level {
            //let mut nested_path_to_gun: Option<Vec<Rc<RefCell<Mover>>>> = search_mover_child_tree(top_level_mover.clone(), gun.clone());
            if let Some(nested_path_to_gun) = search_mover_child_tree(top_level_mover.clone(), gun.clone()) {
                let mut last_mover: Option<Rc<RefCell<Mover>>> = None;
                for mover in nested_path_to_gun.iter() {
                    if let Some(l_m) = last_mover {
                        origin += l_m.borrow().rotation.borrow().transform_vector(&mover.borrow().translation.borrow());
                    }
                    last_mover = Some(mover.clone());
                }
            } else {
                println!("UNABLE TO FIND GUN {} INSIDE TOP LEVEL MOVER {}", gun.borrow().name, top_level_mover.borrow().name);
                return None;
            }
        }
        let relative_position = target_point - origin;
        let mut unit_vector_to_target: Vect3 = relative_position.normalize();

        let mut us = ShootProjectileAction {
            projectile: None, 
            projectile_actor: None, 
            target_point: target_point,
            top_level_mover_center: *top_level_mover.borrow().translation.borrow(),
            top_level_mover_radius: top_level_mover.borrow().radius,
            gun_center: origin,
            gun_radius: gun.borrow().radius,
            magic_shot_distance_coefficient: 1.1,

        }


    }
}


impl Action for ShootProjectileAction {
    fn perform_on(&self, mover: Rc<RefCell<Mover>>) -> Vec<Mover> {
        vec![]
    }
}

//#[derive(Clone)]
//pub enum ActionType {
//    ThrustAction(ThrustAction),
//    TestAction(TestAction)
//}

impl_TraitEnumToBox!(
    ActionType, Action,
    ThrustAction(ThrustAction),
    TestAction(TestAction),
);

fn test() {
    let mut x: Vec<Box<ActionType>> = vec![Box::new(ActionType::ThrustAction(ThrustAction::new(None)))];
    let one = &x[0];

    match &**one {
        ActionType::ThrustAction(t) => {
            
        } 
        _ => {

        }
    }

    let y = Box::new(ActionType::ThrustAction(ThrustAction::new(None)));
    if let ActionType::ThrustAction(t) = *y {

    }
}