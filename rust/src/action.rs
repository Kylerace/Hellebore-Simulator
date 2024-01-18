use std::{rc::{Rc, Weak}, cell::RefCell};
use nalgebra::UnitQuaternion;

use crate::{Mover, Interval, Space, globals::{ActorRepresentation, Vect3}};

pub trait Action {
    /// create a new mover by returning one in the vector, otherwise just affect mover 
    fn perform_on(&self, mover: Rc<RefCell<Mover>>) -> Vec<(Mover, Option<ActorRepresentation>)>;
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
    fn perform_on(&self, mover: Rc<RefCell<Mover>>) -> Vec<(Mover, Option<ActorRepresentation>)> {
        
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
    
    fn perform_on(&self, mover: Rc<RefCell<Mover>>) -> Vec<(Mover, Option<ActorRepresentation>)> {
        println!("TestAction with x = {} performed on mover {}", self.x, mover.borrow().name);

        vec![]
    }
}

pub struct ShootProjectileAction {
    pub projectile: Mover,
    pub projectile_actor: Option<ActorRepresentation>,
    pub target: ProjectileTarget,
    /// the mover that contains our gun
    pub top_level_mover_center: Vect3,
    pub top_level_mover_radius: f64,
    ///what we shoot from, if it's not a top level mover we set its rotation to our rotation and then shoot
    pub gun_center: Vect3,
    pub gun_radius: f64,
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

pub enum ProjectileTargetAimBehavior {
    Direct,
    LeadVelocity,
    //LeadAcceleration
}

#[derive(Clone)]
pub enum ProjectileTarget {
    Mover(Weak<RefCell<Mover>>),
    Point(Vect3)
}

impl ShootProjectileAction {
    fn try_create(top_level_mover: Rc<RefCell<Mover>>, gun: Rc<RefCell<Mover>>, target: ProjectileTarget, projectile: Mover,
            aim_behavior: ProjectileTargetAimBehavior, added_velocity: f64) -> Option<Self> {

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


        let mut target_point: Vect3;
        let mut relative_position: Vect3;
        let mut relative_velocity: Vect3 = Vect3_new!();
        //let mut target_weakref: Option<Weak<RefCell<Mover>>> = None;
        match target {
            ProjectileTarget::Mover(ref w) => {
                //target_weakref = Some(w.clone());
                if let Some(target_ref) = w.upgrade() {
                    target_point = target_ref.borrow().translation.borrow().clone();
                   

                    match aim_behavior {
                        ProjectileTargetAimBehavior::Direct => {

                        },
                        ProjectileTargetAimBehavior::LeadVelocity => {
                            relative_velocity = target_ref.borrow().velocity.borrow().clone() - top_level_mover.borrow().velocity.borrow().clone();
                        },
                    }

                    relative_position = target_point - origin;
                } else {
                    return None;
                }
            },
            ProjectileTarget::Point(p) => {
                target_point = p;
                relative_position = target_point - origin;
            },
        }

        let unit_vector_to_target: Vect3 = relative_position.normalize();
        let projectile_velocity = unit_vector_to_target * added_velocity + relative_velocity;

        projectile.velocity.replace(projectile_velocity);

        let distance_to_place_projectile = (top_level_mover.borrow().radius + projectile.radius) * 1.5;

        projectile.translation.replace(unit_vector_to_target * distance_to_place_projectile);
        //projectile.rotation.replace(); //TODO: figure out how to rotate the projectile towards the target

        let us = ShootProjectileAction {
            projectile, 
            projectile_actor: None, 
            target: target.clone(),
            top_level_mover_center: *top_level_mover.borrow().translation.borrow(),
            top_level_mover_radius: top_level_mover.borrow().radius,
            gun_center: origin,
            gun_radius: top_level_mover.borrow().radius,
        };

        Some(us)

    }
}


impl Action for ShootProjectileAction {
    fn perform_on(&self, mover: Rc<RefCell<Mover>>) -> Vec<(Mover, Option<ActorRepresentation>)> {
        vec![(self.projectile, self.projectile_actor)]
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