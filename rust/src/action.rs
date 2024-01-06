use std::{rc::Rc, cell::RefCell};
use nalgebra::UnitQuaternion;

use crate::{Mover, mover::{Interval, Space}};

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