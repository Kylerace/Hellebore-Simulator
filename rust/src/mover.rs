
use std::{rc::Rc, cell::RefCell, marker::Tuple, collections::HashMap, hash::Hash};
use nalgebra::{UnitQuaternion, Vector3, Rotation3, Unit};
use crate::{Vect3, globals::{PI, C_S_L, MoverTuple}, MoversList, Deltas};
use num::Bounded;

pub trait Space {
    type ElementType;

    /**
     * returns an element of this space
     */
    //fn sample(&self) -> Self::ElementType;

    fn contains(&self, point: &Self::ElementType) -> bool;
}

#[derive(Clone, Debug)]
pub struct Interval<T: Bounded + PartialOrd> {
    pub low: T,
    pub high: T
}

impl<T: Bounded + PartialOrd> Interval<T> {
    pub fn new(low: Option<T>, high: Option<T>) -> Self {
        let use_low = match low {
            Some(l) => {
                l
            }
            None => {
                T::min_value()
            }
        };
        let use_high = match high {
            Some(h) => {
                h
            }
            None => {
                T::max_value()
            }
        };

        Interval::<T> {
            low: use_low,
            high: use_high
        }
    }
}

impl<T: Bounded + PartialOrd> Space for Interval<T> {
    type ElementType = T;

    fn contains(&self, point: &T) -> bool {
        *point >= self.low && *point <= self.high
    }
}

pub struct IntervalProduct<D: Bounded + PartialOrd, const S: usize> {
    pub bounds: [Interval<D>; S]
}

impl<D: Bounded + PartialOrd, const S: usize> IntervalProduct<D, S> {
    fn new(input: [(Option<D>, Option<D>); S]) -> Self {
        let bounds: [Interval<D>; S] = input.map(|x| Interval::<D>::new(x.0, x.1));

        IntervalProduct::<D, S> {
            bounds
        }
    }
}

impl<D: Bounded + PartialOrd, const S: usize> Space for IntervalProduct<D, S> {
    type ElementType = [D; S];

    fn contains(&self, point: &Self::ElementType) -> bool {
        for (our_bound, their_value) in self.bounds.iter().zip(point.iter()) {
            if !our_bound.contains(their_value) {
                return false;
            }
        }
        true
    }
}

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
macro_rules! impl_TraitEnumToBox{($enum_name: ident, $trait_name: ident, $($enumvariant: ident($foo: ty),)*) => {
    #[derive(Clone, Debug)]
    pub enum $enum_name {
        $($enumvariant($foo),)*
    }
    impl $enum_name {
        pub fn into_box(self) -> Box<dyn $trait_name> {
            match self {
                $($enum_name::$enumvariant(foo) => Box::new(foo),)*
            }
        }
    }
}}

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

pub struct ObservationSpace<'a> {
    ///the self
    pub ego: Rc<RefCell<Mover>>,
    pub all_movers: &'a MoversList,
    pub start_time: f64,
    pub dt: f64,
}

pub trait Actor { 
    fn decide<'a, 'b: 'a>(&'a mut self, observation: &ObservationSpace, actions: &Vec<(Rc<RefCell<Mover>>, Rc<Vec<ActionType>>)>) -> Vec<(Rc<RefCell<Mover>>, Vec<ActionType>)>;
}

#[derive(Clone, Debug)]
pub struct SimpleMissileActor {

}


impl_TraitEnumToBox!(
    ActorType, Actor,
    SimpleMissileActor(SimpleMissileActor),
);

impl Actor for SimpleMissileActor {
    fn decide<'a, 'b: 'a>(&'a mut self, observation: &ObservationSpace, actions: &Vec<(Rc<RefCell<Mover>>, Rc<Vec<ActionType>>)>) -> Vec<(Rc<RefCell<Mover>>, Vec<ActionType>)> {
        let mut output: Vec<(Rc<RefCell<Mover>>, Vec<ActionType>)> = vec![];

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
            if mover.0.borrow().team != our_team {
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
        let _our_translation = observation.ego.borrow();
        let our_translation = _our_translation.translation.borrow();
        
        let closest_enemy_ref = closest_enemy.as_ref().unwrap().borrow();
        let their_translation = closest_enemy_ref.translation.borrow();

        let pos_delta = Vector3::<f64>::new(our_translation.x - their_translation.x, our_translation.y - their_translation.y, our_translation.z - their_translation.z);
        let to_closest = UnitQuaternion::face_towards(&pos_delta, &Vector3::x());

        let new_thrust_rotation = to_closest;//observation.ego.borrow().thrust_rotation.inverse() * to_closest;

        let mut return_thrust_action = thrust_action.as_ref().unwrap().1.clone();
        return_thrust_action.rotation = Some(ThrustActionRotationChoice::SetRotation(new_thrust_rotation));
        return_thrust_action.new_thrust_force = Some(return_thrust_action.new_thrust_bounds.high);

        output.push((thrust_action.unwrap().0, vec![ActionType::ThrustAction(return_thrust_action)]));

        flag_print!("print_SimpleMissileActor", "SimpleMissileActor {} at {}, closest enemy: {} at {}, angle from us to them is {}, we will now set thrust_rotation to {}", 
            observation.ego.borrow().name, stringify_vector!(our_translation, 3), closest_enemy_ref.name, stringify_vector!(their_translation, 3), to_closest, new_thrust_rotation);
        return output;
    }
}

#[derive(Debug)]
pub struct Mover {
    pub children: Vec<Rc<RefCell<Mover>>>,

    ///relative to the origin of the space it is operating in, outer space for top level Mover's and the center of parent for recursive Mover's
    pub translation: RefCell<Vect3>,
    pub rotation: RefCell<UnitQuaternion<f64>>,

    pub velocity: RefCell<Vect3>,
    pub angular_velocity: RefCell<Vect3>,

    pub rest_mass: f64,
    pub rel_mass: f64,

    pub radius: f64,

    pub health: f64,
    pub is_alive: bool,

    pub current_thrust: f64,
    pub thrust_rotation: UnitQuaternion<f64>,

    /// linear F = self's rotation matrix * F_thrust of self + sigma(child's rotation matrix * child's F_thrust).
    /// 
    /// torque = sigma(torque of each child + child's translation.cross(child's F)).
    pub F_thrust: Vect3,

    pub name: String,

    pub available_actions: Rc<Vec<ActionType>>,

    pub team: String
}

impl Hash for Mover {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.name.hash(state);
    }
}

impl Mover {
    pub fn new(translation: Option<Vect3>, rotation: Option<UnitQuaternion<f64>>) -> Self {
        let translation = translation.unwrap_or(Vect3_new!());
        let _original = rotation.clone();
        let rotation = rotation.unwrap_or(UnitQuaternion::<f64>::from_axis_angle(&Vector3::<f64>::z_axis(), 0. * PI));//

        Mover{
            children: vec![],
            translation: RefCell::new(translation),
            rotation: RefCell::new(rotation),

            velocity: RefCell::new(Vect3::new(0.,0.,0.)),
            angular_velocity: RefCell::new(Vect3_new!()),
            
            rest_mass: 1.,
            rel_mass: 1.,
            radius: 1.,
            health: 1.,
            is_alive: true,
            current_thrust: 0.,
            thrust_rotation: UnitQuaternion::<f64>::from_axis_angle(&Vector3::<f64>::z_axis(), 0. * PI),
            F_thrust: Vect3_new!(),
            name: "unnamed".to_string(),
            available_actions: Rc::new(vec![]),
            team: "A".to_string()
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

    fn get_recursive_children(ego: Rc<RefCell<Mover>>) -> Vec<Rc<RefCell<Mover>>> {
        let mut children = ego.borrow().children.clone();
        children.push(ego.clone());
        for child in ego.borrow().children.iter() {
            let x = &Mover::get_recursive_children(child.clone());
            if x.is_empty() {
                continue;
            }
            children.append(&mut x.clone());
        }

        children
    }

    pub fn get_recursive_actions(ego: Rc<RefCell<Mover>>) -> Vec<(Rc<RefCell<Mover>>, Rc<Vec<ActionType>>)> {
        let mut recursive_actions: Vec<(Rc<RefCell<Mover>>, Rc<Vec<ActionType>>)> = vec![];
        let recursive_children = Mover::get_recursive_children(ego.clone());

        for child in recursive_children.iter() {
            let actions = child.borrow().available_actions.clone();
            recursive_actions.push((child.clone(), actions));
        }
 
        recursive_actions
    }

    pub fn decide(ego: Rc<RefCell<Mover>>, actor: Rc<RefCell<dyn Actor>>, movers: &MoversList, start_time: f64, dt: f64) -> Vec<Mover> {
        let observation = ObservationSpace {
            ego: ego.clone(),
            all_movers: movers,
            start_time,
            dt
        };
        
        
        let recursive_actions: Vec<(Rc<RefCell<Mover>>, Rc<Vec<ActionType>>)> = Mover::get_recursive_actions(ego.clone());

        let decided_actions = actor.borrow_mut().decide(&observation, &recursive_actions);
        let mut all_created_movers: Vec<Mover> = vec![];

        for mover_action_pair in decided_actions.into_iter() {
            for action in mover_action_pair.1.into_iter().map(|a| a.into_box()) {
                let mut new_movers = action.perform_on(mover_action_pair.0.clone());
                all_created_movers.append(&mut new_movers);
            }
        }

        all_created_movers
    }

}
