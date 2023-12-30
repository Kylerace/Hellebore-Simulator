
use std::{rc::Rc, cell::RefCell};
use nalgebra::{UnitQuaternion, Vector3, Rotation3};
use crate::{Vect3, globals::{PI, C_S_L}};


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
    /// the force of thrust vector with rotation vector of <0,0,0>, you need to multiply this by a rotation matrix of rotation's euler angles to get 
    /// the true force of thrust vector in body coordinates.
    /// 
    /// linear F = self's rotation matrix * F_thrust of self + sigma(child's rotation matrix * child's F_thrust).
    /// 
    /// torque = sigma(torque of each child + child's translation.cross(child's F)).
    pub F_thrust: Vect3,

    pub name: String,

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
