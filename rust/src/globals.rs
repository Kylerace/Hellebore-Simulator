

pub const X: usize = 0;
pub const Y: usize = 1;
pub const Z: usize = 2;

pub const ROLL: usize = 0;
pub const PITCH: usize = 1;
pub const YAW: usize = 2;

pub const DEFAULT_COSMIC_SPEED_LIMIT: f64 = 299_792_458.0;
pub const DEFAULT_SPEED_OF_LIGHT: f64 = DEFAULT_COSMIC_SPEED_LIMIT;
pub const PI: f64 = std::f64::consts::PI;
pub const MAX_DURATION: f64 = 5.;

pub static C_S_L: f64 = DEFAULT_COSMIC_SPEED_LIMIT;
pub static C_V: f64 = DEFAULT_SPEED_OF_LIGHT;

pub static mut end_prints: Vec<String> = vec![];
pub static mut outs_to_use: Vec<String> = vec![];


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

#[macro_export]
macro_rules! Vect3_new {
    () => {
        Vect3::new(0.,0.,0.)
    };
    ($x:expr, $y:expr, $z:expr) => {
        Vect3::new($x,$y,$z)
    };
}
#[macro_export]
macro_rules! stringify_vector {
    ($vector:expr) => {
        format!("[{}, {}, {}]", $vector.x, $vector.y, $vector.z)
    };
    ($vector:expr, $precision:literal) => {
        format!("[{:.3$}, {:.3$}, {:.3$}]", $vector.x, $vector.y, $vector.z, $precision)
    };
}
#[macro_export]
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
#[macro_export]
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