use std::{rc::Rc, cell::RefCell};

use num::Bounded;

use crate::{mover::Mover, globals::MoversList};



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


pub struct ObservationSpace<'a> {
    ///the self
    pub ego: Rc<RefCell<Mover>>,
    pub all_movers: &'a MoversList,
    pub start_time: f64,
    pub dt: f64,
}