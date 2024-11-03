use crate::acc_config;
use std::{error, fmt};

// impl<E: core::fmt::Debug> error::Error for AnyError {}

// impl<E: core::fmt::Debug> fmt::Display for AnyError {
//     fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
//         write!(f, "{:?}", self)
//     }
// }

impl error::Error for acc_config::Error {}

impl fmt::Display for acc_config::Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}