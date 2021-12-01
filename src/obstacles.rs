use monitor_tool::{vertex, Vertex};

pub(super) const OBSTACLES_TOPIC: &str = "obstacles";

pub(super) const TRICYCLE_OUTLINE: [Vertex; 6] = [
    vertex!(0; 1.0, 0.0; 0),
    vertex!(0; 0.0, 0.75; 64),
    vertex!(0; -2.0, 0.75; 64),
    vertex!(0; -2.0, -0.75; 64),
    vertex!(0; 0.0, -0.75; 64),
    //
    vertex!(0; 1.0, 0.0; 64),
];
