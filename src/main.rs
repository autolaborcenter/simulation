use async_std::{net::UdpSocket, sync::Arc, task};
use monitor_tool::{Encoder, Vertex, RGBA};
use nalgebra::{Isometry2, Point2, Vector2};
use palette::{named, Pixel};
use pm1_control_model::{Odometry, Optimizer, Physical, StatusPredictor, TrajectoryPredictor};
use rand::Rng;
use std::{
    f32::consts::{FRAC_PI_2, FRAC_PI_8},
    time::Duration,
};

macro_rules! vertex {
    ($x:expr, $y:expr; $level:expr, $tie:expr) => {
        Vertex {
            x: $x,
            y: $y,
            dir: f32::NAN,
            level: $level,
            tie: $tie,
            _zero: 0,
        }
    };

    ($pose:expr; $level:expr, $tie:expr) => {
        Vertex {
            x: $pose.translation.vector[0],
            y: $pose.translation.vector[1],
            dir: $pose.rotation.angle(),
            level: $level,
            tie: $tie,
            _zero: 0,
        }
    };

    ($x:expr, $y:expr, $dir:expr; $level:expr, $tie:expr) => {
        Vertex {
            x: $x,
            y: $y,
            dir: $dir,
            level: $level,
            tie: $tie,
            _zero: 0,
        }
    };
}

fn main() {
    task::block_on(async {
        let socket = Arc::new(UdpSocket::bind("0.0.0.0:0").await.unwrap());
        send_config(socket.clone(), Duration::from_secs(3));

        {
            let mut encoder = Encoder::default();
            encoder.topic_clear("odometry");
            let _ = socket
                .send_to(encoder.encode().as_slice(), "127.0.0.1:12345")
                .await;
        }
        const PERIOD: Duration = Duration::from_millis(50);
        let mut odometry = Odometry::ZERO;
        let mut predictor = TrajectoryPredictor {
            period: PERIOD,
            model: Default::default(),
            predictor: StatusPredictor::new(Optimizer::new(0.5, 1.2, PERIOD), PERIOD),
        };
        predictor.predictor.target = Physical {
            speed: 0.5,
            rudder: FRAC_PI_8,
        };
        loop {
            let sign = rand::thread_rng().gen_range(-5..=5);
            let rudder = &mut predictor.predictor.target.rudder;
            let delta = if *rudder > 0.0 {
                match sign {
                    -5 | -4 => -0.1,
                    5 => 0.1,
                    _ => 0.0,
                }
            } else {
                match sign {
                    -5 => -0.1,
                    5 | 4 => 0.1,
                    _ => 0.0,
                }
            };
            *rudder = (*rudder + delta).clamp(-FRAC_PI_2, FRAC_PI_2);
            odometry += predictor.next().unwrap().1;

            let mut encoder = Encoder::default();

            encoder.topic_push("odometry", vertex!(odometry.pose; 0, false));

            encoder.topic_clear("chassis");
            for v in ROBOT_OUTLINE {
                encoder.topic_push("chassis", transform(&odometry.pose, v));
            }

            let tr = odometry.pose
                * Isometry2::new(
                    Vector2::new(-0.355, 0.0),
                    predictor.predictor.current.rudder,
                );
            for v in RUDDER {
                encoder.topic_push("chassis", transform(&tr, v));
            }

            let _ = socket
                .send_to(encoder.encode().as_slice(), "127.0.0.1:12345")
                .await;
            task::sleep(Duration::from_millis(50)).await;
        }
    });
}

fn send_config(socket: Arc<UdpSocket>, period: Duration) {
    let mut encoder = Encoder::default();
    encoder.topic_set_capacity("chassis", 100);
    encoder.topic_set_focus("chassis", 100);

    let color: [u8; 3] = named::BLACK.into_format().into_raw();
    encoder.topic_set_color("chassis", 0, RGBA(color[0], color[1], color[2], 192));
    let color: [u8; 3] = named::GOLD.into_format().into_raw();
    encoder.topic_set_color("chassis", 1, RGBA(color[0], color[1], color[2], 192));

    encoder.topic_set_capacity("odometry", 20000);
    encoder.topic_set_focus("odometry", 200);
    let color: [u8; 3] = named::DARKGREEN.into_format().into_raw();
    encoder.topic_set_color("odometry", 0, RGBA(color[0], color[1], color[2], 192));

    let packet = encoder.encode();
    task::spawn(async move {
        loop {
            let _ = socket.send_to(&packet, "127.0.0.1:12345").await;
            task::sleep(period).await;
        }
    });
}

const ROBOT_OUTLINE: [Vertex; 17] = [
    vertex!(0.25, 0.08; 0, true),
    vertex!(0.12, 0.14; 0, true),
    vertex!(0.10, 0.18; 0, true),
    vertex!(0.10, 0.26; 0, true),
    //
    vertex!(-0.10, 0.26; 0, true),
    vertex!(-0.10, 0.18; 0, true),
    vertex!(-0.25, 0.18; 0, true),
    vertex!(-0.47, 0.12; 0, true),
    //
    vertex!(-0.47, -0.12; 0, true),
    vertex!(-0.25, -0.18; 0, true),
    vertex!(-0.10, -0.18; 0, true),
    vertex!(-0.10, -0.26; 0, true),
    //
    vertex!(0.10, -0.26; 0, true),
    vertex!(0.10, -0.18; 0, true),
    vertex!(0.12, -0.14; 0, true),
    vertex!(0.25, -0.08; 0, true),
    //
    vertex!(0.25, 0.08; 0, false),
];

const RUDDER: [Vertex; 5] = [
    vertex!(-0.075, 0.06; 1, true),
    vertex!(-0.075, -0.06; 1, true),
    vertex!(0.075, -0.06; 1, true),
    vertex!(0.075, 0.06; 1, true),
    //
    vertex!(-0.075, 0.06; 1, false),
];

fn transform(tr: &Isometry2<f32>, mut vertex: Vertex) -> Vertex {
    let p = Point2::new(vertex.x, vertex.y);
    let p = tr * p;
    vertex.x = p[0];
    vertex.y = p[1];
    if vertex.dir.is_finite() {
        vertex.dir += tr.rotation.angle();
    }
    vertex
}
