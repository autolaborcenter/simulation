use async_std::{net::UdpSocket, sync::Arc, task};
use monitor_tool::{Encoder, Vertex, RGBA};
use nalgebra::{Isometry2, Point2, Vector2};
use palette::{named, Pixel};
use pm1_control_model::{Odometry, Optimizer, Physical, StatusPredictor, TrajectoryPredictor};
use rand::Rng;
use std::{
    f32::consts::{FRAC_PI_2, FRAC_PI_8},
    time::{Duration, Instant},
};

macro_rules! vertex {
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
    ($x:expr, $y:expr; $level:expr, $tie:expr) => {
        vertex!($x, $y, f32::NAN; $level, $tie)
    };
    ($pose:expr; $level:expr, $tie:expr) => {
        vertex!($pose.translation.vector[0],
                $pose.translation.vector[1],
                $pose.rotation.angle();
                $level, $tie)
    };
}

macro_rules! pose {
    ($x:expr, $y:expr; $theta:expr) => {
        Isometry2::new(Vector2::new($x, $y), $theta)
    };
}

const CHASSIS_TOPIC: &str = "chassis";
const ODOMETRY_TOPIC: &str = "odometry";

fn main() {
    task::block_on(async {
        let socket = Arc::new(UdpSocket::bind("0.0.0.0:0").await.unwrap());
        send_config(socket.clone(), Duration::from_secs(3));

        const PERIOD: Duration = Duration::from_millis(33);
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

        let mut time = Instant::now();
        loop {
            // 更新
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
            // 编码并发送
            let socket = socket.clone();
            task::spawn(async move {
                let packet = Encoder::with(|encoder| {
                    encoder
                        .topic(ODOMETRY_TOPIC)
                        .push(vertex!(odometry.pose; 0, false));
                    encoder.with_topic(CHASSIS_TOPIC, |mut chassis| {
                        chassis.clear();
                        ROBOT_OUTLINE
                            .iter()
                            .for_each(|v| chassis.push(transform(&odometry.pose, *v)));

                        let tr =
                            odometry.pose * pose!(-0.355, 0.0; predictor.predictor.current.rudder);
                        RUDDER.iter().for_each(|v| chassis.push(transform(&tr, *v)));
                    });
                });
                let _ = socket.send_to(&packet, "127.0.0.1:12345").await;
            });
            // 延时到下一周期
            time += PERIOD;
            if let Some(dur) = time.checked_duration_since(Instant::now()) {
                task::sleep(dur).await;
            }
        }
    });
}

fn send_config(socket: Arc<UdpSocket>, period: Duration) {
    let packet = Encoder::with(|encoder| {
        encoder.with_topic(CHASSIS_TOPIC, |mut chassis| {
            chassis.set_capacity(100);
            chassis.set_focus(100);

            let color: [u8; 3] = named::BLACK.into_format().into_raw();
            chassis.set_color(0, RGBA(color[0], color[1], color[2], 192));
            let color: [u8; 3] = named::GOLD.into_format().into_raw();
            chassis.set_color(1, RGBA(color[0], color[1], color[2], 192));
        });
        encoder.with_topic(ODOMETRY_TOPIC, |mut odometry| {
            odometry.set_capacity(20000);
            odometry.set_focus(200);
            let color: [u8; 3] = named::DARKGREEN.into_format().into_raw();
            odometry.set_color(0, RGBA(color[0], color[1], color[2], 192));
        });
    });
    task::spawn(async move {
        let clear = Encoder::with(|encoder| encoder.topic(ODOMETRY_TOPIC).clear());
        let _ = socket.send_to(clear.as_slice(), "127.0.0.1:12345").await;
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
