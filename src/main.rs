use async_std::{net::UdpSocket, sync::Arc, task};
use monitor_tool::{Encoder, Shape, Vertex};
use nalgebra::{Isometry2, Point2, Vector2};
use palette::Srgba;
use pm1_control_model::{Odometry, Optimizer, Physical, StatusPredictor, TrajectoryPredictor};
use std::{
    f32::consts::FRAC_PI_8,
    time::{Duration, Instant},
};

macro_rules! vertex {
    ($level:expr; $x:expr, $y:expr; $alpha:expr; $shape:ident, $extra:expr) => {
        monitor_tool::Vertex {
            x: $x,
            y: $y,
            level: $level,
            alpha: $alpha,
            _zero: 0,
            shape: monitor_tool::Shape::$shape,
            extra: $extra,
        }
    };
    ($level:expr; $x:expr, $y:expr; $alpha:expr) => {
        vertex!($level; $x, $y; $alpha; Arrow, f32::NAN)
    };
    ($level:expr; $pose:expr; $alpha:expr) => {
        vertex!($level;
                $pose.translation.vector[0],
                $pose.translation.vector[1];
                $alpha;
                Arrow, $pose.rotation.angle())
    };
}

mod chassis;

use chassis::{random_rudder, CHASSIS_TOPIC, ODOMETRY_TOPIC, ROBOT_OUTLINE, RUDDER};

macro_rules! pose {
    ($x:expr, $y:expr; $theta:expr) => {
        Isometry2::new(Vector2::new($x, $y), $theta)
    };
}

const LIGHT_TOPIC: &str = "light";

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
            random_rudder(&mut predictor.predictor.target.rudder);
            odometry += predictor.next().unwrap().1;
            // 编码并发送
            let socket = socket.clone();
            task::spawn(async move {
                let packet = Encoder::with(|encoder| {
                    encoder
                        .topic(ODOMETRY_TOPIC)
                        .push(vertex!(0; odometry.pose; 0));
                    encoder.with_topic(CHASSIS_TOPIC, |mut chassis| {
                        chassis.clear();
                        ROBOT_OUTLINE
                            .iter()
                            .for_each(|v| chassis.push(transform(&odometry.pose, *v)));

                        let tr =
                            odometry.pose * pose!(-0.355, 0.0; predictor.predictor.current.rudder);
                        RUDDER.iter().for_each(|v| chassis.push(transform(&tr, *v)));
                    });
                    encoder.with_topic(LIGHT_TOPIC, |mut light| {
                        light.clear();
                        light.push(transform(
                            &odometry.pose,
                            vertex!(0; 1.0, 0.0; 0; Circle, 1.0),
                        ));
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

fn transform(tr: &Isometry2<f32>, mut vertex: Vertex) -> Vertex {
    let p = Point2::new(vertex.x, vertex.y);
    let p = tr * p;
    vertex.x = p[0];
    vertex.y = p[1];
    if vertex.shape == Shape::Arrow && vertex.extra.is_finite() {
        vertex.extra += tr.rotation.angle();
    }
    vertex
}

macro_rules! rgba {
    ($named:ident; $alpha:expr) => {
        Srgba {
            color: palette::named::$named.into_format(),
            alpha: $alpha,
        }
    };
}

fn send_config(socket: Arc<UdpSocket>, period: Duration) {
    let packet = Encoder::with(|encoder| {
        encoder.with_topic(CHASSIS_TOPIC, |mut chassis| {
            chassis.set_capacity(100);
            chassis.set_focus(100);

            chassis.set_color(0, rgba!(AZURE; 1.0));
            chassis.set_color(1, rgba!(GOLD; 1.0));
        });
        encoder.with_topic(ODOMETRY_TOPIC, |mut odometry| {
            odometry.set_capacity(20000);
            odometry.set_focus(500);
            odometry.set_color(0, rgba!(GREEN; 0.5));
        });
        encoder.with_topic(LIGHT_TOPIC, |mut light| {
            light.set_capacity(10);
            light.set_focus(10);
            light.set_color(0, rgba!(RED; 0.5));
        })
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
