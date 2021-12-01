use async_std::{net::UdpSocket, sync::Arc, task};
use monitor_tool::{rgba, vertex, Encoder, Shape, Vertex};
use parry2d::na::{Isometry2, Point2, Vector2};
use path_tracking::Tracker;
use pm1_control_model::{Odometry, Optimizer, Physical, Pm1Predictor, TrajectoryPredictor};
use std::{
    f32::consts::{FRAC_PI_3, FRAC_PI_8, PI},
    time::{Duration, Instant},
};

mod chassis;
mod obstacles;

use chassis::{rgbd_bounds, CHASSIS_TOPIC, ODOMETRY_TOPIC, ROBOT_OUTLINE, RUDDER, SIMPLE_OUTLINE};
use obstacles::TRICYCLE_OUTLINE;

use crate::obstacles::OBSTACLES_TOPIC;

macro_rules! vertex_from_pose {
    ($level:expr; $pose:expr; $alpha:expr) => {
        vertex!($level;
                $pose.translation.vector[0],
                $pose.translation.vector[1];
                Arrow, $pose.rotation.angle();
                $alpha)
    };
}

macro_rules! pose {
    ($x:expr, $y:expr; $theta:expr) => {
        Isometry2::new(Vector2::new($x, $y), $theta)
    };
}

const FOCUS_TOPIC: &str = "focus";
const LIGHT_TOPIC: &str = "light";
const PATH_TOPIC: &str = "path";
const PRE_TOPIC: &str = "pre";

const FF: f32 = 3.0; // 倍速仿真
const PATH_TO_TRACK: &str = "1105-1"; // 路径名字

fn main() {
    task::block_on(async {
        let socket = Arc::new(UdpSocket::bind("0.0.0.0:0").await.unwrap());
        let mut repos = Tracker::new("path").unwrap();

        const PERIOD: Duration = Duration::from_millis(40);
        // 初始位姿
        let mut odometry = Odometry {
            s: 0.0,
            a: 0.0,
            pose: pose!(-7686.5, -1871.5; 0.0),
        };
        // 底盘运动仿真
        let mut predictor = TrajectoryPredictor::<Pm1Predictor> {
            period: PERIOD,
            model: Default::default(),
            predictor: Pm1Predictor::new(Optimizer::new(0.5, 1.2, PERIOD), PERIOD),
        };
        predictor.predictor.target = Physical {
            speed: 0.5,
            rudder: FRAC_PI_8,
        };
        // 深度相机
        let rgbd = rgbd_bounds(4.0, 85.0);
        let tricicle = {
            let tr = pose!(-7685.5, -1880.0; FRAC_PI_3);
            TRICYCLE_OUTLINE
                .iter()
                .map(|v| transform(&tr, *v))
                .collect::<Vec<_>>()
        };
        // 发送固定物体
        let path = repos.read(PATH_TO_TRACK).unwrap();
        send_config(socket.clone(), Duration::from_secs(3), path, vec![tricicle]);
        // 循线仿真
        let _ = repos.track(PATH_TO_TRACK);
        let mut time = Instant::now();
        loop {
            // 更新
            odometry += predictor.next().unwrap();
            // 循线
            let ref mut target = predictor.predictor.target;
            if let Some((speed, rudder)) = repos.put_pose(&odometry.pose) {
                target.speed = speed * 0.4;
                target.rudder = rudder;
            } else {
                target.speed = 0.0;
            }
            // 编码并发送
            let socket = socket.clone();
            let predictor = predictor.clone();
            let rgbd = rgbd.clone();
            task::spawn(async move {
                let tr = odometry.pose;
                let packet = Encoder::with(|encoder| {
                    encoder
                        .topic(ODOMETRY_TOPIC)
                        .push(vertex_from_pose!(0; odometry.pose; 0));
                    encoder.with_topic(CHASSIS_TOPIC, |mut chassis| {
                        chassis.clear();
                        chassis.extend(ROBOT_OUTLINE.iter().map(|v| transform(&tr, *v)));
                        chassis.extend(SIMPLE_OUTLINE.iter().map(|v| transform(&tr, *v)));
                        chassis.extend(rgbd.iter().map(|v| transform(&tr, *v)));

                        let tr = tr * pose!(-0.355, 0.0; predictor.predictor.current.rudder);
                        chassis.extend(RUDDER.iter().map(|v| transform(&tr, *v)));
                    });
                    encoder.with_topic(FOCUS_TOPIC, |mut light| {
                        light.clear();
                        light.push(transform(&tr, vertex!(0; 2.0, 0.0; Circle, 3.0; 0)));
                    });
                    encoder.with_topic(LIGHT_TOPIC, |mut light| {
                        light.clear();
                        light.push(transform(&tr, vertex!(0; 0.4, 0.0; Circle, 0.4; 0)));
                    });
                    // 轨迹预测
                    encoder.with_topic(PRE_TOPIC, |mut pre| {
                        pre.clear();
                        let period = predictor.period;
                        let mut t = Duration::ZERO;
                        let mut s = odometry.s + 0.1;
                        let mut a = odometry.a + FRAC_PI_8;

                        let max_s = odometry.s + 5.0;
                        let max_a = odometry.s + PI * 2.0;
                        let max_t = Duration::from_secs(10);
                        for d in predictor {
                            t += period;
                            odometry += d;
                            if odometry.s > s || odometry.a > a {
                                s = odometry.s + 0.1;
                                a = odometry.a + FRAC_PI_8;
                                pre.push(vertex_from_pose!(0; odometry.pose; 0));
                            }
                            if odometry.s > max_s || odometry.a > max_a || t > max_t {
                                break;
                            }
                        }
                    });
                });
                let _ = socket.send_to(&packet, "127.0.0.1:12345").await;
            });
            // 延时到下一周期
            time += PERIOD.div_f32(FF);
            if let Some(dur) = time.checked_duration_since(Instant::now()) {
                task::sleep(dur).await;
            }
        }
    });
}

fn transform(tr: &Isometry2<f32>, mut vertex: Vertex) -> Vertex {
    let p = tr * Point2::new(vertex.x, vertex.y);
    vertex.x = p[0];
    vertex.y = p[1];
    if vertex.shape == Shape::Arrow && vertex.extra.is_finite() {
        vertex.extra += tr.rotation.angle();
    }
    vertex
}

fn send_config(
    socket: Arc<UdpSocket>,
    period: Duration,
    path: Vec<Isometry2<f32>>,
    obstacles: impl IntoIterator<Item = Vec<Vertex>>,
) {
    let packet = Encoder::with(|encoder| {
        encoder.config_topic(
            CHASSIS_TOPIC,
            100,
            0,
            &[
                (0, rgba!(AZURE; 1.0)),
                (1, rgba!(GOLD; 1.0)),
                (2, rgba!(ORANGE; 1.0)),
                (3, rgba!(GREEN; 0.5)),
            ],
            |_| {},
        );
        encoder.config_topic(ODOMETRY_TOPIC, 20000, 0, &[(0, rgba!(VIOLET; 0.1))], |_| {});
        encoder.config_topic(FOCUS_TOPIC, 1, 1, &[(0, rgba!(BLACK; 0.0))], |_| {});
        encoder.config_topic(LIGHT_TOPIC, 1, 0, &[(0, rgba!(RED; 1.0))], |_| {});
        encoder.config_topic(PRE_TOPIC, 100, 0, &[(0, rgba!(SKYBLUE; 0.3))], |_| {});
        encoder.config_topic(
            OBSTACLES_TOPIC,
            10000,
            0,
            &[(0, rgba!(AZURE; 0.6))],
            |mut encoder| {
                encoder.clear();
                for o in obstacles {
                    encoder.extend(o.into_iter());
                }
            },
        );
        encoder.config_topic(
            PATH_TOPIC,
            10000,
            0,
            &[(0, rgba!(GRAY; 0.4))],
            |mut encoder| {
                encoder.clear();
                encoder.extend(path.iter().map(|v| vertex_from_pose!(0; v; 64)));
            },
        );
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
