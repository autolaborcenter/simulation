use async_std::{net::UdpSocket, path::PathBuf, sync::Arc, task};
use monitor_tool::{rgba, vertex, Encoder, Shape, Vertex};
use nalgebra::{Isometry2, Point2, Vector2};
use obstacle_avoidance::{convex_from_origin, fit, Obstacle, ObstacleError::*, Polar};
use path_tracking::{Parameters, PathFile, Sector, State, TrackContext, Tracker};
use pm1_control_model::{
    isometry, Odometry, Optimizer, Physical, Pm1Predictor, TrajectoryPredictor,
};
use std::{
    f32::consts::{FRAC_PI_3, FRAC_PI_4, FRAC_PI_8, PI},
    time::{Duration, Instant},
};

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
    ($x:expr, $y:expr) => {
        pm1_control_model::isometry($x as f32, $y as f32, 1.0, 0.0)
    };
    ($x:expr, $y:expr; $theta:expr) => {{
        let (sin, cos) = ($theta as f32).sin_cos();
        pm1_control_model::isometry($x as f32, $y as f32, cos, sin)
    }};
}

macro_rules! point {
    ($p:expr) => {
        crate::Point2 { coords: $p }
    };
    ($x:expr, $y:expr) => {
        crate::Point2 {
            coords: crate::vector($x as f32, $y as f32),
        }
    };
}

mod chassis;
mod outlines;
mod ray_cast;

use chassis::{sector_vertex, CHASSIS_TOPIC, ODOMETRY_TOPIC, ROBOT_OUTLINE, RUDDER};
use outlines::{Person, 三轮车, 墙, 崎岖轮廓, LIDAR_TOPIC, MOVING_TOPIC, OBSTACLES_TOPIC};
use ray_cast::ray_cast;

const FOCUS_TOPIC: &str = "focus";
const LIGHT_TOPIC: &str = "light";
const PATH_TOPIC: &str = "path";
const PRE_TOPIC: &str = "pre";

const FF: f32 = 3.0; // 倍速仿真
const PATH_TO_TRACK: &str = "1105-1"; // 路径名字

const PERIOD: Duration = Duration::from_millis(40);
const RGBD_ON_CHASSIS: Isometry2<f32> = pose!(0.1, 0);
const RGBD_METERS: f32 = 4.0;
const RGBD_DEGREES: f32 = 170.0;
const TRACK_SPEED: f32 = 0.6;
const LIGHT_RADIUS: f32 = 0.4;

fn main() {
    task::block_on(async {
        let socket = Arc::new(UdpSocket::bind("0.0.0.0:0").await.unwrap());
        let _ = socket.connect("127.0.0.1:12345").await;
        // 初始位姿
        let mut odometry = Odometry {
            s: 0.0,
            a: 0.0,
            pose: pose!(-7686.0, -1873.5; 1.7),
        };
        // 底盘运动仿真
        let mut predictor = TrajectoryPredictor::<Pm1Predictor> {
            period: PERIOD,
            model: Default::default(),
            predictor: Pm1Predictor::new(Optimizer::new(0.5, 1.2, PERIOD), PERIOD),
        };
        // 深度相机
        let rgbd_bounds = sector_vertex(RGBD_ON_CHASSIS, RGBD_METERS, RGBD_DEGREES);
        let rgbd = Sector {
            radius: RGBD_METERS,
            angle: RGBD_DEGREES.to_radians(),
        };
        let obstacles_on_world = vec![
            {
                let tr = isometry(-7686.0, -1871.5, 0.0, 0.5);
                三轮车.iter().map(|v| tr * v).collect::<Vec<_>>()
            },
            {
                let tr = isometry(-7684.0, -1880.0, 0.0, 0.5);
                崎岖轮廓.iter().map(|v| tr * v).collect::<Vec<_>>()
            },
            {
                let tr = isometry(-7688.0, -1880.0, 0.0, 0.5);
                崎岖轮廓.iter().map(|v| tr * v).collect::<Vec<_>>()
            },
            {
                let tr = pose!(-7682.0, -1891.0; 2.6);
                崎岖轮廓.iter().map(|v| tr * v).collect::<Vec<_>>()
            },
            {
                let tr = pose!(-7686.0, -1882.0; FRAC_PI_4);
                三轮车.iter().map(|v| tr * v).collect::<Vec<_>>()
            },
            {
                let tr = pose!(-7688.5, -1888.0; FRAC_PI_4);
                墙.iter().map(|v| tr * v).collect::<Vec<_>>()
            },
        ];
        let mut person = Person::new(
            (50 as f32 / FF).round() as u8,
            0.4,
            pose!(-7682.0, -1898.0; 0.0),
        );
        // 路径
        let path = path_tracking::Path::new(
            PathFile::open(PathBuf::from(format!("path/{}.path", PATH_TO_TRACK)).as_path())
                .await
                .unwrap(),
            Sector {
                radius: LIGHT_RADIUS,
                angle: FRAC_PI_3,
            },
            10,
        );
        // 发送固定物体
        send_config(
            socket.clone(),
            Duration::from_secs(3),
            &path,
            obstacles_on_world.clone(),
        );
        // 循线仿真
        let mut tracker = Tracker {
            path: &path,
            context: TrackContext::new(Parameters {
                search_range: rgbd,
                light_radius: LIGHT_RADIUS,
                auto_reinitialize: true,
                r#loop: false,
            }),
        };
        let mut time = Instant::now();
        let mut person_time = time;
        let mut trend = 1.0;
        loop {
            // 更新移动障碍物
            person.update();
            if time - person_time > Duration::from_secs_f32(5.0 / FF) {
                person.next = PI;
                person_time = time;
            }
            // 更新位姿
            predictor.next().map(|d| odometry += d);
            let width = if let State::Initializing = tracker.context.state {
                1.0
            } else {
                0.7
            };
            // 构造障碍物对象
            let person = person.to_points();
            let lidar = {
                let mut obstacles = obstacles_on_world.clone();
                obstacles.push(person.0.clone());
                obstacles.push(person.1.clone());
                ray_cast(odometry.pose, RGBD_ON_CHASSIS, obstacles, rgbd)
            };
            let mut besieged = vec![];
            let mut obstacles = vec![];
            for convex in convex_from_origin(lidar.clone(), 0.8) {
                match Obstacle::new(
                    RGBD_ON_CHASSIS,
                    &fit(convex, rgbd.radius + 0.5, 0.04),
                    width,
                ) {
                    Ok(o) => obstacles.push(o),
                    Err(Empty) => {}
                    Err(Besieged(o)) => {
                        besieged.push(o);
                        break;
                    }
                }
            }
            predictor.predictor.target = if besieged.is_empty() {
                match tracker.track(odometry.pose) {
                    Err(_) => Physical::RELEASED,
                    Ok((k, rudder)) => {
                        let next = Physical {
                            speed: TRACK_SPEED * k,
                            rudder,
                        };
                        // 循线
                        let to_local = odometry.pose.inverse();
                        let rgbd_checker = rgbd.get_checker();
                        let mut checker = tracker.clone();
                        let mut time = Duration::ZERO;
                        let mut odom = odometry.clone();
                        let mut predictor = predictor.clone();
                        let mut last = odom.pose.translation.vector;
                        loop {
                            // 更新时间，10s 未碰撞则不再采样
                            time += predictor.period;
                            if time > Duration::from_secs(10) {
                                trend = 1.0;
                                break next;
                            }
                            // 仿真，停止仍未碰撞则不再采样
                            predictor.predictor.target = match checker.track(odom.pose) {
                                Ok((k, rudder)) => Physical {
                                    speed: TRACK_SPEED * k,
                                    rudder,
                                },
                                Err(_) => {
                                    trend = 1.0;
                                    break next;
                                }
                            };
                            // 更新位姿
                            if let Some(d) = predictor.next() {
                                odom += d;
                            } else {
                                trend = 1.0;
                                break next;
                            }
                            // 跳过一些点以降低计算量
                            if (odom.pose.translation.vector - last).norm_squared() > 0.0025 {
                                last = odom.pose.translation.vector;
                            } else {
                                continue;
                            }
                            // 变换到机器人坐标系，离开视野仍未碰撞则不再采样
                            let point = to_local * point!(odom.pose.translation.vector);
                            if point.coords.norm_squared() > RGBD_METERS.powi(2) {
                                trend = 1.0;
                                break next;
                            }
                            // 是否遭遇障碍物
                            if let Some(o) = obstacles.iter().find(|o| o.contains(point)) {
                                // 如果正在上线，目标位置后移
                                let target = match checker.context.state {
                                    State::Initializing => point!(-LIGHT_RADIUS, 0.0),
                                    _ => point!(0.0, 0.0),
                                };
                                // 路径变换到机器人坐标系上作为目标点的位置
                                let slice = checker
                                    .path
                                    .slice(checker.context.index)
                                    .iter()
                                    .map(|p| to_local * p * target);
                                // 计算避开障碍物的方法
                                let (index, rudder) = o.to_avoid(
                                    slice,
                                    |p| rgbd_checker.contains(p.coords),
                                    &mut trend,
                                );
                                // 如果已经离开路线，推进度
                                match tracker.context.state {
                                    State::Tracking => {}
                                    _ => tracker.context.index.1 = checker.context.index.1 + index,
                                };
                                // 计算控制量
                                break Physical {
                                    speed: TRACK_SPEED,
                                    rudder,
                                };
                            }
                        }
                    }
                }
            } else {
                Physical::RELEASED
            };
            // 编码并发送
            let socket = socket.clone();
            let predictor = predictor.clone();
            let rgbd_bounds = rgbd_bounds.clone();
            let local = tracker.path.slice(tracker.context.index)[0];
            task::spawn(async move {
                let tr = odometry.pose;
                let packet = Encoder::with(|figure| {
                    figure.with_topic(FOCUS_TOPIC, |mut light| {
                        light.clear();
                        light.push(transform(&tr, vertex!(0; 2.0, 0.0; Circle, 3.0; 0)));
                    });

                    figure
                        .topic(ODOMETRY_TOPIC)
                        .push(vertex_from_pose!(0; odometry.pose; 0));
                    figure.with_topic(CHASSIS_TOPIC, |mut topic| {
                        topic.clear();
                        topic.extend(ROBOT_OUTLINE.iter().map(|v| transform(&tr, *v)));
                        topic.extend(rgbd_bounds.iter().map(|v| transform(&tr, *v)));

                        let tr = tr * pose!(-0.355, 0.0; predictor.predictor.current.rudder);
                        topic.extend(RUDDER.iter().map(|v| transform(&tr, *v)));
                    });
                    figure.with_topic(LIGHT_TOPIC, |mut topic| {
                        topic.clear();
                        topic.push(transform(
                            &tr,
                            vertex!(0; LIGHT_RADIUS, 0.0; Circle, LIGHT_RADIUS; 0),
                        ));
                        topic.push(vertex_from_pose!(0; local; 0));
                    });
                    figure.with_topic(MOVING_TOPIC, |mut topic| {
                        topic.clear();
                        let (l, r) = person;
                        topic.extend_polygon(l.into_iter().map(|p| vertex!(0; p[0], p[1]; 255)));
                        topic.extend_polygon(r.into_iter().map(|p| vertex!(0; p[0], p[1]; 255)));
                    });
                    figure.with_topic(LIDAR_TOPIC, |mut topic| {
                        topic.clear();
                        {
                            let tr = tr * RGBD_ON_CHASSIS;
                            topic.extend(
                                lidar
                                    .iter()
                                    .map(|p| tr * Point2::from(*p))
                                    .map(|p| vertex!(0; p[0], p[1]; 0)),
                            );
                        }
                        for b in besieged {
                            topic.extend_polygon(b.into_iter().map(|p| {
                                let p = tr * Point2::from(p);
                                vertex!(2; p[0], p[1]; 255)
                            }));
                        }
                        obstacles
                            .iter()
                            .map(|o| {
                                let mut vec: Vec<_> =
                                    o.front.iter().map(|p| Point2::from(*p)).collect();
                                vec.extend(o.back.iter().map(|p| Point2::from(*p)));
                                vec
                            })
                            .for_each(|v| {
                                topic.extend_polygon(v.into_iter().map(|p| {
                                    let p = tr * p;
                                    vertex!(1; p[0], p[1]; 255)
                                }));
                            });
                    });
                    // 轨迹预测
                    figure.with_topic(PRE_TOPIC, |mut topic| {
                        topic.clear();
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
                                topic.push(vertex_from_pose!(0; odometry.pose; 0));
                            }
                            if odometry.s > max_s || odometry.a > max_a || t > max_t {
                                break;
                            }
                        }
                    });
                });
                let _ = socket.send(&packet).await;
            });
            // 延时到下一周期
            // let mut ignored = Default::default();
            // let _ = async_std::io::stdin().read_line(&mut ignored).await;
            time += PERIOD.div_f32(FF);
            if let Some(dur) = time.checked_duration_since(Instant::now()) {
                task::sleep(dur).await;
            }
        }
    });
}

fn transform(tr: &Isometry2<f32>, mut vertex: Vertex) -> Vertex {
    let p = tr * point!(vertex.x, vertex.y);
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
    path: &path_tracking::Path,
    obstacles: impl IntoIterator<Item = Vec<Point2<f32>>>,
) {
    let packet = Encoder::with(|figure| {
        figure.config_topic(
            CHASSIS_TOPIC,
            u32::MAX,
            0,
            &[
                (0, rgba!(AZURE; 1.0)),
                (1, rgba!(GOLD; 1.0)),
                (2, rgba!(ORANGE; 1.0)),
                (3, rgba!(GREEN; 0.5)),
            ],
            |_| {},
        );
        figure.config_topic(ODOMETRY_TOPIC, 20000, 0, &[(0, rgba!(VIOLET; 0.1))], |_| {});
        figure.config_topic(FOCUS_TOPIC, 1, 1, &[(0, rgba!(BLACK; 0.0))], |_| {});
        figure.config_topic(LIGHT_TOPIC, u32::MAX, 0, &[(0, rgba!(RED; 1.0))], |_| {});
        figure.config_topic(
            LIDAR_TOPIC,
            u32::MAX,
            0,
            &[
                (0, rgba!(DARKGOLDENROD; 0.3)),
                (1, rgba!(GOLD; 1.0)),
                (2, rgba!(RED; 1.0)),
            ],
            |_| {},
        );
        figure.config_topic(PRE_TOPIC, u32::MAX, 0, &[(0, rgba!(SKYBLUE; 0.3))], |_| {});
        figure.config_topic(
            OBSTACLES_TOPIC,
            u32::MAX,
            0,
            &[(0, rgba!(AZURE; 0.6))],
            |mut topic| {
                topic.clear();
                for o in obstacles {
                    topic.extend_polygon(o.into_iter().map(|p| vertex!(0; p[0], p[1]; 64)));
                }
            },
        );
        figure.config_topic(
            MOVING_TOPIC,
            u32::MAX,
            0,
            &[(0, rgba!(GREENYELLOW; 0.6))],
            |_| {},
        );
        figure.config_topic(
            PATH_TOPIC,
            u32::MAX,
            0,
            &[(0, rgba!(GRAY; 0.4))],
            |mut topic| {
                topic.clear();
                for v in &path.0 {
                    topic.extend(v.iter().map(|v| vertex_from_pose!(0; v; 64)));
                }
            },
        );
    });
    task::spawn(async move {
        let clear = Encoder::with(|encoder| encoder.topic(ODOMETRY_TOPIC).clear());
        let _ = socket.send(clear.as_slice()).await;
        loop {
            let _ = socket.send(&packet).await;
            task::sleep(period).await;
        }
    });
}

#[inline]
const fn vector(x: f32, y: f32) -> Vector2<f32> {
    use nalgebra::{ArrayStorage, OVector};
    OVector::from_array_storage(ArrayStorage([[x, y]]))
}
