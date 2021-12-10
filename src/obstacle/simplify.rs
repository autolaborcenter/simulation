use super::{is_left, isometry, Point2, Polar};
use std::{collections::VecDeque, f32::consts::FRAC_PI_6};

/// 同时分割+凸包
pub(crate) fn convex_from_origin(
    src: impl IntoIterator<Item = Polar>,
    width: f32,
) -> Vec<Vec<Point2<f32>>> {
    let mut source = src.into_iter().map(|p| (p, p.to_point()));
    let mut result = vec![];
    if let Some((polar, point)) = source.next() {
        // 最后一点的极坐标，用于分集
        let mut last = polar;
        // 初始化折线
        result.push(vec![point]);
        for (polar, point) in source {
            // 最后一条折线
            let tail = result.last_mut().unwrap();
            // 判断与上一个点形成的狭缝能否通过
            // 可以通过，分割
            if (polar.rho - last.rho).abs() > width || {
                let rho = f32::max(polar.rho, last.rho);
                let theta = (polar.theta - last.theta).abs();
                2.0 * rho * (theta * 0.5).sin()
            } > width
            {
                result.push(vec![point]);
            }
            // 不可通过，计算凸包
            else {
                loop {
                    match tail.as_slice() {
                        &[.., t1, t0] => {
                            if is_left(t1, t0, point) {
                                tail.pop();
                            } else {
                                break;
                            }
                        }
                        _ => break,
                    }
                }
                tail.push(point);
            }
            last = polar;
        }
    }
    result
}

/// 线段拟合
pub(crate) fn fit(src: Vec<Point2<f32>>, radius: f32, max_len: f32) -> Vec<Point2<f32>> {
    match src.len() {
        0 => return src,
        1 => {
            return vec![
                src[0],
                Polar {
                    rho: radius,
                    theta: src[0][0].atan2(src[0][1]),
                }
                .to_point(),
            ];
        }
        _ => {}
    }

    let back = src[src.len() - 1];
    let back = back[1].atan2(back[0]);
    let front = src[0];
    let front = front[1].atan2(front[0]);
    let n = ((back - front) / FRAC_PI_6).round() as usize;

    let mut result = Vec::with_capacity(src.len() + 2 + n);

    let step = (back - front) / (n + 1) as f32;
    for i in 0..=n {
        result.push(
            Polar {
                rho: radius,
                theta: back - i as f32 * step,
            }
            .to_point(),
        );
    }
    result.push(
        Polar {
            rho: radius,
            theta: front,
        }
        .to_point(),
    );

    let mut buf = vec![];
    for p in src {
        if buf.is_empty() {
            buf.push(p);
        } else {
            let dir = (*result.last().unwrap() - p).normalize();
            let tr = isometry(p[0], p[1], dir[0], dir[1]).inverse();
            let mut min = 0.0;
            let mut max = 0.0;
            // 超界，折断线段
            let mut outside = false;
            // 验证线段上所有点仍在线段上
            for p in buf.iter().rev() {
                let p = tr * p;
                let y = p[1];
                let updated = if y < min {
                    min = y;
                    true
                } else if y > max {
                    max = y;
                    true
                } else {
                    false
                };
                if updated && max - min > max_len {
                    outside = true;
                    break;
                }
            }
            if outside {
                result.push(*buf.last().unwrap());
                buf.clear();
                buf.push(p);
            } else {
                buf.push(p);
            }
        }
    }
    if let Some(p) = buf.pop() {
        result.push(p);
    }
    result
}

// Melkman 快速凸包算法，用于任意简单且封闭的多边形
#[allow(dead_code)]
pub(crate) fn melkman(src: Vec<Point2<f32>>) -> Vec<Point2<f32>> {
    if src.len() <= 3 {
        src
    } else {
        let mut buf = VecDeque::from_iter(src.iter().take(3).cloned());
        for p in src.into_iter().skip(3) {
            // 复制最后一点形成闭环
            let last = *buf.back().unwrap();
            buf.push_front(last);
            // 新点在队头线段的左侧
            while !is_left(buf[1], buf[0], p) {
                buf.pop_front();
            }
            // 新点在队尾线段的右侧
            while is_left(buf[buf.len() - 2], buf[buf.len() - 1], p) {
                buf.pop_back();
            }
            // 添加新点
            buf.push_back(p);
        }
        buf.into_iter().collect()
    }
}
