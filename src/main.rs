use std::time::SystemTime;

use plotters::{prelude::*, style::full_palette::ORANGE};
use rand::prelude::*;
use rrtstar::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let now = SystemTime::now();
    let mut rng = rand::thread_rng();
    let goal = Goal::new(9.0, 9.0, 0.1);
    let mut end = None;
    let mut tree = Tree::new();
    for _ in 0..2 {
        let (x, y) = (rng.gen::<f32>() * 9.0 + 0.25, rng.gen::<f32>() * 9.0 + 0.25);
        let (w, h) = (rng.gen::<f32>() * 5.0, rng.gen::<f32>() * 5.0);
        tree.add_obs(Obstacle::new(x - w / 2.0, y - h / 2.0, w, h));
    }
    tree.add(0.0, 0.0);
    let mut iters = 0;
    loop {
        let new = if iters % 10 == 0 {
            (goal.x, goal.y)
        } else {
            (rng.gen::<f32>() * 10.0, rng.gen::<f32>() * 10.0)
        };
        let node = tree.add(new.0, new.1);
        if goal.sample(tree.nodes[node].x, tree.nodes[node].y) {
            end = Some(node);
            // break;
        }

        if iters > 5000 {
            break;
        }

        iters += 1;
        println!("iter {}", iters);
    }
    let path = if end.is_some() {
        tree.ancestry(end.unwrap_or(0))
    } else {
        vec![]
    };

    println!("solve time {:?} in {} iters", now.elapsed().unwrap(), iters);

    let root = BitMapBackend::new("image.png", (640, 480)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption("graph", ("sans-serif", 50).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(0f32..10f32, 0.0f32..10.0f32)?;

    chart.configure_mesh().draw()?;

    // chart.draw_series(
    //     ran_nodes.iter().map(|n| {
    //         Circle::new((n.0.0, n.0.1), 2.5, &GREEN)
    //     })
    // ).unwrap();

    // chart.draw_series(
    //     ran_nodes.iter().map(|n| {
    //         PathElement::new([n.0, n.1], &GREEN)
    //     })
    // ).unwrap();

    chart
        .draw_series(tree.nodes.iter().map(|n| {
            return match n.parent {
                Some(m) => {
                    PathElement::new([(n.x, n.y), (tree.nodes[m].x, tree.nodes[m].y)], &BLUE)
                }
                None => PathElement::new([], &BLUE),
            };
        }))
        .unwrap();

    chart
        .draw_series(tree.nodes.iter().map(|n| Circle::new((n.x, n.y), 2, &RED)))
        .unwrap();

    chart
        .draw_series(
            tree.obstacles
                .iter()
                .map(|o| Rectangle::new([(o.x, o.y), (o.x + o.w, o.y + o.h)], &ORANGE)),
        )
        .unwrap();

    let goals = vec![goal, Goal::new(0.0, 0.0, 0.0)];
    chart
        .draw_series(
            goals
                .iter()
                .map(|g| Circle::new((g.x, g.y), g.r * 50.0, &GREEN)),
        )
        .unwrap();

    chart
        .draw_series(path.iter().as_slice().windows(2).map(|ps| {
            let (a, b) = (ps[0], ps[1]);
            PathElement::new(
                [
                    (tree.nodes[a].x, tree.nodes[a].y),
                    (tree.nodes[b].x, tree.nodes[b].y),
                ],
                GREEN.stroke_width(2),
            )
        }))
        .unwrap();

    root.present()?;

    println!("total time {:?}", now.elapsed().unwrap());

    Ok(())
}
