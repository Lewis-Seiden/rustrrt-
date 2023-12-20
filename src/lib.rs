

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Node {
    pub parent: Option<usize>,
    pub cost: f32,
    pub x: f32,
    pub y: f32,
}

impl Node {
    pub fn new(parent: Option<usize>, x: f32, y: f32, total_dist: f32) -> Self {
        Self { parent, cost: total_dist, x, y }
    }

    pub fn depth(&self, tree: &Tree) -> f32 {
        match self.parent {
            Some(p) => tree.nodes.get(p).unwrap().depth(tree) + self.cost,
            None => 0.0,
        }
    }

    fn dist(&self, x: f32, y: f32) -> f32 {
        dist(self.x, self.y, x, y)
    }
}

fn dist(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)).sqrt()
}

#[derive(Debug)]
pub struct Tree {
    pub nodes: Vec<Node>,
    pub obstacles: Vec<Obstacle>,
}

impl Tree {
    pub fn new() -> Self {
        Self { nodes: vec![], obstacles: vec![] }
    }

    pub fn min_dist(&self, x: f32, y: f32) -> Option<usize> {
        let mut min: Option<&Node> = None;
        let mut min_dist = f32::INFINITY;
        for node in &self.nodes {
            if node.dist(x, y) < min_dist {
                min = Some(node);
                min_dist = node.dist(x, y);
            }
            // println!("node {:?} min {:?} dist {:?}", node, min, min_dist);
        }
        match min {
            Some(n) => {
                // println!("min {:?} d {:?}", min, n.dist(x, y));
                self.nodes.iter().position(|e| e == n)
            },
            None => None
        }
    }

    pub fn min_cost(&self, node: usize) -> Option<usize> {
        let mut min: Option<&Node> = None;
        let mut min_cost = f32::INFINITY;
        for other in &self.nodes {
            if other.cost < min_cost {
                min = Some(other);
                min_cost = other.cost;
            }
            // println!("node {:?} min {:?} dist {:?}", node, min, min_dist);
        }
        match min {
            Some(n) => {
                // println!("min {:?} d {:?}", min, n.dist(x, y));
                self.nodes.iter().position(|e| e == n)
            },
            None => None
        }
    }

    fn intersect(s: (f32, f32), e: (f32, f32), obs: &Vec<Obstacle>) -> (f32, f32) {
        const RES: usize = 5; // steps per node
        let mut n = s;
        // println!("1 = {}", (dist(s.0, s.1, e.0, e.1) / RES as f32));
        // println!("s {:?} e {:?}", s, e);
        for t in (0..=RES).map(|x| x as f32 * (1.0 / RES as f32)) {
            let s_i = Self::interpolate(s, e, t);
            // println!("t {} x {} y {}", t, s_i.0, s_i.1);
            for o in obs {
                // println!("x {}, y {} obs {}", s_i.0, s_i.1, o.sample(s_i.0, s_i.1));
                if o.sample(s_i.0, s_i.1) {
                    return n;
                }
            }
            n = s_i;
        }
        n
    }

    fn interpolate(a: (f32, f32), b: (f32, f32), t: f32) -> (f32, f32) {
        let d = (b.0 - a.0, b.1 - a.1);
        (a.0 + (d.0 * t), a.1 + (d.1 * t))
    }

    fn constrain(n: (f32, f32), p: (f32, f32)) -> (f32, f32) {
        const R: f32 = 0.1;
        let n_t = (n.0 - p.0, n.1 - p.1);
        let mag = (n_t.0 * n_t.0 + n_t.1 * n_t.1).sqrt();
        // println!("n {:?} nt {:?} p {:?} mag {}", n, n_t, p, mag);
        if mag <= R {
            n
        } else {
            (n_t.0 * (R / mag) + p.0, n_t.1 * (R / mag) + p.1)
        }
    }

    pub fn add(&mut self, x: f32, y: f32) -> usize {
        let min = self.min_dist(x, y);
        // println!("min {:?}", min);
        let (par_x, par_y) = match min {
            Some(e)  => (self.nodes.get(e).unwrap().x, self.nodes.get(e).unwrap().y),
            None => (x, y)
        };
        let par_cost = match min {
            Some(e) => self.nodes.get(e).unwrap().cost,
            None => 0.0
        };
        let (c_x, c_y) = Self::constrain((x, y), (par_x, par_y));
        let (i_x, i_y) = Self::intersect((par_x, par_y), (c_x, c_y), &self.obstacles);
        // println!("og {:?} circle {:?} obs {:?}", (x, y), (c_x, c_y), (i_x, i_y));
        let new_par = self.min_dist(i_x, i_y);
        let (new_par_x, new_par_y) = match new_par {
            Some(e)  => (self.nodes.get(e).unwrap().x, self.nodes.get(e).unwrap().y),
            None => (x, y)
        };
        let new_par_cost = match new_par {
            Some(e) => self.nodes.get(e).unwrap().cost,
            None => 0.0
        };
        self.nodes.push(Node::new(new_par, i_x, i_y, dist(i_x, i_y, new_par_x, new_par_y) + new_par_cost));
        if min != self.min_dist(i_x, i_y) { println!("optimized")};
        self.nodes.len() - 1
    }

    pub fn add_obs(&mut self, obs: Obstacle) {
        self.obstacles.push(obs);
    }

    pub fn ancestry(&self, node: usize) -> Vec<usize> {
        match self.nodes[node].parent {
            Some(p) => {
                let mut res = vec![node];
                res.append(&mut self.ancestry(p));
                res
            },
            None => vec![node]
        }
    }
}

/// Bottom left, up and right positive
#[derive(Debug)]
pub struct Obstacle {
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
}

impl Obstacle {
    pub fn new(x: f32, y: f32, w: f32, h: f32) -> Self {
        Self { x, y, w, h }
    }

    pub fn sample(&self, x: f32, y: f32) -> bool {
        x >= self.x && x <= self.x + self.w && y >= self.y && y <= self.y + self.h
    }
}

pub struct Goal {
    pub x: f32,
    pub y: f32,
    pub r: f32
}

impl Goal {
    pub fn new(x: f32, y: f32, r: f32) -> Self {
        Self { x, y, r }
    }

    pub fn sample(&self, x: f32, y: f32) -> bool {
        dist(x, y, self.x, self.y) <= self.r
    }
}