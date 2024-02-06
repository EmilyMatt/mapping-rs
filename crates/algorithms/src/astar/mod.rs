use crate::BinaryHeap;
use crate::Box;
use crate::Ordering;
use crate::{vec, Vec};
use nalgebra::{DMatrix, Point2};

#[derive(Clone, Debug)]
struct Node {
    position: Point2<i32>,
    state: CellState,
    cost_from_start: f32,
    estimated_cost: f32,
    total_cost: f32,
    parent: Option<Box<Node>>,
}
//implement some func to serve priority Queue
impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
    }
}
impl Eq for Node {}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.total_cost.partial_cmp(&self.total_cost)
    }
}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl Node {
    fn distance_heuristic(&mut self, node: &Node) {
        self.estimated_cost = distance_heuristic(&self.position, &node.position);
    }
    fn update_total_cost(&mut self, node: &Node) {
        self.distance_heuristic(node);
        self.update_cost_from_start();

        self.total_cost = self.cost_from_start + self.estimated_cost;
    }
    fn update_cost_from_start(&mut self) {
        if self.parent != None {
            self.cost_from_start = self.parent.clone().unwrap().cost_from_start
                + distance_heuristic(&self.parent.clone().unwrap().position, &self.position);
        } else {
            self.cost_from_start = 0.0;
        }
    }
}
#[derive(Copy, Clone, Debug, PartialEq)]
enum CellState {
    Free,
    Occupied,
    Unknown,
}
fn create_grid(nrow: usize, ncol: usize, occupied_cells: Vec<(usize, usize)>) -> DMatrix<Node> {
    let mut grid = DMatrix::from_fn(nrow, ncol, |row: usize, col: usize| Node {
        position: Default::default(),
        state: CellState::Free,
        cost_from_start: f32::INFINITY,
        estimated_cost: 0.0,
        total_cost: f32::INFINITY,
        parent: None,
    });
    for (x, y) in occupied_cells {
        if let Some(node) = grid.get_mut((x as usize, y as usize)) {
            node.state = CellState::Occupied;
        }
    }
    grid
}
//Euclidean distance
fn distance_heuristic(p1: &Point2<i32>, p2: &Point2<i32>) -> f32 {
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    ((dx * dx + dy * dy) as f32).sqrt()
}

fn a_star(mut start: Node, end: Node, grid: &mut DMatrix<Node>) -> Option<Vec<Node>> {
    start.update_total_cost(&end);
    println!("{}", start.total_cost);
    let mut to_check = BinaryHeap::new();
    to_check.push(start);
    println!("{}", to_check.len());

    let mut was_check = Vec::new();
    while let Some(current) = to_check.pop() {
        println!("{}", current.position);
        if current.position.eq(&end.position) {
            let mut path: Vec<Node> = vec![current.clone()];
            let mut temp = current;
            println!("before loop path len:{}", path.len());
            while let Some(parent) = &temp.parent {
                path.push(*parent.clone());
                temp = *parent.clone();
            }
            println!("after loop path len: {}", path.len());
            path.reverse();
            return Some(path);
        }
        was_check.push(current.position);
        //get neighbor
        for &(dx, dy) in &[(-1, 0), (1, 0), (0, -1), (0, 1)] {
            println!("next check");
            let x = current.position.x + dx;
            println!("x = {}", x);
            let y = current.position.y + dy;
            println!("y = {}", y);
            if (x >= 0 && x < grid.ncols() as i32) && (y >= 0 && y < grid.nrows() as i32) {
                let next = grid.index_mut((x as usize, y as usize));
                if Some(next).is_some(){


                    println!("_next position: {}", next.position);
                    if next.state == CellState::Free {
                        let tentative_cost = current.cost_from_start
                            + distance_heuristic(&current.position, &next.position);

                        // Obtain a mutable reference to update the node
                            if tentative_cost < next.cost_from_start {
                                next.parent = Some(Box::new(current.clone()));
                                next.update_total_cost(&end);
                                to_check.push(next.clone()); // Clone the modified Node
                                println!("tentative_cost: {}", tentative_cost);
                                println!("to check len {}", to_check.len());
                                println!("total_cost : {}", next.total_cost);

                        }
                    }
                }
            }
        }
    }
    None
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_node_creation() {
        let position = Point2::new(1, 2);
        let node = Node {
            position,
            state: CellState::Free,
            cost_from_start: 0.0,
            estimated_cost: 0.0,
            total_cost: 0.0,
            parent: None,
        };

        assert_eq!(node.position, Point2::new(1, 2));
        assert_eq!(node.state, CellState::Free);
    }
    #[test]
    fn test_node_ordering() {
        let mut node1 = Node {
            position: Point2::new(0, 0),
            state: CellState::Free,
            cost_from_start: 0.0,
            estimated_cost: 0.0,
            total_cost: 10.0,
            parent: None,
        };
        let mut node2 = Node {
            position: Point2::new(1, 1),
            state: CellState::Free,
            cost_from_start: 0.0,
            estimated_cost: 0.0,
            total_cost: 5.0,
            parent: None,
        };
        //beacause we used binary heap smaller value should be considered greater to get the node with the smallest cost.
        assert!(node1 < node2);
    }
    #[test]
    fn test_distance_heuristic() {
        let p1 = Point2::new(0, 0);
        let p2 = Point2::new(3, 4);
        let distance = distance_heuristic(&p1, &p2);

        assert_eq!(distance, 5.0);
    }
    #[test]
    fn test_a_star_with_path_verification() {
        let start = Node {
            position: Point2::new(0, 0),
            state: CellState::Free,
            cost_from_start: 0.0,
            estimated_cost: 0.0,
            total_cost: 0.0,
            parent: None,
        };
        let end = Node {
            position: Point2::new(2, 2),
            state: CellState::Free,
            cost_from_start: 0.0,
            estimated_cost: 0.0,
            total_cost: 0.0,
            parent: None,
        };

        let mut grid = create_grid(4, 4, vec![(1, 1), (1, 2)]);

        let path_option = a_star(start, end, &mut grid);

        assert!(path_option.is_some());
        let path = path_option.unwrap();

        let expected_path_positions = vec![
            Point2::new(0, 0),
            Point2::new(0, 1),
            Point2::new(0, 2),
            Point2::new(1, 3),
            Point2::new(2, 3),
            Point2::new(2, 2),
        ];

        assert_eq!(path.len(), expected_path_positions.len());
        for (node, &expected_position) in path.iter().zip(expected_path_positions.iter()) {
            assert_eq!(node.position, expected_position);
        }
    }
    #[test]
    fn test_create_grid() {
        let nrow = 5;
        let ncol = 5;
        let occupied_cells = vec![(1, 1), (2, 2), (3, 3)];

        let grid = create_grid(nrow, ncol, occupied_cells.clone());

        // Check grid dimensions
        assert_eq!(grid.nrows(), nrow);
        assert_eq!(grid.ncols(), ncol);

        // Check if occupied cells are correctly marked
        for &(x, y) in &occupied_cells {
            if let Some(node) = grid.get((x, y)) {
                assert_eq!(node.state, CellState::Occupied);
            } else {
                panic!("Occupied cell not found in the grid");
            }
        }

        // Check if other cells are free and have correct initial values
        for x in 0..nrow {
            for y in 0..ncol {
                if !occupied_cells.contains(&(x, y)) {
                    if let Some(node) = grid.get((x, y)) {
                        assert_eq!(node.state, CellState::Free);
                        assert_eq!(node.cost_from_start, f32::INFINITY);
                        assert_eq!(node.estimated_cost, 0.0);
                        assert_eq!(node.total_cost, f32::INFINITY);
                        assert!(node.parent.is_none());
                    } else {
                        panic!("Free cell not found in the grid");
                    }
                }
            }
        }
    }
}
