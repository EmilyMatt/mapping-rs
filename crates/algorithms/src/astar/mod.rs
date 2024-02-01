use std::cmp::Ordering;
use nalgebra::{DMatrix, Point2};
use crate::astar::CellState::Free;
#[cfg(feature = "std")]
use std::collections::{BinaryHeap,HashSet};
#[cfg(not(feature = "std"))]
use alloc::collections::{BinaryHeap, BTreeSet as HashSet};
use alloc::boxed::Box;
#[derive(Clone)]
struct Node{
    position:Point2<i32>,
    state:CellState,
    cost_from_start:f32,
    estimated_cost:f32,
    total_cost:f32,
    parent:Option<Node>
}
//implement some func to serve priority Queue
impl PartialEq for Node{
    fn eq(&self, other: &Self) -> bool {
        self.position==other.position
    }
}
impl Eq for Node{}
impl PartialOrd for Node{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.total_cost.partial_cmp(&self.total_cost)
    }
}
impl Ord for Node{
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}


impl Node {
    fn distance_heuristic(&mut self,node:&Node){
        self.estimated_cost = self.position.distance(node.position) as f32;
    }
    fn update_total_cost(&mut self, node:&Node){
        self.distance_heuristic(node);
        self.update_cost_from_start();

        self.total_cost = self.cost_from_start+self.estimated_cost;
    }
    fn update_cost_from_start(&mut self){
        if self.parent!=None{
            self.cost_from_start=self.parent.unwrap().cost_from_start+distance_heuristic(&self.parent.unwrap().position, &self.position);
        }
        else { self.cost_from_start =0.0; }
    }
}
#[derive(Copy, Clone)]
enum CellState{
    Free,
    Occupied,
    Unknown
    
}
fn create_grid(nrow:usize,ncol:usize,occupied_cell:Vec<i32,i32>)->DMatrix<Node>{
    DMatrix::from_fn(nrow,ncol,|row:i32,col:i32| Node {
        position: Default::default(),
        state: CellState::Free,
        cost_from_start: f32::INFINITY,
        estimated_cost: 0.0,
        total_cost: f32::INFINITY,
        parent: None,
    })
}
//Euclidean distance
fn distance_heuristic(p1:&Point2<i32>,p2:&Point2<i32>)->f32{
    p1.distance(p2) as f32
}

fn a_star(mut start:Node, end:Node, grid: &DMatrix<Node>) -> Option<Vec<Node>> {
    start.update_total_cost(&end);
    let mut to_check = BinaryHeap::new();
    to_check.insert(start);
    let mut was_check = HashSet::new();
    while let Some (current) = to_check.pop(){
        if current.position.eq(&end.position) {
            let mut path:Vec<Node>= vec![current];
            let mut temp = current;
            while let Some(parent) = &temp.parent{
                path.push(parent.clone());
                temp = **parent;
            }
            path.reverse();
            return Some(path);
        }
        was_check.insert(current.position);
        //get neighbor
        for &(dx, dy) in &[(-1, 0), (1, 0), (0, -1), (0, 1)] {
            let x = current.position.x+dx;
            let y = current.position.y+dy;
            if let Some(mut next) = grid.get((x,y)){
                if next.state==Free&&!was_check.contains(next.position) {
                    let tentative_cost = current.cost_from_start+distance_heuristic(&current.position,&next.position);
                    if tentative_cost<next.cost_from_start {
                        next.parent = Some(Box::new(current.clone()));
                        next.update_total_cost(&end);
                        to_check.push(next.clone());

                    }
                }
            }

        }
    }
    None
}
