//! Library providing A* pathfinding utilities.

#![feature(test)]
extern crate test;

use hibitset::BitSet;

/// A position in the grid.
/// Expressed as a tuple of the (x, y) coordinates.
pub type Position = (usize, usize);

/// The InverseDifficulty indicates how easy it is to travel a square.
/// An inverse difficulty of 1 indicates that the "length" of the square is 1.
/// An inverse difficulty of 0 indicates a square of infinite length (that cannot be traversed). A wall for
/// example would have a value of 0.
/// An inverse difficulty of 2 indicates that the length of the square is 0.5 (2x travel speed).
pub trait InverseDifficulty {
    fn get_weight(&self) -> f32;
}

impl InverseDifficulty for f32 {
    fn get_weight(&self) -> f32 {
        *self
    }
}

/// A Grid is a fixed size 2d grid composed of equally sized squares.
/// Each square is of type B (the generic parameter).
///
/// # Generics
/// B: The type of square of which the grid in composed. See the `InverseDifficulty` trait for more
/// information.
pub struct Grid<B: InverseDifficulty> {
    grid: Vec<B>,
    size: (usize, usize),
}

impl<B: InverseDifficulty + Default + Clone> Grid<B> {
    
    /// Create a new grid of the specified size.
    pub fn new(size_x: usize, size_y: usize) -> Self {
        Grid {
            grid: vec![B::default(); size_x * size_y],
            size: (size_x, size_y),
        }
    }

    /// Get the square at the specified position.
    pub fn get_grid(&self, x: usize, y: usize) -> &B {
        let idx = self.get_index_for_position(x, y);
        self.grid.get(idx).unwrap()
    }

    /// Get the square mutably at the specified position.
    pub fn get_grid_mut(&mut self, x: usize, y: usize) -> &mut B {
        let idx = self.get_index_for_position(x, y);
        self.grid.get_mut(idx).unwrap()
    }

    /// Inserts a square element at the specified position.
    pub fn insert_grid(&mut self, x: usize, y: usize, grid: B) {
        let idx = self.get_index_for_position(x, y);
        *self.grid.get_mut(idx).unwrap() = grid;
    }

    fn get_index_for_position(&self, x: usize, y: usize) -> usize {
        let idx = y * self.size.1 + x;
        assert!(idx < self.max_index());
        idx
    }

    fn get_position_for_index(&self, idx: usize) -> Position {
        let y = idx / self.size.1;
        let x = idx % self.size.1;
        (x, y)
    }

    fn max_index(&self) -> usize {
        self.size.0 * self.size.1 - 1
    }

    fn dist_between(&self, from: usize, to: usize) -> f32 {
        let from = self.get_position_for_index(from);
        let to = self.get_position_for_index(to);

        let to0 = to.0 as f32;
        let to1 = to.1 as f32;
        let from0 = from.0 as f32;
        let from1 = from.1 as f32;
        
        ((to0 - from0) * (to0 - from0) + (to1 - from1) * (to1 - from1)).sqrt()
    }

    fn heuristic(&self, _from: Position, _to: Position) -> f32 {
        // Slowest possible heuristic for testing.
        0.0
    }

    pub(crate) fn neighbors_of(&self, idx: usize) -> Vec<usize> {
        let mut res = vec![];
        let pos = self.get_position_for_index(idx);
        
        // Left
        if pos.0 > 0 {
            res.push(self.get_index_for_position(pos.0 - 1, pos.1));
        }

        // Right
        if pos.0 < self.size.0 - 1{
            res.push(self.get_index_for_position(pos.0 + 1, pos.1));
        }

        // Up
        if pos.1 > 0 {
            res.push(self.get_index_for_position(pos.0, pos.1 - 1));
        }

        // Down
        if pos.1 < self.size.1 - 1 {
            res.push(self.get_index_for_position(pos.0, pos.1 + 1));
        }

        res
    }

    /// Finds the shortest path between two positions if it exists.
    /// The returned position list (path) will be formed like so:
    /// [from, intermediate_1, intermediate_2, ..., intermediate_n, to]
    pub fn shortest_path(&self, from: Position, to: Position) -> Option<Vec<Position>> {
        let max_index = self.max_index();
        let node_count = max_index + 1;

        assert!(from.0 * from.1 < max_index);
        assert!(to.0 * to.1 < max_index);


        // Merge closed_set into came_from?
        let mut closed_set = BitSet::with_capacity(node_count as u32);
        let mut open_set = BitSet::with_capacity(node_count as u32);
        let mut came_from = vec![None; node_count as usize];
        let mut gscore = vec![std::f32::MAX; node_count as usize];
        let mut fscore = vec![std::f32::MAX; node_count as usize];

        let start_idx = self.get_index_for_position(from.0, from.1);
        let to_idx = self.get_index_for_position(to.0, to.1);
        open_set.add(start_idx as u32);
        *gscore.get_mut(start_idx).unwrap() = 0.0;
        *fscore.get_mut(start_idx).unwrap() = self.heuristic(from, to);

        // Find node in open set with lowest fscore.
        while let Some((idx, _fscore)) = (0..max_index).filter(|i| open_set.contains(*i as u32)).map(|i| (i, fscore.get(i).unwrap())).fold(None, |accum: Option<(usize, f32)>, cur| {
            if let Some(acc) = accum {
                if acc.1 > *cur.1 {
                    Some((cur.0, *cur.1))
                } else {
                    Some(acc)
                }
            } else {
                Some((cur.0, *cur.1))
            }
        }) {

            // Exit condition
            if to_idx == idx {
                return Some(self.reconstruct_path(came_from, idx));
            }

            open_set.remove(idx as u32);
            closed_set.add(idx as u32);

            for neighbor in self.neighbors_of(idx) {
                if closed_set.contains(neighbor as u32) {
                    continue;
                }

                let attempt_gscore = gscore.get(idx).unwrap() + self.dist_between(idx, neighbor);

                if !open_set.contains(neighbor as u32) {
                    open_set.add(neighbor as u32);
                } else if attempt_gscore >= *gscore.get(neighbor).unwrap() {
                    continue;
                }

                *came_from.get_mut(neighbor).unwrap() = Some(idx);
                *gscore.get_mut(neighbor).unwrap() = attempt_gscore;
                *fscore.get_mut(neighbor).unwrap() = attempt_gscore + self.heuristic(self.get_position_for_index(neighbor), to);
            }
        }
        None
    }

    /// Reconstruct the paths that was traversed to come to the current position.
    fn reconstruct_path(&self, came_from: Vec<Option<usize>>, current: usize) -> Vec<Position> {
        let mut total_path = vec![self.get_position_for_index(current)];
        let mut current = current;
        while let Some(Some(prev)) = came_from.get(current) {
            total_path.push(self.get_position_for_index(*prev));
            current = *prev;
        }
        total_path.reverse();
        total_path
    }
}



#[cfg(test)]
mod tests {
    use test::Bencher;
    use rand::prelude::*;
    
    use crate::*;

    #[test]
    fn astar() {
        let grid = Grid::<f32>::new(20, 20);
        let from = (2, 4);
        let to = (2, 7);
        let shortest = grid.shortest_path(from, to).unwrap();
        assert_eq!(shortest, vec![(2, 4), (2, 5), (2, 6), (2, 7)]);
    }

    #[test]
    fn neighbors() {
        let grid = Grid::<f32>::new(3, 3);
        let neighbors1 = grid.neighbors_of(4);
        assert_eq!(neighbors1, vec![3, 5, 1, 7]);
        let neighbors2 = grid.neighbors_of(0);
        assert_eq!(neighbors2, vec![1, 3]);
    }

    #[should_panic]
    #[test]
    fn out_of_bounds() {
        let grid = Grid::<f32>::new(20, 20);
        let from = (2, 4);
        let to = (2, 77);
        grid.shortest_path(from, to).unwrap();
    }


    #[bench]
    fn bench_shortest_path(b: &mut Bencher) {
        let grid = Grid::<f32>::new(50, 50);
        let from = (2, 4);
        let to = (20, 48);
        b.iter(|| {
            let _shortest = grid.shortest_path(from, to).unwrap();
        })
    }

    #[bench]
    fn bench_tile_access(b: &mut Bencher) {
        let grid = Grid::<f32>::new(200, 200);
        let mut rng = thread_rng();
        b.iter(|| {
            let idx = rng.gen_range(0, 199);
            let _id = grid.get_grid(idx, idx);
        })
    }


    #[bench]
    fn bench_distance_between(b: &mut Bencher) {
        let grid = Grid::<f32>::new(4,4);
        let from = 1;
        let to = 9;
        b.iter(|| {
            let dist = grid.dist_between(from, to);
            assert_eq!(dist, 2.0);
        })
    }
}
