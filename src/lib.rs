//! Library providing A* pathfinding utilities.

use hibitset::BitSet;

/// A position in the grid.
/// Expressed as a tuple of the (x, y) coordinates.
pub type Position = (usize, usize);

/// The weight indicates how easy it is to travel a square.
/// A weight of 1 indicates that the "length" of the square is 1.
/// A weight of 0 indicates a square of infinite length (that cannot be traversed). A wall for
/// example would have a weight of 0.
/// A weight of 2 indicates that the length of the square is 0.5 (2x travel speed).
pub trait Weighted {
    fn get_weight(&self) -> f32;
}

impl Weighted for f32 {
    fn get_weight(&self) -> f32 {
        *self
    }
}

/// A map is a fixed size grid composed of squares.
/// Each square is of type B (the generic parameter).
///
/// # Generics
/// B: The type of square of which the map in composed. See the `Weighted` trait for more
/// information.
pub struct Map {
    map: Vec<f32>,
    size: (usize, usize),
}

impl Map {
    
    /// Create a new map of the specified size.
    pub fn new(size_x: usize, size_y: usize) -> Self {
        Map {
            map: vec![1.0; size_x * size_y],
            size: (size_x, size_y),
        }
    }

    /// Get the grid at the specified position.
    pub fn get_grid(&self, x: usize, y: usize) -> &f32 {
        let idx = self.get_index_for_position(x, y);
        self.map.get(idx).unwrap()
    }

    /// Get the grid mutably at the specified position.
    pub fn get_grid_mut(&mut self, x: usize, y: usize) -> &mut f32 {
        let idx = self.get_index_for_position(x, y);
        self.map.get_mut(idx).unwrap()
    }

    /// Inserts a grid element at the specified position.
    pub fn insert_grid(&mut self, x: usize, y: usize, grid: f32) {
        let idx = self.get_index_for_position(x, y);
        *self.map.get_mut(idx).unwrap() = grid;
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

    fn neighbors_of(&self, idx: usize) -> Vec<usize> {
        let mut res = vec![];
        let pos = self.get_position_for_index(idx);
        
        // Left
        if pos.0 > 0 {
            res.push(self.get_index_for_position(pos.0 - 1, pos.1));
        }

        // Right
        if pos.0 < self.size.0 {
            res.push(self.get_index_for_position(pos.0 + 1, pos.1));
        }

        // Up
        if pos.1 > 0 {
            res.push(self.get_index_for_position(pos.0, pos.1 - 1));
        }

        // Down
        if pos.1 < self.size.1 {
            res.push(self.get_index_for_position(pos.0, pos.1 + 1));
        }

        res
    }

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
    use crate::*;

    #[test]
    fn astar() {
        let map = Map::new(20, 20);
        let from = (2, 4);
        let to = (2, 7);
        let shortest = map.shortest_path(from, to).unwrap();
        assert_eq!(shortest, vec![(2, 4), (2, 5), (2, 6), (2, 7)]);
    }

    #[should_panic]
    #[test]
    fn out_of_bounds() {
        let map = Map::new(20, 20);
        let from = (2, 4);
        let to = (2, 77);
        map.shortest_path(from, to).unwrap();
    }
}
