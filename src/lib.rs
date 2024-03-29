use nalgebra::{Isometry2, Point2, Unit, Vector2};
use ncollide2d::shape::{Ball, Compound, Cuboid, Plane, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::force_generator::ForceGenerator;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::material::{BasicMaterial, MaterialHandle};
use nphysics2d::math::{Force, ForceType, Velocity};
use nphysics2d::object::{
    Body, BodyHandle, BodyPartHandle, BodySet, ColliderDesc, DefaultBodySet, DefaultColliderSet,
    Ground, RigidBody, RigidBodyDesc,
};
use nphysics2d::solver::IntegrationParameters;
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use rand;
use rand::prelude::*;
use wasm_bindgen::prelude::*;

// When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
// allocator.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

const RESTITUTION: f32 = 0.99;

macro_rules! log {
    ( $( $x:expr ),* ) => {
        log(&format!($($x,)*));
    };
}

#[wasm_bindgen]
extern "C" {
    fn alert(s: &str);
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

#[derive(Clone, Copy)]
enum ObjType {
    Circle = 0,
    Wall,
}

#[wasm_bindgen]
pub struct World {
    mech: DefaultMechanicalWorld<f32>,
    geo: DefaultGeometricalWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    joint_constraints: DefaultJointConstraintSet<f32>,
    force_generators: DefaultForceGeneratorSet<f32>,
    positions: Vec<(f32, f32, f32, f32, f32)>,
    width: f32,
    height: f32,
    pub num_particles: usize,
    rng: rand::rngs::ThreadRng,
}

#[wasm_bindgen]
impl World {
    pub fn new(
        width: f32,
        height: f32,
        gravity: f32,
        num_particles: usize,
        radius: f32,
        mass: f32,
        heat: f32,
        _van_der_waals: f32,
    ) -> Self {
        console_error_panic_hook::set_once();
        let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(0.0, gravity));
        let geometrical_world = DefaultGeometricalWorld::new();

        let mut bodies = DefaultBodySet::new();
        let mut colliders = DefaultColliderSet::new();
        let joint_constraints = DefaultJointConstraintSet::new();
        let mut force_generators = DefaultForceGeneratorSet::new();

        // Create box boundaries
        let walls = [
            // x, y, normal_vector
            (0.0f32, 0.0f32, Vector2::new(0., 1.)),  // bottom
            (0., 0., Vector2::new(1., 0.)),          // left
            (width, 0., Vector2::new(-1., 0.)),      // right
            (0.0f32, height, Vector2::new(0., -1.)), // top
        ];
        let ground = ShapeHandle::new(Compound::new(
            walls
                .iter()
                .copied()
                .map(|(x, y, norm)| {
                    let delta = Isometry2::new(Vector2::new(x, y), 0.);
                    let shape = ShapeHandle::new(Plane::new(Unit::new_normalize(norm)));
                    (delta, shape)
                })
                .collect(),
        ));
        let ground_handle = bodies.insert(Ground::new());
        colliders.insert(
            ColliderDesc::new(ground)
                //.translation(Vector2::y() * height)
                // restitution, friction: 0.5
                .material(MaterialHandle::new(BasicMaterial::new(RESTITUTION, 0.0)))
                .build(BodyPartHandle(ground_handle, 0)),
        );

        // Add Van Der Waals forces between the particles
        //if van_der_waals < 0. {
        //    log!("Adding Van Der Waals force {}", van_der_waals);
        //    force_generators.insert(Box::new(VanDerWaalsForce {
        //        magnitude: van_der_waals,
        //        distance_threshold: 128,
        //        world_width: width as usize,
        //        world_height: height as usize,
        //        num_particles,
        //    }));
        //}

        // Seed it with particles
        let rng = rand::thread_rng();
        let mut world = World {
            mech: mechanical_world,
            geo: geometrical_world,
            bodies,
            colliders,
            joint_constraints,
            force_generators,
            rng,
            num_particles: 0,
            positions: vec![],
            width: width,
            height: height,
        };
        let mut rng = rand::thread_rng();
        (0..num_particles).for_each(|_i| {
            world.add_particles(
                width * rng.gen::<f32>(),
                height * rng.gen::<f32>(),
                radius,
                mass,
                heat,
                1,
            );
        });
        world
    }

    pub fn add_particles(&mut self, x: f32, y: f32, radius: f32, mass: f32, heat: f32, n: usize) {
        //log!("Adding {} particle(s) at {}, {}", n, x, y);
        (0..n).for_each(|_| {
            let rigid_body = RigidBodyDesc::new()
                .position(Isometry2::new(Vector2::new(x, y), 0.))
                .gravity_enabled(true)
                .velocity(Velocity::linear(
                    heat * self.rng.gen::<f32>() - heat / 2.,
                    heat * self.rng.gen::<f32>() - heat / 2.,
                ))
                .mass(mass)
                .sleep_threshold(None)
                .user_data((ObjType::Circle, radius, radius))
                .build();
            let handle = self.bodies.insert(rigid_body);

            self.colliders.insert(
                ColliderDesc::new(ShapeHandle::new(Ball::new(radius)))
                    // restitution, friction: 0.5
                    .material(MaterialHandle::new(BasicMaterial::new(RESTITUTION, 0.0)))
                    .margin(0.5)
                    .build(BodyPartHandle(handle, 0)),
            );
        });
    }

    pub fn remove_particles(&mut self, x: f32, y: f32, radius: f32) {
        log!("Removing particle from {}, {}", x, y);
        let r2 = radius.powi(2);
        let bodies: Vec<_> = self
            .colliders
            .iter()
            .filter_map(|(col_h, collider)| {
                if collider.shape_handle().is_shape::<Ball<_>>() {
                    Some((
                        col_h,
                        collider.body(),
                        self.bodies.get(collider.body()).unwrap(),
                    ))
                } else {
                    None
                }
            })
            .filter(|(_, _, body)| {
                let pos = body.part(0).unwrap().position().translation.vector;
                let (dx, dy) = ((pos.x - x).abs(), (pos.y - y).abs());
                dx < radius && dy < radius && dx.powi(2) + dy.powi(2) < r2
            })
            .map(|(col_h, bod_h, _)| (col_h, bod_h))
            .collect();

        bodies.iter().for_each(|(col_h, bod_h)| {
            self.bodies.remove(*bod_h);
            self.colliders.remove(*col_h);
        });
    }

    pub fn add_wall(&mut self, mass: f32, horizontal: bool) {
        log!("Adding {} wall", if horizontal { "horiz" } else { "vert" });
        if self
            .colliders
            .iter()
            .find(|(_, collider)| collider.shape_handle().is_shape::<Cuboid<f32>>())
            .is_some()
        {
            log!("Wall already exists");
            return;
        }
        log!("Wall doesn't exist");
        let width = self.width;
        let height = 0.1 * self.height;
        let rigid_body = RigidBodyDesc::new()
            .position(Isometry2::new(
                Vector2::new(self.width / 2., self.height - 1.),
                0.,
            ))
            .gravity_enabled(true)
            .mass(mass)
            .sleep_threshold(None)
            .user_data((ObjType::Wall, width, height))
            .build();
        log!("Inserting wall");
        let handle = self.bodies.insert(rigid_body);

        log!("Inserting collider");
        self.colliders.insert(
            ColliderDesc::new(ShapeHandle::new(Cuboid::new(Vector2::new(width, height))))
                // restitution, friction: 0.5
                .material(MaterialHandle::new(BasicMaterial::new(RESTITUTION, 0.0)))
                .margin(height * .1)
                .build(BodyPartHandle(handle, 0)),
        );

        log!("Wall added");
    }

    pub fn remove_wall(&mut self) {
        log!("Removing wall...");
        let wall: Vec<_> = self
            .colliders
            .iter()
            .filter_map(|(collider_handle, collider)| {
                if collider.shape_handle().is_shape::<Cuboid<f32>>() {
                    Some((collider_handle, collider.body()))
                } else {
                    None
                }
            })
            .collect();
        wall.iter().for_each(|(wall_handle, wall_body)| {
            self.bodies.remove(*wall_body);
            self.colliders.remove(*wall_handle);
            log!("Removed");
        });
    }

    pub fn update_positions(&mut self) -> *const (f32, f32, f32, f32, f32) {
        self.positions = self
            .bodies
            .iter()
            .filter_map(|(_, body)| body.downcast_ref::<RigidBody<_>>())
            .filter(|body| body.user_data().is_some())
            .map(|body| {
                let pos = body.part(0).unwrap().position().translation.vector;
                let (obj_type, dim_x, dim_y) = *body
                    .user_data()
                    .unwrap()
                    .downcast_ref::<(ObjType, f32, f32)>()
                    .unwrap();
                (obj_type as i32 as f32, pos.x, pos.y, dim_x, dim_y)
            })
            .collect();
        // Expose this so that the caller knows the length of the returned array
        self.num_particles = self.positions.len();
        self.positions.as_ptr()
    }

    pub fn step(&mut self) -> *const (f32, f32, f32, f32, f32) {
        // Run the simulation.
        self.mech.step(
            &mut self.geo,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joint_constraints,
            &mut self.force_generators,
        );
        self.update_positions()
    }

    pub fn temperature(&mut self, x: f32, y: f32, radius: f32, factor: f32) {
        let r2 = radius.powi(2);
        self.bodies
            .iter_mut()
            .filter(|(_, body)| {
                body.downcast_ref::<RigidBody<_>>()
                    .and_then(|rigid_body| rigid_body.user_data())
                    .is_some()
            })
            .filter(|(_, body)| {
                let pos = body.part(0).unwrap().position().translation.vector;
                let (dx, dy) = ((pos.x - x).abs(), (pos.y - y).abs());
                dx < radius && dy < radius && dx.powi(2) + dy.powi(2) < r2
            })
            .for_each(|(_, body)| {
                body.generalized_velocity_mut()
                    .row_iter_mut()
                    .for_each(|mut v| {
                        v.x = v.x * factor;
                    });
            });
    }

    pub fn timestep(&self) -> f32 {
        self.mech.timestep()
    }
    pub fn set_timestep(&mut self, timestep: f32) {
        self.mech.set_timestep(timestep);
    }
    pub fn gravity(&self) -> Vec<f32> {
        vec![self.mech.gravity[0], self.mech.gravity[1]]
    }
    pub fn set_gravity(&mut self, gravity: f32) {
        self.mech.gravity = Vector2::y() * gravity;
    }
}

struct Buckets2d<H: Copy> {
    width: usize,
    height: usize,
    elements: Vec<(H, Point2<f32>, f32)>,
    bucket_size: usize,
    bucket_heads: Vec<Option<usize>>, // the first element in the linked list
    next_elements: Vec<Option<usize>>, // the index is the left element, the value is the right element>
}
impl<H: Copy> Buckets2d<H> {
    pub fn new(
        elements: &Vec<(H, Point2<f32>, f32)>,
        bucket_size: usize,
        world_width: usize,
        world_height: usize,
    ) -> Self {
        log!("A whole new world");
        let (width, height) = (
            1 + world_width / bucket_size,
            1 + world_height / bucket_size,
        );
        let mut bucket_heads = vec![None; width * height];
        let mut next_elements = vec![None; elements.len()];
        for (i, (_, pos, _)) in elements.iter().enumerate() {
            let bucket_pos = (pos[0] as usize / bucket_size, pos[1] as usize / bucket_size);
            let bucket_idx = Self::to_idx(width, height, bucket_pos);
            let prev_first = bucket_heads[bucket_idx];
            bucket_heads[bucket_idx] = Some(i);
            if let Some(_) = prev_first {
                next_elements[i] = prev_first;
            }
        }
        Self {
            width,
            height,
            bucket_size,
            bucket_heads,
            next_elements,
            elements: elements.to_vec(),
        }
    }
    fn to_idx(width: usize, height: usize, (x, y): (usize, usize)) -> usize {
        if x < width && y < height {
            x + y * width
        } else {
            let x = if x >= width { width - 1 } else { x };
            let y = if y >= height { height - 1 } else { y };
            x + y * width
        }
    }
    pub fn get_bucket_contents(
        &self,
        pos: &Point2<f32>,
        dx: i64,
        dy: i64,
    ) -> Option<Vec<(H, Point2<f32>, f32)>> {
        let o_bucket = (
            pos[0] as usize / self.bucket_size,
            pos[1] as usize / self.bucket_size,
        );
        let bucket = (o_bucket.0 as i64 + dx, o_bucket.1 as i64 + dy);
        // Make sure the neighbor is within bounds
        if bucket.0 < 0 || bucket.1 < 0 {
            return None;
        }
        let bucket = (bucket.0 as usize, bucket.1 as usize);
        if bucket.0 >= self.width || bucket.1 >= self.height {
            return None;
        }

        if let Some(mut idx) = self.bucket_heads[Self::to_idx(self.width, self.height, bucket)] {
            let mut bucket_contents = Vec::with_capacity(self.next_elements.len());
            bucket_contents.push(self.elements[idx]);
            while let Some(next_idx) = self.next_elements[idx] {
                bucket_contents.push(self.elements[next_idx]);
                idx = next_idx;
            }
            Some(bucket_contents)
        } else {
            None
        }
    }
}

// TODO: Does this force not get updated when we add and remove particles? Does it only affect the
// original particles?
struct VanDerWaalsForce {
    magnitude: f32,
    distance_threshold: usize,
    num_particles: usize,
    world_width: usize,
    world_height: usize,
}
impl<H: BodyHandle> ForceGenerator<f32, H> for VanDerWaalsForce {
    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut dyn BodySet<f32, Handle = H>) {
        // First, put the stupid bodyset into a vector for easier manipulation
        let mut particles = Vec::with_capacity(self.num_particles + 8);
        bodies.foreach(&mut |bh, body: &dyn Body<f32>| {
            // Just use the x-radius, to simplify things
            if let Some(&(_obj_type, radius, _)) = body
                .downcast_ref::<RigidBody<f32>>()
                .and_then(|rigid_body| rigid_body.user_data())
                .and_then(|user_data| user_data.downcast_ref::<(ObjType, f32, f32)>())
            {
                // TODO: deal with non-circle object types
                particles.push((bh, body.part(0).unwrap().center_of_mass(), radius))
            }
        });
        self.num_particles = particles.len();
        let distance_threshold = self.distance_threshold as f32;

        if self.num_particles < 10 {
            for (_, pos1, r1) in particles.iter() {
                for (bh, pos2, r2) in particles.iter() {
                    let distance = pos2 - pos1;
                    let norm = distance.norm();
                    if norm <= 0. || norm > distance_threshold {
                        continue;
                    }
                    let body = bodies.get_mut(*bh).unwrap();
                    // One of those `norm`s is to reduce `distance` to a unit vector
                    let force = Force::linear(
                        self.magnitude * r1 * r2 * distance / ((r1 + r2) * distance.norm().powi(3)),
                    );
                    body.apply_force(0, &force, ForceType::Force, false);
                }
            }
        } else {
            // Put all the particles into cells to reduce the number of comparisons we need to do
            let cells = Buckets2d::new(
                &particles,
                self.distance_threshold,
                self.world_width,
                self.world_height,
            );

            // Because the cell assigments are rounded, we have to check neighbor cells too
            let neighbor_cells_affected = [(0, 0), (1, 0), (1, -1), (0, -1), (-1, -1)];
            particles.iter().for_each(|(_, pos1, r1)| {
                neighbor_cells_affected.iter().for_each(|(dx, dy)| {
                    if let Some(p2s) = cells.get_bucket_contents(pos1, *dx, *dy) {
                        for (bh2, pos2, r2) in p2s {
                            let distance = pos2 - pos1;
                            let norm = distance.norm();
                            if norm <= 0. || norm > distance_threshold {
                                continue;
                            }
                            let body = bodies.get_mut(bh2).unwrap();
                            // One of those `norm`s is to reduce `distance` to a unit vector
                            let force = Force::linear(
                                self.magnitude * r1 * r2 * distance
                                    / ((r1 + r2) * distance.norm().powi(3)),
                            );
                            body.apply_force(0, &force, ForceType::Force, false);
                        }
                    }
                });
            });
        }
    }
}
