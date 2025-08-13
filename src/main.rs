#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

#[cfg(feature = "v2")]
use microbit::{
    display::blocking::Display,
    hal::{Timer, twim},
    pac::twim0::frequency::FREQUENCY_A,
};

use lsm303agr::{AccelOutputDataRate, Lsm303agr, MagOutputDataRate};
use micromath::F32Ext;

// some constants for simulation
const SIM_WIDTH: usize = 5;
const SIM_HEIGHT: usize = 5;

const WIDTH: usize = SIM_WIDTH + 2; // grid width including solid border
const HEIGHT: usize = SIM_HEIGHT + 2; // grid height including solid border
const NUM_CELLS: usize = WIDTH * HEIGHT;
const NUM_PARTICLES: usize = 25; // max number of particles in the simulation

const DT: f32 = 0.6; // time step
const PARTICLE_RADIUS: f32 = 0.2;
const COLLISION_ITERS: usize = 8;
const INCOMPRESSIBILITY_ITERATIONS: usize = 10;
const OVER_RELAXATION: f32 = 1.9; // between 1 and 2, higher is faster but can be unstable
const FLIP_RATIO: f32 = 0.8;
const DAMPING: f32 = 0.99;

#[derive(Clone, Copy, PartialEq, Debug)]
enum CellType {
    Fluid,
    Air,
    Solid,
}

#[derive(Clone, Copy, Debug, Default)]
struct Particle {
    x: f32,
    y: f32,
    vx: f32,
    vy: f32,
}

struct FluidSim {
    // need number of cells
    // need height (cell size)
    // need width (cell size)
    // i think i need velocity for each cell
    // i think i need pressure for each cell
    // i think i need type for each cell (solid, air, fluid)
    // i think i need particle count for each cell
    // slice of particles in general
    particles: [Particle; NUM_PARTICLES], // particles in the simulation

    u: [f32; NUM_CELLS], // horizontal velocity
    v: [f32; NUM_CELLS], // vertical velocity
    prev_u: [f32; NUM_CELLS], // previous horizontal velocity - copy for advecting
    prev_v: [f32; NUM_CELLS], // previous vertical velocity - copy for advecting

    p: [f32; NUM_CELLS], // pressure
    s: [f32; NUM_CELLS], // cell solid mask (0.0 for solid, 1.0 for fluid/air)

    cell_types: [CellType; NUM_CELLS], // type of each cell

    display_density: [f32; NUM_CELLS],
}

impl FluidSim {
    fn new() -> Self {
        let mut particles = [Particle::default(); NUM_PARTICLES];
        // basically just initialize particles together in a block
        for i in 0..NUM_PARTICLES {
            particles[i] = Particle {
                x: (i % 5) as f32 + 1.5,
                y: (i / 5) as f32 + 1.5,
                vx: 0.0,
                vy: 0.0,
            };
        }

        let mut s = [1.0; NUM_CELLS];
        // create border of 'solid' cells: 0th column and row, last column and row
        for i in 0..WIDTH {
            for j in 0..HEIGHT {
                let index = j * WIDTH + i;
                if i == 0 || i == WIDTH - 1 || j == 0 || j == HEIGHT - 1 {
                    s[index] = 0.0;
                }
            }
        }

        FluidSim {
            particles,
            u: [0.0; NUM_CELLS],
            v: [0.0; NUM_CELLS],
            prev_u: [0.0; NUM_CELLS],
            prev_v: [0.0; NUM_CELLS],
            p: [0.0; NUM_CELLS],
            s,
            cell_types: [CellType::Air; NUM_CELLS],
            display_density: [0.0; NUM_CELLS],
        }
    }

    fn step(&mut self, accel_x: i32, accel_y: i32) {
        // aply external forces/gravity (tilt gathered accelerometer) and chatgpt reccomends damping to particles
        self.integrate_extenal_forces(accel_x, accel_y);

        // transfer particle velocities to the grid (P2G)
        self.apply_particles_to_grid();

        // apply/solve for incompressibility
        self.apply_incompressibility_to_grid();

        // transfer grid velocity changes back to particles (G2P)
        self.apply_grid_velocities_to_particles();

        // advect (move) particles and resolve collisions
        self.advect_and_collide_particles();

        // update the grid for display
        self.update_display_grid();
    }

    fn integrate_extenal_forces(&mut self, accel_x: i32, accel_y: i32) {
        // for each particle
        // add gravity * dt to velocities (in the direction of the accelerometer x and y)
        let fx = accel_x as f32 / -1024.0;
        let fy = accel_y as f32 / -1024.0;

        for p in self.particles.iter_mut() {
            p.vx += fx * DT;
            p.vy += fy * DT;
            p.vx *= DAMPING;
            p.vy *= DAMPING;
        }
    }

    fn apply_particles_to_grid(&mut self) {
        // for each *water* particle
        // find the grid cell it is in
        // add its velocity to the grid cell using weighted sum
        self.prev_u.copy_from_slice(&self.u);
        self.prev_v.copy_from_slice(&self.v);

        self.u.fill(0.0);
        self.v.fill(0.0);
        let mut u_weights = [0.0; NUM_CELLS];
        let mut v_weights = [0.0; NUM_CELLS];

        for i in 0..NUM_CELLS {
            self.cell_types[i] = if self.s[i] == 0.0 {
                CellType::Solid
            } else {
                CellType::Air
            };
        }

        // determine which cells are fluid based on particle location
        for p in self.particles.iter() {
            let cell_x = p.x.floor() as usize;
            let cell_y = p.y.floor() as usize;
            if cell_x < WIDTH && cell_y < HEIGHT {
                let index = cell_y * WIDTH + cell_x;
                if self.cell_types[index] != CellType::Solid {
                    self.cell_types[index] = CellType::Fluid;
                }
            }
        }

        // for u (horizontal velocities)
        for p in self.particles.iter() {
            let cell_x_f = p.x.floor();
            let cell_y_f = (p.y - 0.5).floor();
            let dx = p.x - cell_x_f;
            let dy = (p.y - 0.5) - cell_y_f;
            
            let cell_x = cell_x_f as usize;
            let cell_y = cell_y_f as usize;

            if cell_x + 1 >= WIDTH || cell_y + 1 >= HEIGHT { continue; }

            let bottom_left_w = (1.0 - dx) * (1.0 - dy);
            let bottom_right_w = dx * (1.0 - dy);
            let top_left_w = (1.0 - dx) * dy;
            let top_right_w = dx * dy;

            let bottom_left_idx = cell_y * WIDTH + cell_x;
            let bottom_right_idx = cell_y * WIDTH + (cell_x + 1);
            let top_left_idx = (cell_y + 1) * WIDTH + cell_x;
            let top_right_idx = (cell_y + 1) * WIDTH + (cell_x + 1);

            self.u[bottom_left_idx] += p.vx * bottom_left_w; u_weights[bottom_left_idx] += bottom_left_w;
            self.u[bottom_right_idx] += p.vx * bottom_right_w; u_weights[bottom_right_idx] += bottom_right_w;
            self.u[top_left_idx] += p.vx * top_left_w; u_weights[top_left_idx] += top_left_w;
            self.u[top_right_idx] += p.vx * top_right_w; u_weights[top_right_idx] += top_right_w;
        }

        // for v (vertical velocities)
        for p in self.particles.iter() {
            let cell_x_f = (p.x - 0.5).floor();
            let cell_y_f = p.y.floor();
            let dx = (p.x - 0.5) - cell_x_f;
            let dy = p.y - cell_y_f;

            let cell_x = cell_x_f as usize;
            let cell_y = cell_y_f as usize;
            if cell_x + 1 >= WIDTH || cell_y + 1 >= HEIGHT { continue; }

            let bottom_left_w = (1.0 - dx) * (1.0 - dy);
            let bottom_right_w = dx * (1.0 - dy);
            let top_left_w = (1.0 - dx) * dy;
            let top_right_w = dx * dy;

            let bottom_left_idx = cell_y * WIDTH + cell_x;
            let bottom_right_idx = cell_y * WIDTH + (cell_x + 1);
            let top_left_idx = (cell_y + 1) * WIDTH + cell_x;
            let top_right_idx = (cell_y + 1) * WIDTH + (cell_x + 1);

            self.v[bottom_left_idx] += p.vy * bottom_left_w; v_weights[bottom_left_idx] += bottom_left_w;
            self.v[bottom_right_idx] += p.vy * bottom_right_w; v_weights[bottom_right_idx] += bottom_right_w;
            self.v[top_left_idx] += p.vy * top_left_w; v_weights[top_left_idx] += top_left_w;
            self.v[top_right_idx] += p.vy * top_right_w; v_weights[top_right_idx] += top_right_w;
        }

        for i in 0..NUM_CELLS {
            if u_weights[i] > 0.0 { self.u[i] /= u_weights[i]; }
            if v_weights[i] > 0.0 { self.v[i] /= v_weights[i]; }
        }

        // enforce boundary conditions
        for j in 0..HEIGHT {
            for i in 0..WIDTH {
                let index = j * WIDTH + i;
                if self.cell_types[index] == CellType::Solid {
                    self.u[index] = 0.0;
                    self.v[index] = 0.0;
                    if i < WIDTH - 1 { self.u[index + 1] = 0.0; }
                    if j < HEIGHT - 1 { self.v[index + WIDTH] = 0.0; }
                }
            }
        }
    }

    fn apply_incompressibility_to_grid(&mut self) {
        // for each grid cell
        // compute the divergence of the cell (how much it is flowing in or out)
        // for each grid cell
        // adjust the cell's velocity based on the average divergence of its neighbors
        self.p.fill(0.0);

        for _ in 0..INCOMPRESSIBILITY_ITERATIONS {
            for j in 1..HEIGHT - 1 {
                for i in 1..WIDTH - 1 {
                    let index = j * WIDTH + i;
                    if self.cell_types[index] != CellType::Fluid { continue; }

                    let s_left = self.s[index - 1];
                    let s_right = self.s[index + 1];
                    let s_down = self.s[index - WIDTH];
                    let s_up = self.s[index + WIDTH];
                    let s_sum = s_left + s_right + s_down + s_up;

                    if s_sum == 0.0 { continue; }

                    let divergence = self.u[index + 1] - self.u[index]
                        + self.v[index + WIDTH] - self.v[index];

                    let mut pressure_correction = -divergence / s_sum;
                    pressure_correction *= OVER_RELAXATION;

                    self.p[index] += pressure_correction;
                    self.u[index] -= pressure_correction * s_left;
                    self.u[index + 1] += pressure_correction * s_right;
                    self.v[index] -= pressure_correction * s_down;
                    self.v[index + WIDTH] += pressure_correction * s_up;
                }
            }
        }
    }

    fn apply_grid_velocities_to_particles(&mut self) {
        // for each particle
        // find the grid cell it is in
        // set the particle's velocity to the grid cell's velocity
        for p in self.particles.iter_mut() {
            // u component
            let cell_x_f = p.x.floor();
            let cell_y_f = (p.y - 0.5).floor();
            let dx = p.x - cell_x_f;
            let dy = (p.y - 0.5) - cell_y_f;

            let cell_x = cell_x_f as usize;
            let cell_y = cell_y_f as usize;
            if cell_x + 1 >= WIDTH || cell_y + 1 >= HEIGHT { continue; }

            let bottom_left_w = (1.0 - dx) * (1.0 - dy);
            let bottom_right_w = dx * (1.0 - dy);
            let top_left_w = (1.0 - dx) * dy;
            let top_right_w = dx * dy;

            let bottom_left_idx = cell_y * WIDTH + cell_x;
            let bottom_right_idx = cell_y * WIDTH + (cell_x + 1);
            let top_left_idx = (cell_y + 1) * WIDTH + cell_x;
            let top_right_idx = (cell_y + 1) * WIDTH + (cell_x + 1);

            let du = (self.u[bottom_left_idx] - self.prev_u[bottom_left_idx]) * bottom_left_w
                + (self.u[bottom_right_idx] - self.prev_u[bottom_right_idx]) * bottom_right_w
                + (self.u[top_left_idx] - self.prev_u[top_left_idx]) * top_left_w
                + (self.u[top_right_idx] - self.prev_u[top_right_idx]) * top_right_w;
            let pic_vx = self.u[bottom_left_idx] * bottom_left_w
                + self.u[bottom_right_idx] * bottom_right_w
                + self.u[top_left_idx] * top_left_w
                + self.u[top_right_idx] * top_right_w;

            // v component
            let cell_x_f = (p.x - 0.5).floor();
            let cell_y_f = p.y.floor();
            let dx = (p.x - 0.5) - cell_x_f;
            let dy = p.y - cell_y_f;
            
            let cell_x = cell_x_f as usize;
            let cell_y = cell_y_f as usize;
            if cell_x + 1 >= WIDTH || cell_y + 1 >= HEIGHT { continue; }

            let bottom_left_w = (1.0 - dx) * (1.0 - dy);
            let bottom_right_w = dx * (1.0 - dy);
            let top_left_w = (1.0 - dx) * dy;
            let top_right_w = dx * dy;

            let bottom_left_idx = cell_y * WIDTH + cell_x;
            let bottom_right_idx = cell_y * WIDTH + (cell_x + 1);
            let top_left_idx = (cell_y + 1) * WIDTH + cell_x;
            let top_right_idx = (cell_y + 1) * WIDTH + (cell_x + 1);

            let dv = (self.v[bottom_left_idx] - self.prev_v[bottom_left_idx]) * bottom_left_w
                + (self.v[bottom_right_idx] - self.prev_v[bottom_right_idx]) * bottom_right_w
                + (self.v[top_left_idx] - self.prev_v[top_left_idx]) * top_left_w
                + (self.v[top_right_idx] - self.prev_v[top_right_idx]) * top_right_w;
            let pic_vy = self.v[bottom_left_idx] * bottom_left_w
                + self.v[bottom_right_idx] * bottom_right_w
                + self.v[top_left_idx] * top_left_w
                + self.v[top_right_idx] * top_right_w;

            let flip_vx = p.vx + du;
            let flip_vy = p.vy + dv;

            p.vx = (1.0 - FLIP_RATIO) * pic_vx + FLIP_RATIO * flip_vx;
            p.vy = (1.0 - FLIP_RATIO) * pic_vy + FLIP_RATIO * flip_vy;
        }
    }

    fn advect_and_collide_particles(&mut self) {
        // for each particle
        // move the particle based on its velocity
        // if it collides with a wall, stop it
        for p in self.particles.iter_mut() {
            p.x += p.vx * DT;
            p.y += p.vy * DT;
        }
        self.resolve_particle_collisions();
        self.resolve_boundary_collisions();
    }

    fn resolve_particle_collisions(&mut self) {
        let min_dist = 2.0 * PARTICLE_RADIUS;
        let min_dist_sq = min_dist * min_dist;

        for _ in 0..COLLISION_ITERS {
            for i in 0..self.particles.len() {
                for j in (i + 1)..self.particles.len() {
                    let (p1_slice, p2_slice) = self.particles.split_at_mut(j);
                    let p1 = &mut p1_slice[i];
                    let p2 = &mut p2_slice[0];

                    let dx = p2.x - p1.x;
                    let dy = p2.y - p1.y;
                    let dist_sq = dx * dx + dy * dy;

                    if dist_sq > 1e-6 && dist_sq < min_dist_sq {
                        let dist = dist_sq.sqrt();
                        let s = 0.5 * (min_dist - dist) / dist;
                        let push_x = dx * s;
                        let push_y = dy * s;

                        p1.x -= push_x;
                        p1.y -= push_y;
                        p2.x += push_x;
                        p2.y += push_y;
                    }
                }
            }
        }
    }

    fn resolve_boundary_collisions(&mut self) {
        let min_coord = 1.0 + PARTICLE_RADIUS;
        let max_x = (WIDTH - 1) as f32 - PARTICLE_RADIUS;
        let max_y = (HEIGHT - 1) as f32 - PARTICLE_RADIUS;

        for p in self.particles.iter_mut() {
            if p.x < min_coord { p.x = min_coord; p.vx = 0.0; }
            if p.x > max_x { p.x = max_x; p.vx = 0.0; }
            if p.y < min_coord { p.y = min_coord; p.vy = 0.0; }
            if p.y > max_y { p.y = max_y; p.vy = 0.0; }
        }
    }

    fn update_display_grid(&mut self) {
        self.display_density.fill(0.0);
        for p in self.particles.iter() {
            let cell_x = p.x.floor() as usize;
            let cell_y = p.y.floor() as usize;

            if cell_x < WIDTH && cell_y < HEIGHT {
                let index = cell_y * WIDTH + cell_x;
                self.display_density[index] = 1.0;
            }
        }
    }

    fn render(&self, leds: &mut [[u8; 5]; 5]) {
        // mark the grid cells that have particles in them for display
        *leds = [[0; 5]; 5];
        for j in 0..SIM_HEIGHT {
            for i in 0..SIM_WIDTH {
                // Offset by 1 to read from the inner 5x5 grid
                let index = (j + 1) * WIDTH + (i + 1);
                if self.display_density[index] > 0.0 {
                    leds[j][i] = 9;
                }
            }
        }
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = microbit::Board::take().unwrap();

    #[cfg(feature = "v2")]
    let i2c = { twim::Twim::new(board.TWIM0, board.i2c_internal.into(), FREQUENCY_A::K100) };

    #[cfg(not(feature = "v2"))]
    let i2c = { twim::Twim::new(board.TWIM0, board.i2c_internal.into(), FREQUENCY_A::K100) };

    let mut timer = Timer::new(board.TIMER0);
    let mut display = Display::new(board.display_pins);

    let mut sensor = Lsm303agr::new_with_i2c(i2c);
    sensor.init().unwrap();
    // Set magnetometer to 10Hz to gate the main loop
    sensor.set_mag_odr(MagOutputDataRate::Hz10).unwrap();
    // Set accelerometer to a higher rate for more responsive tilting
    sensor.set_accel_odr(AccelOutputDataRate::Hz50).unwrap();
    let mut sensor = sensor.into_mag_continuous().ok().unwrap();

    let mut leds = [[0; 5]; 5];
    let mut fluid_sim = FluidSim::new();

    rprintln!("FLIP/PIC fluid simulation started!");

    loop {
        // some experimentation to get the accelerometer values
        // started with accelerometer because the simualtion is just math
        while !sensor.mag_status().unwrap().xyz_new_data {}
        let accel_data = sensor.accel_data().unwrap();

        fluid_sim.step(-accel_data.x, accel_data.y);
        fluid_sim.render(&mut leds);

        display.show(&mut timer, leds, 50);
    }
}
