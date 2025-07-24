import pygame
import sys
import math
import random
from typing import List

# --- Constants ---
WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH // 2, HEIGHT // 2)
FPS = 60
TITLE = "Ball Trying to Escape the Circles"

GRAVITY = pygame.Vector2(0, 350)  # px/sec², downward
LINEAR_FRICTION = 0.999           # per frame
RESTITUTION = 0.95                # bounciness for collisions

BALL_RADIUS = 12
BALL_MASS = 1

# Boundary parameters
INITIAL_RINGS = 150
BOUNDARY_THICKNESS = 6            # px
SHRINK_RATE = 18.0                # px/sec (slower)
BOUNDARY_SPACING = 12             # px between ring radii
BASE_GAP_SIZE = 2.9               # radians (~166°) - new constant
GAP_CLOSING_RATE = 0.25           # radians/sec (slower closing)
MIN_GAP_SIZE = 0.34               # radians (~19.5°), doubled
GAP_ROTATION_RATE = 0.3           # radians/sec
BOUNDARY_RADII = [450 - i*BOUNDARY_SPACING for i in range(INITIAL_RINGS)]

# Chaos kick
CHAOS_GAP_THRESHOLD = 0.7         # radians (~40°)
CHAOS_CHANCE_PER_SEC = 0.8        # chance per second when gap is small

# Colors
BG_COLOR = (28, 28, 28)
BALL_COLOR = (240, 240, 255)
BOUNDARY_COLOR = (100, 180, 255)
ESCAPED_COLOR = (180, 220, 220)
CENTRE_COLOR = (255, 120, 120)
TEXT_COLOR = (235, 235, 255)

# --- Helper functions ---

def angle_normalize(theta: float) -> float:
    """Normalize angle to [0, 2*pi)"""
    return theta % (2 * math.pi)

def point_to_angle(x: float, y: float) -> float:
    """Get angle from center to point (x, y) in [0, 2*pi)"""
    dx, dy = x - CENTER[0], y - CENTER[1]
    theta = math.atan2(dy, dx)
    return angle_normalize(theta)

def angle_diff(a: float, b: float) -> float:
    """Smallest difference between two angles"""
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return d

# --- Classes ---

class Ball:
    def __init__(self):
        # Slight randomization for variety
        jitter_x = random.uniform(-32, 32)
        jitter_y = random.uniform(-32, 32)
        self.pos = pygame.Vector2(CENTER[0] + jitter_x, CENTER[1] - 100 + jitter_y)
        vx = random.uniform(-50, 50)
        vy = random.uniform(-20, 20)
        self.vel = pygame.Vector2(vx, vy)
        self.radius = BALL_RADIUS
        self.mass = BALL_MASS

    def update(self, dt: float):
        # Apply gravity
        self.vel += GRAVITY * dt
        # Apply friction
        self.vel *= LINEAR_FRICTION
        # Update position
        self.pos += self.vel * dt

    def draw(self, surface: pygame.Surface):
        pygame.draw.circle(surface, BALL_COLOR, (int(self.pos.x), int(self.pos.y)), self.radius)

class Boundary:
    def __init__(self, radius: float):
        self.current_radius: float = radius
        self.shrink_rate: float = SHRINK_RATE
        # Start gap at BASE_GAP_SIZE
        self.gap_size: float = BASE_GAP_SIZE
        self.gap_closing_rate: float = GAP_CLOSING_RATE
        self.gap_centre_angle: float = random.uniform(0, 2 * math.pi)
        self.gap_rotation_rate: float = GAP_ROTATION_RATE * random.choice([-1, 1])
        self.thickness: int = BOUNDARY_THICKNESS

    def update(self, dt: float):
        self.current_radius -= self.shrink_rate * dt
        self.gap_size = max(self.gap_size - self.gap_closing_rate * dt, MIN_GAP_SIZE)
        self.gap_centre_angle = angle_normalize(self.gap_centre_angle + self.gap_rotation_rate * dt)

    def ball_is_inside(self, ball: Ball) -> bool:
        dist = (ball.pos - pygame.Vector2(CENTER)).length()
        return dist < self.current_radius

    def angle_in_gap(self, theta: float) -> bool:
        """Return True if angle theta is within the current gap."""
        gap_start = angle_normalize(self.gap_centre_angle - self.gap_size / 2)
        gap_end = angle_normalize(self.gap_centre_angle + self.gap_size / 2)
        if gap_start < gap_end:
            return gap_start <= theta <= gap_end
        else:
            # Wraps around 2pi
            return theta >= gap_start or theta <= gap_end

    def draw(self, surface: pygame.Surface):
        # Draw arc segments directly (no extra surface)
        color = BOUNDARY_COLOR
        r = self.current_radius
        thickness = self.thickness
        gap_start = angle_normalize(self.gap_centre_angle - self.gap_size / 2)
        gap_end = angle_normalize(self.gap_centre_angle + self.gap_size / 2)

        rect = pygame.Rect(0, 0, r*2, r*2)
        rect.center = CENTER

        arcs = []
        if gap_end < gap_start:
            arcs.append((gap_end, gap_start))
        else:
            arcs.append((0, gap_start))
            arcs.append((gap_end, 2 * math.pi))
        for start, end in arcs:
            pygame.draw.arc(surface, color, rect, start, end, thickness)

    def ball_hits_wall(self, ball: Ball) -> bool:
        """Check if ball hits wall (not in gap)"""
        v = ball.pos - pygame.Vector2(CENTER)
        dist = v.length()
        if dist == 0:
            return False  # Ball is exactly at centre, ignore
        wall_r = self.current_radius - self.thickness / 2
        if dist + ball.radius >= wall_r:
            theta = angle_normalize(math.atan2(v.y, v.x))
            if not self.angle_in_gap(theta):
                return True
        return False

    def resolve_collision(self, ball: Ball):
        """Reflect velocity, add wall shrink speed, and reposition ball just inside boundary."""
        v = ball.pos - pygame.Vector2(CENTER)
        dist = v.length()
        if dist == 0:
            normal = pygame.Vector2(1, 0)
        else:
            normal = v.normalize()
        rel_vel = ball.vel
        vel_norm = rel_vel.dot(normal)
        if vel_norm <= 0:
            return
        wall_speed = -self.shrink_rate
        new_vel = (ball.vel - 2 * vel_norm * normal) + wall_speed * normal * 0.75
        ball.vel = new_vel * RESTITUTION
        wall_r = self.current_radius - self.thickness / 2
        overlap = (dist + ball.radius) - wall_r
        pushback = overlap + 1.5
        if pushback > 0:
            ball.pos -= normal * pushback

class Shard:
    """A short-lived line segment, used for ring shatter effect."""
    def __init__(self, center: pygame.Vector2, radius: float, angle: float, thickness: float):
        self.max_life = 0.5
        self.life = self.max_life
        base = center + pygame.Vector2(math.cos(angle), math.sin(angle)) * (radius - thickness/2)
        tip = center + pygame.Vector2(math.cos(angle), math.sin(angle)) * (radius + thickness/2 + 20)
        self.start = pygame.Vector2(base)
        self.end = pygame.Vector2(tip)
        spread = 80
        v = pygame.Vector2(math.cos(angle), math.sin(angle)).rotate(random.uniform(-24, 24)) * random.uniform(30, spread)
        self.vel = v
        self.wiggle = random.uniform(-2, 2)

    def update(self, dt: float):
        d = self.end - self.start
        move = self.vel * dt
        self.start += move
        self.end += move
        self.life -= dt
        # Optionally wiggle a bit for effect
        wiggle_angle = self.wiggle * math.sin(self.life * 15)
        if wiggle_angle != 0:
            mid = (self.start + self.end) / 2
            perp = pygame.Vector2(-(d.y), d.x).normalize() * wiggle_angle
            self.start -= perp / 2
            self.end += perp / 2

    def draw(self, surface: pygame.Surface):
        fade = max(0, min(1, self.life / self.max_life))
        color = (255, 220, 60 + int(100 * fade), int(180*fade) + 60)
        # pygame does not support per-line alpha, so just fade color (no alpha channel, just RGB)
        c = (int(255*fade), int(200*fade), int(60 + 100*fade))
        pygame.draw.line(surface, c, (int(self.start.x), int(self.start.y)), (int(self.end.x), int(self.end.y)), 2)

class Simulation:
    def __init__(self):
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption(TITLE)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 28)
        self.font_big = pygame.font.SysFont("consolas", 52, bold=True)
        self.ball = Ball()
        self.boundaries: List[Boundary] = [Boundary(radius) for radius in BOUNDARY_RADII]
        self.shards: List[Shard] = []
        self.running = True
        self.in_freedom = False
        self.chaos_timer = 0.0

        self.total_escaped = 0
        self.spawn_counter = 0
        self.outside = False
        self.ball_free_velocity = None  # Used in freedom state

    def spawn_new_boundaries(self, count: int):
        outer = max((b.current_radius for b in self.boundaries), default=450)
        for i in range(count):
            new_r = outer + (i + 1) * BOUNDARY_SPACING
            self.boundaries.append(Boundary(new_r))

    def spawn_shards(self, boundary: Boundary):
        # Spawn 16 evenly spaced shards
        for i in range(16):
            angle = 2 * math.pi * i / 16
            self.shards.append(Shard(
                pygame.Vector2(CENTER), boundary.current_radius, angle, boundary.thickness
            ))

    def update(self, dt: float):
        # Shards always update
        for shard in self.shards:
            shard.update(dt)
        self.shards = [s for s in self.shards if s.life > 0]

        if self.outside:
            return

        if self.in_freedom:
            self.ball.pos += self.ball_free_velocity * dt
            if (self.ball.pos.x < -BALL_RADIUS or self.ball.pos.x > WIDTH + BALL_RADIUS or
                self.ball.pos.y < -BALL_RADIUS or self.ball.pos.y > HEIGHT + BALL_RADIUS):
                self.outside = True
            return

        # Update boundaries
        for boundary in self.boundaries:
            boundary.update(dt)

        # Update ball physics
        self.ball.update(dt)

        # Handle collisions (innermost first so ball doesn't get stuck between)
        for boundary in reversed(self.boundaries):
            if boundary.ball_hits_wall(self.ball):
                boundary.resolve_collision(self.ball)

        # Mark and remove boundaries escaped this frame, spawn shards
        escaped_this_frame = 0
        new_boundaries = []
        for boundary in self.boundaries:
            dist = (self.ball.pos - pygame.Vector2(CENTER)).length()
            if dist - self.ball.radius > boundary.current_radius + boundary.thickness / 2:
                self.spawn_shards(boundary)
                escaped_this_frame += 1
                # Not added to new_boundaries: removes it
            else:
                new_boundaries.append(boundary)
        self.boundaries = new_boundaries

        if escaped_this_frame > 0:
            self.total_escaped += escaped_this_frame
            while self.total_escaped - self.spawn_counter >= 3:
                self.spawn_counter += 3
                self.spawn_new_boundaries(2)

        # Freedom state: if all boundaries are escaped
        if not self.in_freedom and len(self.boundaries) == 0:
            self.in_freedom = True
            self.ball_free_velocity = self.ball.vel.copy()

        # Apply chaos kick if gap is small and ball is still inside
        for boundary in self.boundaries:
            if boundary.gap_size < CHAOS_GAP_THRESHOLD and boundary.ball_is_inside(self.ball):
                self.chaos_timer += dt
                prob = 1 - math.exp(-CHAOS_CHANCE_PER_SEC * self.chaos_timer)
                if random.random() < prob:
                    angle = random.uniform(0, 2 * math.pi)
                    mag = random.uniform(80, 140)
                    chaos = pygame.Vector2(math.cos(angle), math.sin(angle)) * mag
                    self.ball.vel += chaos
                    self.chaos_timer = 0.0
                break
        else:
            self.chaos_timer = 0.0

    def draw(self):
        self.screen.fill(BG_COLOR)

        # Draw boundaries
        for boundary in self.boundaries:
            boundary.draw(self.screen)

        # Draw shards overlay
        for shard in self.shards:
            shard.draw(self.screen)

        # Draw centre crosshair
        pygame.draw.line(self.screen, CENTRE_COLOR, (CENTER[0] - 9, CENTER[1]), (CENTER[0] + 9, CENTER[1]), 2)
        pygame.draw.line(self.screen, CENTRE_COLOR, (CENTER[0], CENTER[1] - 9), (CENTER[0], CENTER[1] + 9), 2)

        # Draw ball
        self.ball.draw(self.screen)

        # Draw info text
        remaining = len(self.boundaries)
        txt = self.font.render(f"Boundaries Left: {remaining}", True, TEXT_COLOR)
        self.screen.blit(txt, (16, 14))

        if self.outside:
            t = self.font_big.render("Escaped the Map!", True, (255, 240, 100))
            rect = t.get_rect(center=(WIDTH // 2, HEIGHT // 2 - 20))
            self.screen.blit(t, rect)
        elif self.in_freedom:
            t = self.font_big.render("Freedom!", True, (255, 240, 100))
            rect = t.get_rect(center=(WIDTH // 2, HEIGHT // 2 - 20))
            self.screen.blit(t, rect)

        pygame.display.flip()

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False

def main():
    pygame.init()
    sim = Simulation()

    while sim.running:
        dt_ms = sim.clock.tick(FPS)
        dt = dt_ms / 1000.0  # seconds

        sim.handle_events()
        sim.update(dt)
        sim.draw()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()