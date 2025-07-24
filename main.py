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
BOUNDARY_THICKNESS = 8            # px
SHRINK_RATE = 60.0                # px/sec
GAP_CLOSING_RATE = 0.35           # radians/sec
MIN_GAP_SIZE = 0.17               # radians (~10°)
GAP_ROTATION_RATE = 0.3           # radians/sec
BOUNDARY_RADII = list(range(400, 20, -12))  # 32 rings, last ~32 px

# Chaos kick
CHAOS_GAP_THRESHOLD = 0.35        # radians (~20°)
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
        # Start gap at 1.45 rad (~83°)
        self.gap_size: float = 1.45
        self.gap_closing_rate: float = GAP_CLOSING_RATE
        self.gap_centre_angle: float = random.uniform(0, 2 * math.pi)
        self.gap_rotation_rate: float = GAP_ROTATION_RATE * random.choice([-1, 1])
        self.thickness: int = BOUNDARY_THICKNESS
        self.escaped: bool = False

    def update(self, dt: float):
        if not self.escaped:
            self.current_radius -= self.shrink_rate * dt
            self.gap_size = max(self.gap_size - self.gap_closing_rate * dt, MIN_GAP_SIZE)
            self.gap_centre_angle = angle_normalize(self.gap_centre_angle + self.gap_rotation_rate * dt)

    def mark_escaped(self):
        self.escaped = True

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
        if self.escaped:
            color = ESCAPED_COLOR
            alpha = 80
        else:
            color = BOUNDARY_COLOR
            alpha = 200

        # Arc angles: gap is omitted, draw two arcs
        r = self.current_radius
        thickness = self.thickness
        gap_start = angle_normalize(self.gap_centre_angle - self.gap_size / 2)
        gap_end = angle_normalize(self.gap_centre_angle + self.gap_size / 2)

        # Pygame draw.arc takes rect, start_angle, stop_angle (clockwise, in radians)
        rect = pygame.Rect(0, 0, r*2, r*2)
        rect.center = CENTER

        arcs = []
        if gap_end < gap_start:
            arcs.append((gap_end, gap_start))
        else:
            arcs.append((0, gap_start))
            arcs.append((gap_end, 2 * math.pi))

        surf_arc = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        for start, end in arcs:
            pygame.draw.arc(
                surf_arc, color, rect, start, end, thickness
            )
        surface.blit(surf_arc, (0, 0))

    def ball_hits_wall(self, ball: Ball) -> bool:
        """Check if ball hits wall (not in gap) and not escaped"""
        if self.escaped:
            return False
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
        # Project velocity onto normal
        vel_norm = rel_vel.dot(normal)
        # We want to handle only if ball is moving outward (vel_norm > 0)
        if vel_norm <= 0:
            # Ball moving inward or tangent, skip
            return

        # Reflect and add wall speed, then multiply by restitution
        wall_speed = -self.shrink_rate  # wall moving inward
        new_vel = (ball.vel - 2 * vel_norm * normal) + wall_speed * normal * 0.75
        ball.vel = new_vel * RESTITUTION

        # Reposition ball just inside the wall
        wall_r = self.current_radius - self.thickness / 2
        overlap = (dist + ball.radius) - wall_r
        pushback = overlap + 1.5
        if pushback > 0:
            ball.pos -= normal * pushback

class Simulation:
    def __init__(self):
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption(TITLE)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 28)
        self.font_big = pygame.font.SysFont("consolas", 52, bold=True)
        self.ball = Ball()
        self.boundaries: List[Boundary] = [Boundary(radius) for radius in BOUNDARY_RADII]
        self.running = True
        self.in_freedom = False
        self.chaos_timer = 0.0

    def update(self, dt: float):
        if self.in_freedom:
            return

        # Update boundaries
        for boundary in self.boundaries:
            boundary.update(dt)

        # Update ball physics
        self.ball.update(dt)

        # Handle collisions (innermost first so ball doesn't get stuck between)
        for boundary in reversed(self.boundaries):
            if not boundary.escaped and boundary.ball_hits_wall(self.ball):
                boundary.resolve_collision(self.ball)

        # Mark boundaries as escaped only if ball is outside and angle is within gap
        for boundary in self.boundaries:
            if not boundary.escaped:
                v = self.ball.pos - pygame.Vector2(CENTER)
                dist = v.length()
                theta = angle_normalize(math.atan2(v.y, v.x))
                # Mark escaped if ball is fully outside and angle in gap
                if (
                    boundary.angle_in_gap(theta)
                    and (dist - self.ball.radius > boundary.current_radius + boundary.thickness / 2)
                ):
                    boundary.mark_escaped()

        # Check for freedom
        if all(b.escaped for b in self.boundaries):
            self.in_freedom = True

        # Apply chaos kick if gap is small and ball is still inside
        for boundary in self.boundaries:
            if not boundary.escaped and boundary.gap_size < CHAOS_GAP_THRESHOLD and boundary.ball_is_inside(self.ball):
                self.chaos_timer += dt
                # Once per ~1.25s on average (if threshold is met)
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

        # Draw centre crosshair
        pygame.draw.line(self.screen, CENTRE_COLOR, (CENTER[0] - 9, CENTER[1]), (CENTER[0] + 9, CENTER[1]), 2)
        pygame.draw.line(self.screen, CENTRE_COLOR, (CENTER[0], CENTER[1] - 9), (CENTER[0], CENTER[1] + 9), 2)

        # Draw ball
        self.ball.draw(self.screen)

        # Draw info text
        remaining = sum(not b.escaped for b in self.boundaries)
        txt = self.font.render(f"Boundaries Left: {remaining}", True, TEXT_COLOR)
        self.screen.blit(txt, (16, 14))

        if self.in_freedom:
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