import os
os.environ["SDL_AUDIODRIVER"] = "dummy"  # Disable audio for lightweight run

import pygame
import random
import math

# ==== CONSTANTS ====
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
FPS = 60

BALL_RADIUS = 8
BALL_COLOR = (240, 60, 60)
BALL_JITTER_FORCE = 600  # px/s^2
BALL_MAX_SPEED = 360     # px/s

OBSTACLE_COUNT = 14
OBSTACLE_BASE_RADIUS_RANGE = (40, 70)
OBSTACLE_PULSE_AMPLITUDE_RANGE = (5, 15)
OBSTACLE_PULSE_SPEED = 1.7  # radians/sec
OBSTACLE_DRIFT_SPEED_RANGE = (10, 30)
OBSTACLE_COLOR = (60, 200, 200, 105)  # RGBA with alpha for transparency

BG_COLOR = (13, 20, 28)

# ==== UTILS ====
def random_pos_away_from(center, min_dist, max_x, max_y):
    """Return a random pos at least min_dist from center and inside bounds."""
    while True:
        x = random.uniform(min_dist, max_x - min_dist)
        y = random.uniform(min_dist, max_y - min_dist)
        pos = pygame.math.Vector2(x, y)
        if center is None or pos.distance_to(center) >= min_dist:
            return pos

def circles_overlap(pos1, r1, pos2, r2):
    return pos1.distance_to(pos2) < (r1 + r2)

# ==== OBSTACLE CLASS ====
class Obstacle:
    def __init__(self, pos, base_radius, pulse_amp, pulse_phase, drift):
        self.base_radius = base_radius
        self.pulse_amp = pulse_amp
        self.pulse_phase = pulse_phase
        self.drift = drift
        self.pos = pos
        self.radius = base_radius  # will be updated every frame

    def update(self, dt, elapsed):
        # Pulse
        self.radius = self.base_radius + math.sin(elapsed * OBSTACLE_PULSE_SPEED + self.pulse_phase) * self.pulse_amp
        # Drift
        self.pos += self.drift * dt

        # Window border bounce for center
        if self.pos.x - self.radius < 0:
            self.pos.x = self.radius
            self.drift.x *= -1
        elif self.pos.x + self.radius > WINDOW_WIDTH:
            self.pos.x = WINDOW_WIDTH - self.radius
            self.drift.x *= -1
        if self.pos.y - self.radius < 0:
            self.pos.y = self.radius
            self.drift.y *= -1
        elif self.pos.y + self.radius > WINDOW_HEIGHT:
            self.pos.y = WINDOW_HEIGHT - self.radius
            self.drift.y *= -1

    def draw(self, surf):
        # Draw with alpha using a temporary surface
        s = pygame.Surface((self.radius * 2 + 2, self.radius * 2 + 2), pygame.SRCALPHA)
        pygame.draw.circle(s, OBSTACLE_COLOR, (self.radius + 1, self.radius + 1), self.radius)
        surf.blit(s, (self.pos.x - self.radius - 1, self.pos.y - self.radius - 1))

# ==== BALL CLASS ====
class Ball:
    def __init__(self, pos):
        self.pos = pos
        angle = random.uniform(0, 2 * math.pi)
        speed = random.uniform(BALL_MAX_SPEED * 0.3, BALL_MAX_SPEED * 0.7)
        self.vel = pygame.math.Vector2(math.cos(angle), math.sin(angle)) * speed

    def apply_jitter(self, dt):
        # Random 2D acceleration
        theta = random.uniform(0, 2 * math.pi)
        force = pygame.math.Vector2(math.cos(theta), math.sin(theta)) * BALL_JITTER_FORCE
        self.vel += force * dt
        # Cap speed
        if self.vel.length() > BALL_MAX_SPEED:
            self.vel.scale_to_length(BALL_MAX_SPEED)

    def update(self, dt):
        self.pos += self.vel * dt

    def window_bounce(self):
        bounced = False
        if self.pos.x - BALL_RADIUS < 0:
            self.pos.x = BALL_RADIUS
            self.vel.x = abs(self.vel.x)
            bounced = True
        elif self.pos.x + BALL_RADIUS > WINDOW_WIDTH:
            self.pos.x = WINDOW_WIDTH - BALL_RADIUS
            self.vel.x = -abs(self.vel.x)
            bounced = True

        if self.pos.y - BALL_RADIUS < 0:
            self.pos.y = BALL_RADIUS
            self.vel.y = abs(self.vel.y)
            bounced = True
        elif self.pos.y + BALL_RADIUS > WINDOW_HEIGHT:
            self.pos.y = WINDOW_HEIGHT - BALL_RADIUS
            self.vel.y = -abs(self.vel.y)
            bounced = True
        return bounced

    def obstacle_bounce(self, obstacle):
        # Check collision
        dist = self.pos.distance_to(obstacle.pos)
        min_dist = BALL_RADIUS + obstacle.radius
        if dist < min_dist:
            # Project ball out
            normal = (self.pos - obstacle.pos).normalize()
            self.pos = obstacle.pos + normal * min_dist
            # Reflect velocity
            self.vel = self.vel.reflect(normal)
            # Cap speed after bounce
            if self.vel.length() > BALL_MAX_SPEED:
                self.vel.scale_to_length(BALL_MAX_SPEED)
            return True
        return False

    def draw(self, surf):
        pygame.draw.circle(surf, BALL_COLOR, (int(self.pos.x), int(self.pos.y)), BALL_RADIUS)

# ==== MAIN ====
def main():
    pygame.init()
    pygame.display.set_caption("Chaotic Escape")
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()

    # Place obstacles without overlap
    obstacles = []
    max_attempts = 1500
    for i in range(OBSTACLE_COUNT):
        for attempt in range(max_attempts):
            base_radius = random.uniform(*OBSTACLE_BASE_RADIUS_RANGE)
            pos = pygame.math.Vector2(
                random.uniform(base_radius, WINDOW_WIDTH - base_radius),
                random.uniform(base_radius, WINDOW_HEIGHT - base_radius)
            )
            pulse_amp = random.uniform(*OBSTACLE_PULSE_AMPLITUDE_RANGE)
            pulse_phase = random.uniform(0, 2 * math.pi)
            drift_angle = random.uniform(0, 2 * math.pi)
            drift_speed = random.uniform(*OBSTACLE_DRIFT_SPEED_RANGE)
            drift = pygame.math.Vector2(math.cos(drift_angle), math.sin(drift_angle)) * drift_speed
            # Check for overlap
            ok = True
            for o in obstacles:
                if circles_overlap(pos, base_radius + pulse_amp, o.pos, o.base_radius + o.pulse_amp):
                    ok = False
                    break
            if ok:
                obstacles.append(Obstacle(pos, base_radius, pulse_amp, pulse_phase, drift))
                break
        else:
            raise RuntimeError("Could not place all obstacles without overlap. Try reducing OBSTACLE_COUNT.")

    # Place ball away from obstacles
    for attempt in range(max_attempts):
        ball_pos = pygame.math.Vector2(
            random.uniform(BALL_RADIUS, WINDOW_WIDTH - BALL_RADIUS),
            random.uniform(BALL_RADIUS, WINDOW_HEIGHT - BALL_RADIUS)
        )
        ok = True
        for o in obstacles:
            if circles_overlap(ball_pos, BALL_RADIUS, o.pos, o.base_radius + o.pulse_amp):
                ok = False
                break
        if ok:
            ball = Ball(ball_pos)
            break
    else:
        raise RuntimeError("Could not place ball away from obstacles.")

    running = True
    elapsed = 0.0

    while running:
        dt = clock.tick(FPS) / 1000
        elapsed += dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update obstacles
        for o in obstacles:
            o.update(dt, elapsed)

        # Update ball
        ball.apply_jitter(dt)
        ball.update(dt)
        ball.window_bounce()
        for o in obstacles:
            ball.obstacle_bounce(o)

        # Drawing
        screen.fill(BG_COLOR)
        for o in obstacles:
            o.draw(screen)
        ball.draw(screen)
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()