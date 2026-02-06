import pygame
import math
import random

# Initialize Pygame
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Physics constants
GRAVITY = 9.8  # m/s²
DT = 0.016  # ~60 FPS
FRICTION = 0.98  # Ground friction
AIR_RESISTANCE = 0.999  # Damping in air
RESTITUTION = 0.8  # Bounciness
WIND_STRENGTH = 0.5  # Random wind force
MAGNETISM = 50  # Attractive force between objects (optional)

class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vector2(self.x * scalar, self.y * scalar)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def length(self):
        return math.sqrt(self.x**2 + self.y**2)

    def normalize(self):
        l = self.length()
        return Vector2(self.x / l, self.y / l) if l > 0 else Vector2(0, 0)

class PhysicsObject:
    def __init__(self, x, y, mass=1, restitution=RESTITUTION):
        self.pos = Vector2(x, y)
        self.old_pos = Vector2(x, y)  # For Verlet integration
        self.vel = Vector2(0, 0)
        self.mass = mass
        self.restitution = restitution
        self.forces = Vector2(0, 0)
        self.angle = 0  # Rotation angle in radians
        self.angular_vel = 0  # Angular velocity
        self.torque = 0  # Torque
        self.moment_of_inertia = mass * 100  # Simplified MOI

    def apply_force(self, force):
        self.forces = self.forces + force

    def apply_torque(self, torque):
        self.torque += torque

    def update(self):
        # Verlet integration for stability
        temp = self.pos
        accel = self.forces * (1 / self.mass) + Vector2(0, GRAVITY)
        self.pos = self.pos * 2 - self.old_pos + accel * (DT * DT)
        self.old_pos = temp
        self.vel = (self.pos - self.old_pos) * (1 / DT)
        # Air resistance
        self.vel = self.vel * AIR_RESISTANCE
        # Reset forces
        self.forces = Vector2(0, 0)

        # Angular update
        angular_accel = self.torque / self.moment_of_inertia
        self.angular_vel += angular_accel * DT
        self.angle += self.angular_vel * DT
        self.angular_vel *= 0.99  # Angular damping
        self.torque = 0

    def constrain_to_bounds(self):
        if self.pos.x < 0:
            self.pos.x = 0
            self.vel.x *= -self.restitution
        elif self.pos.x > WIDTH:
            self.pos.x = WIDTH
            self.vel.x *= -self.restitution
        if self.pos.y < 0:
            self.pos.y = 0
            self.vel.y *= -self.restitution
        elif self.pos.y > HEIGHT:
            self.pos.y = HEIGHT
            self.vel.y *= -self.restitution
            self.vel.x *= FRICTION  # Ground friction

class Circle(PhysicsObject):
    def __init__(self, x, y, radius, mass=1):
        super().__init__(x, y, mass)
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, (255, 0, 0), (int(self.pos.x), int(self.pos.y)), self.radius)

class Rectangle(PhysicsObject):
    def __init__(self, x, y, width, height, mass=1):
        super().__init__(x, y, mass)
        self.width = width
        self.height = height

    def draw(self, screen):
        # Create rotated rectangle points
        half_w = self.width / 2
        half_h = self.height / 2
        points = [
            Vector2(-half_w, -half_h),
            Vector2(half_w, -half_h),
            Vector2(half_w, half_h),
            Vector2(-half_w, half_h)
        ]
        rotated_points = []
        cos_a = math.cos(self.angle)
        sin_a = math.sin(self.angle)
        for p in points:
            rx = p.x * cos_a - p.y * sin_a
            ry = p.x * sin_a + p.y * cos_a
            rotated_points.append((int(self.pos.x + rx), int(self.pos.y + ry)))
        pygame.draw.polygon(screen, (0, 255, 0), rotated_points)

class Triangle(PhysicsObject):
    def __init__(self, x, y, size, mass=1):
        super().__init__(x, y, mass)
        self.size = size
        self.vertices = [
            Vector2(0, -size),
            Vector2(-size * 0.866, size * 0.5),
            Vector2(size * 0.866, size * 0.5)
        ]

    def draw(self, screen):
        rotated_points = []
        cos_a = math.cos(self.angle)
        sin_a = math.sin(self.angle)
        for v in self.vertices:
            rx = v.x * cos_a - v.y * sin_a
            ry = v.x * sin_a + v.y * cos_a
            rotated_points.append((int(self.pos.x + rx), int(self.pos.y + ry)))
        pygame.draw.polygon(screen, (0, 0, 255), rotated_points)

class Spring:
    def __init__(self, obj1, obj2, rest_length, stiffness=100, damping=10):
        self.obj1 = obj1
        self.obj2 = obj2
        self.rest_length = rest_length
        self.stiffness = stiffness
        self.damping = damping

    def apply_force(self):
        diff = self.obj2.pos - self.obj1.pos
        dist = diff.length()
        if dist == 0: return
        force = diff.normalize() * (self.stiffness * (dist - self.rest_length))
        # Damping
        rel_vel = self.obj2.vel - self.obj1.vel
        force = force + rel_vel * self.damping
        self.obj1.apply_force(force)
        self.obj2.apply_force(force * -1)

    def draw(self, screen):
        pygame.draw.line(screen, (255, 255, 0), (int(self.obj1.pos.x), int(self.obj1.pos.y)), (int(self.obj2.pos.x), int(self.obj2.pos.y)), 2)

# Collision detection and resolution
def circle_circle_collision(c1, c2):
    diff = c1.pos - c2.pos
    dist = diff.length()
    if dist < c1.radius + c2.radius:
        overlap = c1.radius + c2.radius - dist
        normal = diff.normalize()
        c1.pos = c1.pos + normal * (overlap / 2)
        c2.pos = c2.pos - normal * (overlap / 2)
        # Impulse
        rel_vel = c1.vel - c2.vel
        impulse = normal * (rel_vel.dot(normal) * (1 + min(c1.restitution, c2.restitution)) / (1/c1.mass + 1/c2.mass))
        c1.vel = c1.vel - impulse * (1 / c1.mass)
        c2.vel = c2.vel + impulse * (1 / c2.mass)

def circle_rect_collision(c, r):
    # Simplified AABB for rectangle
    closest_x = max(r.pos.x - r.width/2, min(c.pos.x, r.pos.x + r.width/2))
    closest_y = max(r.pos.y - r.height/2, min(c.pos.y, r.pos.y + r.height/2))
    diff = Vector2(c.pos.x - closest_x, c.pos.y - closest_y)
    dist = diff.length()
    if dist < c.radius:
        normal = diff.normalize()
        overlap = c.radius - dist
        c.pos = c.pos + normal * overlap
        # Impulse (simplified)
        rel_vel = c.vel
        impulse = normal * (rel_vel.dot(normal) * (1 + c.restitution) / (1/c.mass))
        c.vel = c.vel - impulse * (1 / c.mass)

def point_in_triangle(p, a, b, c):
    def sign(p1, p2, p3):
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y)
    d1 = sign(p, a, b)
    d2 = sign(p, b, c)
    d3 = sign(p, c, a)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)

def triangle_circle_collision(t, c):
    # Get rotated vertices
    cos_a = math.cos(t.angle)
    sin_a = math.sin(t.angle)
    vertices = []
    for v in t.vertices:
        rx = v.x * cos_a - v.y * sin_a
        ry = v.x * sin_a + v.y * cos_a
        vertices.append(t.pos + Vector2(rx, ry))
    
    # Check if circle center is inside triangle
    if point_in_triangle(c.pos, vertices[0], vertices[1], vertices[2]):
        # Push out
        normal = Vector2(0, -1)  # Simplified
        overlap = c.radius
        c.pos = c.pos + normal * overlap
        # Impulse
        rel_vel = c.vel
        impulse = normal * (rel_vel.dot(normal) * (1 + c.restitution) / (1/c.mass))
        c.vel = c.vel - impulse * (1 / c.mass)
        return
    
    # Check edges
    for i in range(3):
        a = vertices[i]
        b = vertices[(i+1)%3]
        # Closest point on line segment
        ab = b - a
        ac = c.pos - a
        proj = ac.dot(ab) / ab.dot(ab)
        proj = max(0, min(1, proj))
        closest = a + ab * proj
        diff = c.pos - closest
        dist = diff.length()
        if dist < c.radius:
            normal = diff.normalize()
            overlap = c.radius - dist
            c.pos = c.pos + normal * overlap
            # Impulse
            rel_vel = c.vel
            impulse = normal * (rel_vel.dot(normal) * (1 + c.restitution) / (1/c.mass))
            c.vel = c.vel - impulse * (1 / c.mass)
            break

def triangle_rect_collision(t, r):
    # Simplified: treat as AABB for now
    # Could implement SAT for better accuracy
    pass  # Skip for simplicity

# Spatial partitioning for efficiency
GRID_SIZE = 50
def get_grid_key(obj):
    return (int(obj.pos.x // GRID_SIZE), int(obj.pos.y // GRID_SIZE))

def check_collisions(objects):
    grid = {}
    for obj in objects:
        key = get_grid_key(obj)
        if key not in grid:
            grid[key] = []
        grid[key].append(obj)
    for cell in grid.values():
        for i in range(len(cell)):
            for j in range(i + 1, len(cell)):
                o1, o2 = cell[i], cell[j]
                if isinstance(o1, Circle) and isinstance(o2, Circle):
                    circle_circle_collision(o1, o2)
                elif isinstance(o1, Circle) and isinstance(o2, Rectangle):
                    circle_rect_collision(o1, o2)
                elif isinstance(o1, Rectangle) and isinstance(o2, Circle):
                    circle_rect_collision(o2, o1)
                elif isinstance(o1, Rectangle) and isinstance(o2, Rectangle):
                    rect_rect_collision(o1, o2)
                elif isinstance(o1, Triangle) and isinstance(o2, Circle):
                    triangle_circle_collision(o1, o2)
                elif isinstance(o1, Circle) and isinstance(o2, Triangle):
                    triangle_circle_collision(o2, o1)
                elif isinstance(o1, Triangle) and isinstance(o2, Rectangle):
                    triangle_rect_collision(o1, o2)
                elif isinstance(o1, Rectangle) and isinstance(o2, Triangle):
                    triangle_rect_collision(o2, o1)
                # Triangle-triangle would be complex, skip for now

# Create initial objects and springs
objects = [
    Circle(200, 100, 20, mass=1),
    Rectangle(400, 100, 40, 40, mass=2),
    Circle(600, 100, 25, mass=1.5),
    Triangle(300, 200, 30, mass=1.2),
    Triangle(500, 200, 25, mass=0.8)
]
springs = [
    Spring(objects[0], objects[1], 200, stiffness=50),  # Connect first two
]

# Mouse interaction
dragging = None
drag_offset = Vector2(0, 0)

running = True
while running:
    screen.fill((0, 0, 0))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = Vector2(*pygame.mouse.get_pos())
            if event.button == 1:  # Left click: drag
                for obj in objects:
                    if isinstance(obj, Circle) and (obj.pos - mouse_pos).length() < obj.radius:
                        dragging = obj
                        drag_offset = obj.pos - mouse_pos
                        break
                    elif isinstance(obj, Rectangle) and (abs(obj.pos.x - mouse_pos.x) < obj.width/2 and abs(obj.pos.y - mouse_pos.y) < obj.height/2):
                        dragging = obj
                        drag_offset = obj.pos - mouse_pos
                        break
                    elif isinstance(obj, Triangle):
                        # Check if mouse is inside triangle
                        cos_a = math.cos(obj.angle)
                        sin_a = math.sin(obj.angle)
                        vertices = []
                        for v in obj.vertices:
                            rx = v.x * cos_a - v.y * sin_a
                            ry = v.x * sin_a + v.y * cos_a
                            vertices.append(obj.pos + Vector2(rx, ry))
                        if point_in_triangle(mouse_pos, vertices[0], vertices[1], vertices[2]):
                            dragging = obj
                            drag_offset = obj.pos - mouse_pos
                            break
            elif event.button == 3:  # Right click: add circle
                objects.append(Circle(mouse_pos.x, mouse_pos.y, random.randint(10, 30), mass=random.uniform(0.5, 2)))
        elif event.type == pygame.MOUSEBUTTONUP:
            dragging = None

    if dragging:
        mouse_pos = Vector2(*pygame.mouse.get_pos())
        dragging.pos = mouse_pos + drag_offset
        dragging.vel = Vector2(0, 0)

    # Apply global forces
    wind = Vector2(random.uniform(-WIND_STRENGTH, WIND_STRENGTH), 0)
    for obj in objects:
        obj.apply_force(wind)
        # Optional magnetism
        for other in objects:
            if obj != other:
                diff = other.pos - obj.pos
                dist = diff.length()
                if dist > 0:
                    force = diff.normalize() * (MAGNETISM / (dist * dist))
                    obj.apply_force(force)

    # Update springs
    for spring in springs:
        spring.apply_force()

    # Update objects
    for obj in objects:
        obj.update()
        obj.constrain_to_bounds()

    # Check collisions
    check_collisions(objects)

    # Draw
    for spring in springs:
        spring.draw(screen)
    for obj in objects:
        obj.draw(screen)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
