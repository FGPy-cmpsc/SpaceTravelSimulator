import vpython as vp
import numpy as np
from pygments.lexer import words
from vpython import scene
import model
import random

vp.scene.title = "Modeling the motion of planets with the gravitational force"
vp.scene.height = 800
vp.scene.width = 1200


def gravitationalForce(p1, p2):
    G = 1  # real-world value is : G = 6.67e-11
    rVector = p1.pos - p2.pos
    rMagnitude = vp.mag(rVector)
    rHat = rVector / rMagnitude
    F = - rHat * G * p1.mass * p2.mass / rMagnitude ** 2
    return F


# Класс для управления частицами выхлопа
class RocketExhaust:
    def __init__(self, rocket):
        self.rocket = rocket
        self.particles = []
        self.max_particles = 300
        self.particle_lifetime = 0.5  # время жизни частицы в секундах

    def create_particle(self):
        # Создаем частицу у основания ракеты
        rocket_bottom = self.rocket.pos - vp.vector(0, self.rocket.size.y / 2, 0)

        # Случайное смещение у основания
        offset_x = random.uniform(-self.rocket.size.x / 3, self.rocket.size.x / 3)
        offset_z = random.uniform(-self.rocket.size.z / 3, self.rocket.size.z / 3)

        # СИНЕВАТЫЕ ЦВЕТА для частиц
        blue_colors = [
            vp.vector(0.0, 0.5, 1.0),  # Ярко-синий
            vp.vector(0.2, 0.6, 1.0),  # Светло-синий
            vp.vector(0.0, 0.4, 0.8),  # Темно-синий
            vp.vector(0.3, 0.7, 1.0),  # Голубоватый
            vp.vector(0.1, 0.3, 0.9)  # Фиолетово-синий
        ]

        particle = vp.sphere(
            pos=rocket_bottom + vp.vector(offset_x, 0, offset_z),
            radius=random.uniform(20,20),
            color=random.choice(blue_colors),
            opacity=random.uniform(0.6, 0.8),
            make_trail=False
        )

        # Начальная скорость частицы (вниз + случайное рассеивание)
        velocity = vp.vector(
            random.uniform(-0.2, 0.2),
            random.uniform(-2.0, -0.5),
            random.uniform(-0.2, 0.2)
        )

        return {'obj': particle, 'velocity': velocity, 'lifetime': 0}

    def update(self, dt):
        # Удаляем старые частицы
        for i in range(len(self.particles) - 1, -1, -1):
            self.particles[i]['lifetime'] += dt
            if self.particles[i]['lifetime'] > self.particle_lifetime:
                self.particles[i]['obj'].visible = False
                del self.particles[i]

        # Добавляем новые частицы
        if len(self.particles) < self.max_particles:
            for _ in range(3):  # создаем несколько частиц за раз
                if len(self.particles) < self.max_particles:
                    self.particles.append(self.create_particle())

        # Обновляем существующие частицы
        for particle in self.particles:
            # Двигаем частицу
            particle['obj'].pos += particle['velocity'] * dt

            # Уменьшаем размер и прозрачность со временем
            age_ratio = particle['lifetime'] / self.particle_lifetime
            particle['obj'].radius *= 0.95
            particle['obj'].opacity *= 0.9

            # Добавляем случайное движение для эффекта турбулентности
            particle['velocity'] += vp.vector(
                random.uniform(-0.1, 0.1),
                random.uniform(-0.05, 0.05),
                random.uniform(-0.1, 0.1)
            ) * dt


t = 0
dt = model.DELTA_TIME
earth = vp.sphere(pos=vp.vector(0, 0, 0), radius=model.RADIUS_OF_EARTH, color=vp.color.green,
                  mass=1000, momentum=vp.vector(0, 0, 0))

rocket = vp.box(
    pos=vp.vector(0, model.RADIUS_OF_EARTH, 0),
    size=vp.vector(model.Rocket.ROCKET_SIZE_DIM_X, model.Rocket.ROCKET_SIZE_DIM_Y, model.Rocket.ROCKET_SIZE_DIM_Z),
    axis=vp.vector(1, 1, 1),
    radius=0.0001,
    color=vp.color.red,
    make_trail=True,
    trail_color=vp.color.cyan
)

# Создаем систему выхлопа
rocket_exhaust = RocketExhaust(rocket)

# Добавляем освещение
vp.distant_light(direction=vp.vector(1, 1, 1), color=vp.color.white)
vp.distant_light(direction=vp.vector(0, -1, 0), color=vp.vector(0.2, 0.4, 1.0))  # синий свет снизу

orbit_radius = model.RADIUS_OF_EARTH + model.ISS_ALTITUDE
theta = np.linspace(0, 2 * np.pi, 100)
orbit_points = []

for angle in theta:
    x = orbit_radius * np.cos(angle)
    z = orbit_radius * np.sin(angle)
    orbit_points.append(vp.vector(x, 0, z))

iss = vp.box(
    pos=vp.vector(orbit_radius, 0, 0),
    size=vp.vector(100000, 100000, 100000),
    color=vp.color.purple,
    make_trail=True,
    trail_type="points",
    trail_color=vp.vector(0.5, 0.8, 1.0),
    trail_radius=0.05,
    interval=10,
    retain=20
)

angular_velocity = 2 * np.pi / model.ISS_PERIOD * model.SIMULATION_SPEED
distance = 10
scene.camera.pos = vp.vector(0, distance, distance)
scene.center = rocket.pos

rocket_angle = 20
rocket.rotate(angle=-np.radians(rocket_angle), axis=vp.vector(0, 1, 0))

world_model = model.World()

while True:
    world_model.ComputeWorld(t)
    vp.rate(100)

    # Обновляем время
    t += dt

    # Обновляем систему выхлопа
    rocket_exhaust.update(dt)

    # Обновляем позицию МКС
    angle = angular_velocity * t
    x = orbit_radius * np.cos(angle)
    z = orbit_radius * np.sin(angle)

    scene.center = rocket.pos

    # Обновляем позицию ракеты
    rocket.pos = vp.vector(world_model.rocket.center_radius.GetX(),
                           world_model.rocket.center_radius.GetY(),
                           world_model.rocket.center_radius.GetZ())

    # Обновляем ориентацию ракеты по направлению скорости
    if world_model.rocket.velocity.GetModulus() > 0:
        rocket.up = vp.vector(world_model.rocket.velocity.GetX(),
                              world_model.rocket.velocity.GetY(),
                              world_model.rocket.velocity.GetZ())
