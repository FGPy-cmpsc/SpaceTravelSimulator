import vpython as vp
import numpy as np
from pygments.lexer import words
from vpython import scene
import model
import random

vp.scene.title = ""
vp.scene.height = 800
vp.scene.width = 1200


def gravitationalForce(p1, p2):
    G = 1  # real-world value is : G = 6.67e-11
    rVector = p1.pos - p2.pos
    rMagnitude = vp.mag(rVector)
    rHat = rVector / rMagnitude
    F = - rHat * G * p1.mass * p2.mass / rMagnitude ** 2
    return F

t = 0
dt = model.DELTA_TIME
earth = vp.sphere(pos=vp.vector(0, 0, 0), radius=model.RADIUS_OF_EARTH, color=vp.color.green,
                  momentum=vp.vector(0, 0, 0))

orbit = vp.sphere(pos=vp.vector(0, 0, 0), radius=model.RADIUS_OF_EARTH + model.ISS_ALTITUDE, color=vp.color.blue, opacity = 0.2,
                  momentum=vp.vector(0, 0, 0))

rocket = vp.box(
    pos=vp.vector(0, model.RADIUS_OF_EARTH, 0),
    size=vp.vector(model.Rocket.ROCKET_SIZE_DIM_X, model.Rocket.ROCKET_SIZE_DIM_Y, model.Rocket.ROCKET_SIZE_DIM_Z),
    axis=vp.vector(1, 1, 1),
    radius=0.0001,
    color=vp.color.red,
    make_trail=True,
    trail_color=vp.color.cyan,
)

arrow = vp.arrow(
    color = vp.color.red,
    size=vp.vector(2*model.RADIUS_OF_EARTH, 0, 0), shaftwidth = 10)

arrow2 = vp.arrow(
    color = vp.color.red,
    size=vp.vector(2*model.RADIUS_OF_EARTH, 0, 0), shaftwidth = 30)

arrow3 = vp.arrow(
    color = vp.color.red,
    size=vp.vector(2*model.RADIUS_OF_EARTH, 0, 0), shaftwidth = 30)


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
    arrow.axis = vp.vector(world_model.rocket.reactive_force.GetX() / 10000, world_model.rocket.reactive_force.GetY() / 10000, world_model.rocket.reactive_force.GetZ())
    arrow.pos = vp.vector(world_model.rocket.center_radius.GetX(), world_model.rocket.center_radius.GetY(), world_model.rocket.center_radius.GetZ())

    vp.scene.title = str("Реактивная сила: " + str(int(world_model.rocket.reactive_force.GetModulus())) + " H. Скорость ракеты: " + str(int(world_model.rocket.velocity.GetModulus())) + " м/c." + " Высота над поверхностью земли: " + str(int(world_model.rocket.center_radius.GetModulus() - model.RADIUS_OF_EARTH)) + " м")

    """arrow2.axis = vp.vector(world_model.rocket.future_position_delta.GetX(), world_model.rocket.future_position_delta.GetY(), world_model.rocket.future_position_delta.GetZ())
    arrow2.pos = vp.vector(world_model.rocket.future_position_center.GetX(), world_model.rocket.future_position_center.GetY(),
                          world_model.rocket.future_position_center.GetZ())

    arrow3.axis = vp.vector(world_model.rocket.delta.GetX(),
                            world_model.rocket.delta.GetY(),
                            world_model.rocket.delta.GetZ())
    arrow3.pos = vp.vector(world_model.rocket.future_position_center.GetX(), world_model.rocket.future_position_center.GetY(),
                          world_model.rocket.future_position_center.GetZ())"""