import vpython as vp
import numpy as np
from pygments.lexer import words
from vpython import scene
import model

vp.scene.title = "Modeling the motion of planets with the gravitational force"
vp.scene.height = 800
vp.scene.width = 1200

def gravitationalForce(p1,p2):
    G = 1 #real-world value is : G = 6.67e-11
    rVector = p1.pos - p2.pos
    rMagnitude = vp.mag(rVector)
    rHat = rVector / rMagnitude
    F = - rHat * G * p1.mass * p2.mass /rMagnitude**2
    return F

t = 0
dt = model.DELTA_TIME #The step size. This should be a small number

earth = vp.sphere(pos=vp.vector(0, 0, 0), radius=model.RADIUS_OF_EARTH, color=vp.color.green,
                 mass=1000, momentum=vp.vector(0, 0, 0))
rocket = vp.box(
    pos=vp.vector(0, model.RADIUS_OF_EARTH, 0),
    size=vp.vector(20000, 170000, 20000),
    axis=vp.vector(1, 1, 1),
    radius=0.01,
    color=vp.color.red,
    make_trail=True)



# Добавляем освещение
vp.distant_light(direction=vp.vector(1, 1, 1), color=vp.color.white)

orbit_radius = model.RADIUS_OF_EARTH + model.ISS_ALTITUDE
theta = np.linspace(0, 2*np.pi, 100)
orbit_points = []

for angle in theta:
    x = orbit_radius * np.cos(angle)
    z = orbit_radius * np.sin(angle)
    orbit_points.append(vp.vector(x, 0, z))


iss = vp.box(
    pos=vp.vector(orbit_radius, 0, 0),
    size=vp.vector(100000, 100000, 100000),  # Размеры МКС (условно)
    color=vp.color.purple,
    make_trail=True,           # Включаем след
    trail_type="points",
    trail_radius=0.05,
    interval=10,               # Рисовать точку следа каждые 10 шагов
    retain=20                 # Хранить только 100 последних точек
)

angular_velocity = 2 * np.pi / model.ISS_PERIOD * model.SIMULATION_SPEED

distance = 10 # Дистанция слежения за шаром
scene.camera.pos = vp.vector(0, distance, distance) # Камера сверху и сбоку
scene.center = rocket.pos # Центр сцены совпадает с шаром

rocket_angle = 20
rocket.rotate(angle=-np.radians(rocket_angle), axis=vp.vector(0, 1, 0))

world_model = model.World()
while True:
    world_model.ComputeWorld()

    vp.rate(100)  # Ограничение частоты кадров

    # Обновляем время
    t += dt

    # Вычисляем новую позицию МКС на орбите
    angle = angular_velocity * t
    x = orbit_radius * np.cos(angle)
    z = orbit_radius * np.sin(angle)

    #rocket.pos = vp.vector(0.1 * t * np.cos(rocket_angle), 0, 0.1 * t + Earth.RADIUS)
    #rocket.rotate(angle=np.radians(dt), axis=vp.vector(0, 1, 0))
    #rocket.up = vp.vector(0, 1, 0)
    #rocket_angle += dt
    scene.center = rocket.pos  # Центр сцены совпадает с ракетой

    # Обновляем позицию МКС
    print(world_model.rocket.center_radius.GetX(), world_model.rocket.center_radius.GetY(), world_model.rocket.center_radius.GetZ())
    rocket.pos = vp.vector(world_model.rocket.center_radius.GetX(), world_model.rocket.center_radius.GetY(), world_model.rocket.center_radius.GetZ())
    rocket.up = vp.vector(world_model.rocket.velocity.GetX() / world_model.rocket.velocity.modulus, world_model.rocket.velocity.GetY() / world_model.rocket.velocity.modulus, world_model.rocket.velocity.GetZ() / world_model.rocket.velocity.modulus)