import math
from doctest import master

MASS_OF_EARTH = 5.97e24
RADIUS_OF_EARTH = 6_378_000
ISS_ALTITUDE = 408
ISS_PERIOD = 92.5  # минут (период обращения МКС)
SIMULATION_SPEED = 50
DELTA_TIME = 0.01

class Vector:
    def __init__(self, modulus = 0.0, x_angle = math.pi / 2, y_angle = 0, z_angle = math.pi / 2):
        self.modulus = modulus
        self.x_angle = x_angle
        self.y_angle = y_angle
        self.z_angle = z_angle

    def AddVector(self, vector):
        x_new = self.modulus * math.cos(self.x_angle) + vector.modulus * math.cos(vector.x_angle)
        y_new = self.modulus * math.cos(self.y_angle) + vector.modulus * math.cos(vector.y_angle)
        z_new = self.modulus * math.cos(self.z_angle) + vector.modulus * math.cos(vector.z_angle)

        self.modulus = math.sqrt(x_new * x_new + y_new * y_new + z_new * z_new)
        self.x_angle = math.acos(x_new / self.modulus)
        self.y_angle = math.acos(y_new / self.modulus)
        self.z_angle = math.acos(z_new / self.modulus)

    def GetX(self):
        return self.modulus * math.cos(self.x_angle)

    def GetY(self):
        return self.modulus * math.cos(self.y_angle)

    def GetZ(self):
        return self.modulus * math.cos(self.z_angle)

    def __mul__(self, scalar):
        return Vector(self.modulus * scalar, self.x_angle, self.y_angle, self.z_angle)

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

class ISS:
    def __init__(self):
        self.center_radius = Vector(RADIUS_OF_EARTH + ISS_ALTITUDE)

class Rocket:
    ROCKET_SIZE_DIM_X = 30
    ROCKET_SIZE_DIM_Y = 300
    ROCKET_SIZE_DIM_Z = 30
    MIN_ROCKET_WEIGHT = 10
    def __init__(self):
        # basic characteristics
        self.mass = 300_000
        self.mass_fuel_consumption = 880.3
        self.gas_speed_relative_to_the_rocket = 40000

        # dynamics
        self.reactive_force = Vector(self.mass_fuel_consumption * self.gas_speed_relative_to_the_rocket)

        # movement
        self.velocity = Vector()
        self.acceleration = Vector()

        self.center_radius = Vector(RADIUS_OF_EARTH)

        x_angle = math.pi / 2
        y_angle = math.pi / 2
        z_angle = 0

    def ComputeReactiveForce(self):
        if self.mass < self.MIN_ROCKET_WEIGHT:
            self.reactive_force = Vector()

    def ComputeAcceleration(self, external_force):
        self.ComputeReactiveForce()

        total_force = Vector()
        total_force.AddVector(self.reactive_force)
        total_force.AddVector(external_force)

        self.acceleration.modulus = total_force.modulus / self.mass
        self.acceleration.x_angle = total_force.x_angle
        self.acceleration.y_angle = total_force.y_angle
        self.acceleration.z_angle = total_force.z_angle
        print(self.acceleration.modulus)

    def ComputeVelocity(self):
        delta_velocity = self.acceleration * DELTA_TIME
        self.velocity.AddVector(delta_velocity)

    def ComputeCoordinates(self):
        delta_radius = self.velocity * DELTA_TIME
        self.center_radius.AddVector(delta_radius)

    def ComputeRocket(self, external_force):
        self.ComputeAcceleration(external_force)
        self.ComputeVelocity()
        self.ComputeCoordinates()
        if self.mass > self.MIN_ROCKET_WEIGHT:
            self.mass -= self.mass_fuel_consumption * DELTA_TIME

class World:
    def __init__(self):
        self.rocket = Rocket()

        self.rocket.center_radius = Vector(RADIUS_OF_EARTH)

    def ComputeWorld(self):
        self.rocket.ComputeRocket(Vector())
