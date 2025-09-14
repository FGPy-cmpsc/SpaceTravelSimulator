import math
from doctest import master

MASS_OF_EARTH = 5.97e24
RADIUS_OF_EARTH = 6_378_000
ISS_ALTITUDE = 408
ISS_PERIOD = 92.5  # минут (период обращения МКС)
SIMULATION_SPEED = 50
DELTA_TIME = 0.01
GRAVITATIONAL_CONSTANT = 6.6743 * 10**-11

class Vector:
    def __init__(self, x = 0.0, y = 0.0, z = 0.0):
        self.x_cord = x
        self.y_cord = y
        self.z_cord = z

    def GetX(self):
        return self.x_cord

    def GetY(self):
        return self.y_cord

    def GetZ(self):
        return self.z_cord

    def AddVector(self, vector):
        self.x_cord += vector.x_cord
        self.y_cord += vector.y_cord
        self.z_cord += vector.z_cord

    def GetModulus(self):
        return math.sqrt(self.x_cord ** 2 + self.y_cord ** 2 + self.z_cord ** 2)

    def SetDirection(self, direction_vector):
        modulus = self.GetModulus()
        direction_modulus = direction_vector.GetModulus()
        self.x_cord = direction_vector.x_cord * modulus / direction_modulus
        self.y_cord = direction_vector.y_cord * modulus / direction_modulus
        self.z_cord = direction_vector.z_cord * modulus / direction_modulus

    def SetModulus(self, modulus):
        old_modulus = self.GetModulus()
        self.x_cord *= modulus / old_modulus
        self.y_cord *= modulus / old_modulus
        self.z_cord *= modulus / old_modulus

    def CopyVector(self):
        return Vector(self.x_cord, self.y_cord, self.z_cord)

    def __mul__(self, scalar):
        return Vector(self.x_cord * scalar, self.y_cord * scalar, self.z_cord * scalar)

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

class ExternalForce:
    def __init__(self):
        self.ForceVector = Vector()

    def ComputeExternalForce(self, mass_of_rocket, rocket_center_radius):
        new_force_module = GRAVITATIONAL_CONSTANT * MASS_OF_EARTH * mass_of_rocket / rocket_center_radius.GetModulus()**2
        print(new_force_module)

        self.ForceVector = rocket_center_radius.CopyVector()
        self.ForceVector.SetModulus(new_force_module)
        self.ForceVector *= -1

    def GetExternalForceVector(self):
        return self.ForceVector


class ISS:
    def __init__(self):
        self.center_radius = Vector(0, RADIUS_OF_EARTH + ISS_ALTITUDE, 0)

class Rocket:
    ROCKET_SIZE_DIM_X = 30
    ROCKET_SIZE_DIM_Y = 300
    ROCKET_SIZE_DIM_Z = 30
    MIN_ROCKET_WEIGHT = 10

    TARGET = Vector(2 * RADIUS_OF_EARTH, 2 * RADIUS_OF_EARTH, 0)
    def __init__(self):
        # basic characteristics
        self.mass = 300_000
        self.mass_fuel_consumption = 880.3
        self.gas_speed_relative_to_the_rocket = 40000

        # dynamics
        self.reactive_force = Vector(0, self.mass_fuel_consumption * self.gas_speed_relative_to_the_rocket, 0)

        # movement
        self.velocity = Vector()
        self.acceleration = Vector()

        self.center_radius = Vector(0, RADIUS_OF_EARTH, 0)

        x_angle = math.pi / 2
        y_angle = math.pi / 2
        z_angle = 0

    def ComputeReactiveForce(self, external_force):
        if self.mass < self.MIN_ROCKET_WEIGHT:
            self.reactive_force = Vector()


    def ComputeAcceleration(self, external_force):
        total_force = Vector()
        total_force.AddVector(self.reactive_force)
        total_force.AddVector(external_force)

        self.acceleration.x_cord = total_force.x_cord / self.mass
        self.acceleration.y_cord = total_force.y_cord / self.mass
        self.acceleration.z_cord = total_force.z_cord / self.mass

        print(self.acceleration.GetModulus())

    def ComputeVelocity(self):
        delta_velocity = self.acceleration * DELTA_TIME
        self.velocity.AddVector(delta_velocity)

    def ComputeCoordinates(self):
        delta_radius = self.velocity * DELTA_TIME
        self.center_radius.AddVector(delta_radius)

    def ComputeRocket(self, external_force, overall_time):
        self.ComputeReactiveForce(external_force)
        self.ComputeAcceleration(external_force)
        self.ComputeVelocity()
        self.ComputeCoordinates()
        if self.mass > self.MIN_ROCKET_WEIGHT:
            self.mass -= self.mass_fuel_consumption * DELTA_TIME

class World:
    def __init__(self):
        self.rocket = Rocket()
        self.external_force = ExternalForce()

        self.rocket.center_radius = Vector(0, RADIUS_OF_EARTH, 0)

    def ComputeWorld(self, overall_time):
        self.external_force.ComputeExternalForce(self.rocket.mass, self.rocket.center_radius)
        self.rocket.ComputeRocket(self.external_force.GetExternalForceVector(), overall_time)
