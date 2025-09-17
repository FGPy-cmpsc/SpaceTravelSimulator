import math
import random
from doctest import master
from random import randint

MASS_OF_EARTH = 5.97e24
RADIUS_OF_EARTH = 6_378_000
ISS_ALTITUDE = 10_000
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
        self.x_cord *= (modulus / old_modulus)
        self.y_cord *= (modulus / old_modulus)
        self.z_cord *= (modulus / old_modulus)

    def CopyVector(self):
        return Vector(self.x_cord, self.y_cord, self.z_cord)

    def ScalarProduct(self, vector):
        return self.x_cord * vector.x_cord + self.y_cord * vector.y_cord + self.z_cord * vector.z_cord

    def __mul__(self, scalar):
        return Vector(self.x_cord * scalar, self.y_cord * scalar, self.z_cord * scalar)

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

class ExternalForce:
    def __init__(self):
        self.ForceVector = Vector()

    def ComputeExternalForce(self, mass_of_rocket, rocket_center_radius):
        new_force_module = GRAVITATIONAL_CONSTANT * MASS_OF_EARTH * mass_of_rocket / rocket_center_radius.GetModulus()**2
        #print("f = ", new_force_module)

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
    MIN_ROCKET_WEIGHT = 150


    def __init__(self):
        # basic characteristics
        self.mass = 300_000
        self.mass_fuel_consumption = 300
        self.gas_speed_relative_to_the_rocket = 16000

        # dynamics
        self.reactive_force = Vector(0, self.mass_fuel_consumption * self.gas_speed_relative_to_the_rocket, 0)

        # movement
        self.velocity = Vector()
        self.acceleration = Vector()

        self.center_radius = Vector(0, RADIUS_OF_EARTH, 0)

        self.target = Vector(0, RADIUS_OF_EARTH + ISS_ALTITUDE, 0)
        self.orbital_altitude = self.target.GetModulus() - RADIUS_OF_EARTH

        self.reactive_force_angle = 0
        self.delta_target_altitude = abs(self.target.GetModulus() - self.center_radius.GetModulus())
        self.future_position_center = self.center_radius.CopyVector()
        self.future_position_delta = Vector(1000, 0, 0)
        self.future_position = Vector(1000, 0, 0)
        self.delta = Vector(1000, 0, 0)
        self.velocity_per = Vector(1000, 0, 0)

        x_angle = math.pi / 2
        y_angle = math.pi / 2
        z_angle = 0

    def ComputeNormalForce(self, external_force):
        if self.center_radius.ScalarProduct(self.velocity) > 50:
            self.reactive_force.SetDirection(Vector(self.velocity.GetY(), -self.velocity.GetX(), self.velocity.GetZ()))
            self.reactive_force.SetModulus(self.mass_fuel_consumption * self.gas_speed_relative_to_the_rocket)
        elif self.center_radius.ScalarProduct(self.velocity) < -50:
            self.reactive_force.SetDirection(Vector(-self.velocity.GetY(), self.velocity.GetX(), self.velocity.GetZ()))
            self.reactive_force.SetModulus(self.mass_fuel_consumption * self.gas_speed_relative_to_the_rocket)
        else:
            self.reactive_force.SetDirection(external_force)
            self.reactive_force.SetModulus(self.mass * (self.velocity.GetModulus() ** 2) / self.center_radius.GetModulus() - external_force.GetModulus())


    def ComputeReactiveForce(self, external_force):
        if self.mass < self.MIN_ROCKET_WEIGHT:
            self.reactive_force = Vector()
        else:
            if abs(self.center_radius.GetModulus() - self.target.GetModulus()) < 30:
                self.ComputeNormalForce(external_force)
            else:
                print("velocity = ", self.velocity.GetModulus())
                if self.velocity.GetModulus() < 30:
                    return

                self.delta_target_altitude = self.target.GetModulus() - self.center_radius.GetModulus()
                print(self.delta_target_altitude)
                center_radius_of_trajectory = self.center_radius.CopyVector()
                delta_radius = Vector(self.velocity.GetY(), -self.velocity.GetX(), self.velocity.GetZ())
                self.velocity_per = delta_radius.CopyVector()
                #self.delta = delta_radius.CopyVector()
                #self.delta *= 10
                suppose_angle_left = math.acos(
                    self.velocity.ScalarProduct(Vector(0, 1, 0)) / self.velocity.GetModulus())
                suppose_angle_right = math.acos(
                    self.velocity.ScalarProduct(Vector(0, 1, 0)) / self.velocity.GetModulus()) + math.pi / 2

                suppose_reactive_force = Vector()
                while suppose_angle_right - suppose_angle_left > 0.0001:
                    suppose_angle = (suppose_angle_right + suppose_angle_left) / 2
                    suppose_force = self.reactive_force.CopyVector()
                    suppose_force.SetDirection(Vector(math.sin(suppose_angle), math.cos(suppose_angle), 0))
                    suppose_reactive_force = suppose_force.CopyVector()
                    suppose_force.AddVector(external_force)
                    suppose_force_projection = abs(suppose_force.ScalarProduct(delta_radius)) / delta_radius.GetModulus()
                    suppose_acceleration = suppose_force_projection / self.mass
                    if suppose_acceleration == 0:
                        return
                    delta_radius.SetModulus((self.velocity.GetModulus() ** 2) / suppose_acceleration)
                    delta_radius.SetDirection(Vector(self.velocity.GetY(), -self.velocity.GetX(), self.velocity.GetZ()))
                    center_radius_of_trajectory.AddVector(delta_radius)
                    self.delta = delta_radius.CopyVector()
                    delta_radius.SetDirection(center_radius_of_trajectory)
                    suppose_position = center_radius_of_trajectory.CopyVector()
                    suppose_position.AddVector(delta_radius)
                    self.future_position_center = center_radius_of_trajectory.CopyVector()
                    self.future_position_delta = delta_radius.CopyVector()
                    delta_altitude = suppose_position.GetModulus() - self.target.GetModulus()
                    if delta_altitude > 0:
                        suppose_angle_left = suppose_angle
                    else:
                        suppose_angle_right = suppose_angle
                self.reactive_force = suppose_reactive_force.CopyVector()
                #print("delta = ", self.delta.GetX())

                #suppose_angle = random.uniform(self.reactive_force_angle - math.pi / 6, self.reactive_force_angle + math.pi / 6)
                #suppose_force = self.reactive_force.CopyVector()
                #suppose_force.SetDirection(Vector(math.sin(suppose_angle), math.cos(suppose_angle), 0))
                #suppose_reactive_force = suppose_force.CopyVector()
                #suppose_force.AddVector(external_force)
                #suppose_force_projection = abs(suppose_force.ScalarProduct(delta_radius)) / delta_radius.GetModulus()
                #suppose_acceleration = suppose_force_projection / self.mass

                #delta_radius.SetModulus((self.velocity.GetModulus() ** 2) / suppose_acceleration)
                #center_radius_of_trajectory.AddVector(delta_radius)
                #delta_radius.SetDirection(center_radius_of_trajectory)
                #suppose_position = center_radius_of_trajectory.CopyVector()
                #suppose_position.AddVector(delta_radius)
                #delta_altitude = abs(suppose_position.GetModulus() - self.target.GetModulus())

                #if delta_altitude < self.delta_target_altitude:
                #    self.reactive_force = suppose_reactive_force.CopyVector()
                #    self.delta_target_altitude = delta_altitude
                #    self.reactive_force_angle = suppose_angle
                """else:
                    p = math.exp((self.delta_target_altitude - delta_altitude) / abs(self.target.GetModulus() - self.center_radius.GetModulus()))
                    N = 1_000_000
                    n = randint(1, N)
                    if n < p * N:
                        self.reactive_force = suppose_reactive_force.CopyVector()
                        self.delta_target_altitude = delta_altitude
                        self.reactive_force_angle = suppose_angle"""


    def ComputeAcceleration(self, external_force):
        total_force = Vector()
        total_force.AddVector(self.reactive_force)
        total_force.AddVector(external_force)

        self.acceleration.x_cord = total_force.x_cord / self.mass
        self.acceleration.y_cord = total_force.y_cord / self.mass
        self.acceleration.z_cord = total_force.z_cord / self.mass

        #print("a = ", self.acceleration.GetX(), self.acceleration.GetY())

    def ComputeVelocity(self):
        delta_velocity = self.acceleration * DELTA_TIME
        self.velocity.AddVector(delta_velocity)
        #print("v = ", self.velocity.GetX(), self.velocity.GetY())

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
        #print("mass = ", self.mass)

class World:
    def __init__(self):
        self.rocket = Rocket()
        self.external_force = ExternalForce()

        self.rocket.center_radius = Vector(0, RADIUS_OF_EARTH, 0)

    def ComputeWorld(self, overall_time):
        self.external_force.ComputeExternalForce(self.rocket.mass, self.rocket.center_radius)
        self.rocket.ComputeRocket(self.external_force.GetExternalForceVector(), overall_time)
