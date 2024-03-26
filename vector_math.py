"""CS 108 Final Project

This module implements a Vector2 class, with which complex math
can be done with ease and concisity.

Supported features include:

Adding and subtracting vectors

Multiplying and divinging a vector by either a float, int, or another vector

Comparing two vectors with ==

Normalizing the vector

Performing a dot product

Flipping the vector

Drawing the vector on a specified canvas (guizero)

Rotating the vector by an angle

@author: Caleb Shippy (cfs4)
@date: Fall, 2021
"""

import math

class Vector2:

    def __init__(self, x = 0, y = 0):
        """Constructor to initialize the creation of a Vector2. Setting up attributes such as x, y, and magnitude."""

        self.x = x
        self.y = y
        self.magnitude = self.calculate_magnitude()


    def calculate_magnitude(self):
        """Uses pythagoreans theorem to calculate the magnitude of the vector based on its given components."""

        return math.sqrt((self.x * self.x) + (self.y * self.y))

    def get_angle(self, cw = True, mode = 'DEGREE'):
        """Returns the angle from the x-axis"""

        if self.x == 0:
            if self.y > 0:
                a = math.pi/2
            else:
                a = 3 * math.pi/2
        else:
            a = math.atan(self.y/self.x)

        if cw:
            if mode == "DEGREE":
                a = (a / math.pi * 180)
        else:
            if mode == "RADIAN":
                a = math.pi - a
            elif mode == "DEGREE":
                a = 180 - (a / math.pi * 180)

        return a



    def __mul__(self, other):
        """Multiplies the components of each vector together and returns the new vector.
        Unless the operand is a float or an int, then it multiplies each component by that number
        """

        if isinstance(other, float) or isinstance(other, int):
            return Vector2(self.x * other, self.y * other)
        else:
            return Vector2(self.x * other.x, self.y * other.y)


    def __div__(self, other):
        """Divides the components of each vector and returns the new vector.
        Unless the operand is a float or an int, then it divides each component by that number
        """

        if isinstance(other, float) or isinstance(other, int):
            return Vector2(self.x / other, self.y / other)
        else:
            return Vector2(self.x / other.x, self.y / other.y)


    def __add__(self, other):
        """Adds the components of each vector together and returns the new vector."""

        return Vector2(self.x + other.x, self.y + other.y)


    def __sub__(self, other):
        """Subtracts the components of the other vector from its own components and returns the new vector."""

        return Vector2(self.x - other.x, self.y - other.y)

    def __eq__(self, other):
        """Compares the components of the two vectors to determine whether or not they are eual"""

        if self.x == other.x and self.y == other.y:
            return True
        else:
            return False

    def __str__(self):
        """Pretty print constructor, which formats what to print when the class is printed."""

        return  (
            "x component: " + str(self.x) +
            "\n" + "y component: " + str(self.y) +
            "\n" + "magnitude: " + str(self.magnitude)
        )

    def dot_product(self, other):
        """Finds the dot product of the vector and a specified second vector."""

        return (self.x * other.x) + (self.y * other.y)

    def normalize(self):
        """Normalizes the vector so that the magnitude is 1."""

        if not self.magnitude == 0: # Checks to make sure the vector is not a zero vector
            return Vector2(self.x / self.magnitude, self.y / self.magnitude)
        else:
            return Vector2()

    def draw_vector(self, dwg, x=0, y=0, mult=1, color = 'blue', w=1):
        """ Draws the vector on the specified canvas at the specified position."""

        dwg.line(x, y, x + self.x * mult, y + self.y * mult, color=color, width=w)

    def flip(self):
        """ Reverses the direction of the vector (equivalent to multipliying by -1)"""

        return self * -1

    def rotate(self, angle, mode='DEGREE'):
        """ Returns a vector with the same magnitude rotated clockwise by the specified angle"""

        if mode == 'DEGREE':

            a = angle / 180 * math.pi

        elif mode == 'RADIAN':

            a = angle

        vec = self.normalize()

        x_prime = vec.x * math.cos(a) - vec.y * math.sin(a)
        y_prime = vec.x * math.sin(a) + vec.y * math.cos(a)

        return Vector2(x_prime, y_prime).normalize() * self.magnitude


    def move_to(self, other, delta=1):

        if self.x == 0:
            a1 = math.pi / 2
        else:
            a1 = math.atan(self.y / self.x)

        if other.y == 0:
            a1 = math.pi / 2
        else:
            a2 = math.atan(other.y / other.x)

        if a1 > 2 * math.pi:
            a1 = a1 % 2 * math.pi

        if a2 > 2 * math.pi:
            a2 = a2 % 2 * math.pi

        if a1 < 0:
            a1 = 2 * math.pi - a1

        if a2 < 0:
            a2 = 2 * math.pi - a2

        if a2 > a1:

            return self.rotate(-delta,'RADIAN')
        else:
            return self.rotate(delta,'RADIAN')


class Vector3:

    def __init__(self, x = 0, y = 0, z = 0):
        """Constructor to initialize the creation of a Vector3. Setting up attributes such as x, y, z, and magnitude."""

        self.x = x
        self.y = y
        self.z = z
        self.magnitude = self.calculate_magnitude()


    def calculate_magnitude(self):
        """Uses pythagoreans theorem to calculate the magnitude of the vector based on its given components."""

        return math.sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z))


    def __mul__(self, other):
        """Multiplies the components of each vector together and returns the new vector.
        Unless the operand is a float or an int, then it multiplies each component by that number
        """

        if isinstance(other, float) or isinstance(other, int):
            return Vector3(self.x * other, self.y * other, self.z * other)
        else:
            return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)


    def __truediv__(self, other):
        """Divides the components of each vector and returns the new vector.
        Unless the operand is a float or an int, then it divides each component by that number
        """

        if isinstance(other, float) or isinstance(other, int):
            return Vector3(self.x / other, self.y / other, self.y / other)
        else:
            return Vector3(self.x / other.x, self.y / other.y, self.z / other.z)


    def __add__(self, other):
        """Adds the components of each vector together and returns the new vector."""

        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)


    def __sub__(self, other):
        """Subtracts the components of the other vector from its own components and returns the new vector."""

        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __eq__(self, other):
        """Compares the components of the two vectors to determine whether or not they are eual"""

        if self.x == other.x and self.y == other.y and self.z == other.z:
            return True
        else:
            return False

    def __str__(self):
        """Pretty print constructor, which formats what to print when the class is printed."""

        return  (
            "x component: " + str(self.x) +
            "\n" + "y component: " + str(self.y) +
            "\n" + "z component: " + str(self.z) +
            "\n" + "magnitude: " + str(self.magnitude)
        )

    def normalize(self):
        """Normalizes the vector so that the magnitude is 1."""

        if not self.magnitude == 0: # Checks to make sure the vector is not a zero vector
            return Vector3(self.x / self.magnitude, self.y / self.magnitude, self.z / self.magnitude)
        else:
            return Vector3()

    def flip(self):
        """ Reverses the direction of the vector (equivalent to multipliying by -1)"""

        return self * -1
