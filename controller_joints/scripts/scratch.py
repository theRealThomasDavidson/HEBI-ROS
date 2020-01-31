import numpy as np
import random
import time
#import sympy as sym


class Jacobian:
    def __init__(self, location3, angle, orientation):
        """
        this will describe a jacobian that slides in 3space and has a twist a it's end effector
        :param location3: is a tuple of 3 floats that describe a slide in 3 spance with the first being the magnatude  and
        direction (MnD) in x, the second is the MnD in y, and the third is the MnD in z.
        :param angle: this will be an angle descibed as a float of in terms of radians
        :param orientation: will take a string of "x", "y", or "z" to decribe the orientation of the angle
        :return: a np array of a 4x4 jacobian of the transform
        """
        self.matrix = transform(location3, angle, orientation)

    def __mul__(self, other):
        if isinstance(other, Jacobian):
            a = Jacobian((0, 0, 0), 0, "x")
            a.matrix = np.dot(self.matrix, other.matrix)
            return a
        if isinstance(other, np.ndarray):
            a = Jacobian((0, 0, 0), 0, "x")
            a.matrix = np.dot(self.matrix, other)
            return a
        else:
            raise TypeError("multiplying Jacobian by incompatible type, compatiable types are numpy nd arrays or Jacobians")

    def __repr__(self):
        return str(self.matrix)

    def getlocation3(self):
        """
        this returns the distance of the transform in 3 space. with the x dimension being the first element, y 2nd, z third
        :return: a numpy 1d array of size 3 that has the distance of the transform in
        """
        return self.matrix[0:3, 3]

    def getQuaternion(self):
        """
        this returns the quaternion of the end effector of the transform
        :return: a 1d numpy array of size 4 that is the quaternion of the end effector in w,x,y,z form
        """
        return M2Q(self.matrix[0:3, 0:3])


def newtonArgs():
    """
    this will output a function matrix and jacobian matrix
    :return: a function matrix and jacobian matrix as lambda
    """
    F = lambda x: np.array ([
    0.341849382038347 * np.cos(x[0] - 0.315275159242778) + 0.325 * np.cos(x[0] - x[1]),
    -0.341849382038347 * np.sin(x[0] - 0.315275159242778) - 0.325 * np.sin(x[0] - x[1])
    ])
    #F = lambda x: np.array([
    #    0.325*np.cos(x[1] - x[0]) + 0.325280263772643*np.cos(x[0] - 0.0415145954811903),
    #    0.325*np.sin(x[1] - x[0]) - 0.325280263772643*np.sin(x[0] - 0.0415145954811903)
    #])
    #J = lambda x: np.array([[
    #    0.325 * np.sin(x[1] - x[0]) - 0.325280263772643 * np.sin(x[0] - 0.0415145954811903),
    #    -0.325 * np.sin(x[1] - x[0])
    #    ], [
    #    -0.325 * np.cos(x[1] - x[0]) - 0.325280263772643 * np.cos(x[0] - 0.0415145954811903),
    #    0.325 * np.cos(x[1] - x[0])
    #]])
    J = lambda x: np.array([[
        -0.341849382038347* np.sin(x[0] - 0.315275159242778) - 0.325* np.sin(x[0] - x[1]),
        0.325*np.sin(x[0] - x[1])
    ],[
        -0.341849382038347 * np.cos(x[0] - 0.315275159242778) - 0.325 * np.cos(x[0] - x[1]),
        0.325 * np.cos(x[0] - x[1])
    ]])

    Elbow = lambda x: np.array([
        0.325280263772643 * np.cos(x[0] - 0.0415145954811903),
        -0.325280263772643 * np.sin(x[0] - 0.0415145954811903)
    ])

    return F, J, Elbow

def newtonMethod(estimate, functionMatrix, errorTolerence, derivativeMatrix, desiredOutput):
    """
    :param estimate: a tuple of the form (float, ...) of size equal to the number of input variables into your function
    (needs to be on the same side inflections in the function)
    :param functionMatrix: this is a tuple of the form (lambda,...) of the same size as the number of output variables
    your system of equations solves for. these lambda functions will have an input in the form of a tuple (float,...)
    with the langth of the tuple is the number of input variables in the order presented in the estimate tuple
    number of arguments is the number of input variables for the function
    :param iterations: this is just an incrementor since I am working with recursion right here and we can unwrap this
    later.
    :param errorTolerence: this is a float that will give the error tolerance for the approximation
    :param derivativeMatrix: this is a tuple of tuples of lambdas, the outer tuple will be of size equal to the output
    variables the inner tuples will be of a size equal to the number of output variables. each element of the inner
    tuple will be a lambda function that takes the estimate tuple as an argument and will represent the partial
    derivative of the output element associated with the outer tuple in respect to the input element associated with
    the location in your inner tuple.
    :return: a tuple of the same form as the estimate that will be within the error tolerence of an answer.
    """
    if .05 >= np.linalg.norm(desiredOutput, ord=2) >= np.linalg.norm(functionMatrix((0, 0)), ord=2):
        return estimate, "This isn't gonna work, fam."
    location = functionMatrix(estimate)
    locationErr = location - desiredOutput
    errorMag = np.linalg.norm(locationErr, ord=2)
    iterations = 0
    while errorMag > errorTolerence and iterations < 60:
        estimate = np.array([x % (np.pi * 2) for x in list(estimate)])
        location = functionMatrix(estimate)
        locationErr = location - desiredOutput
        errorMag = np.linalg.norm(locationErr, ord=2)
        delta = np.linalg.solve(derivativeMatrix(estimate), -locationErr)
        estimate += delta
        iterations += 1
    return estimate

def LinearSlide(currentState, functionMatrix, derivativeMatrix, desiredOutput):
    """
    This will take a current state and return a
    :param currentState:
    :param functionMatrix:
    :param derivativeMatrix:
    :param desiredOutput:
    :return:
    """
    location = functionMatrix(currentState)
    locationErr = location - desiredOutput


    estimate = np.array([x % (np.pi * 2) for x in list(currentState)])
    location = functionMatrix(estimate)
    locationErr = location - desiredOutput
    delta = np.linalg.solve(derivativeMatrix(estimate), -locationErr)
    newState = currentState + delta

    return newState


def M2Q(rot):
    """
    # convert rotation matrix to normalized quaternion
    :param rot:
    :return:
    """
    m00 = rot[0, 0]
    m01 = rot[0, 1]
    m02 = rot[0, 2]
    m10 = rot[1, 0]
    m11 = rot[1, 1]
    m12 = rot[1, 2]
    m20 = rot[2, 0]
    m21 = rot[2, 1]
    m22 = rot[2, 2]

    tr = m00 + m11 + m22

    if tr > 0:
        S = np.sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S

    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S

    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S

    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    return np.array([qw, qx, qy, qz])


def slide(location3):
    """
    this describes a slide in 3 space and gives an nparray for the jacobian of that slide
    :param location3: is a tuple of 3 floats that describe a slide in 3 spance with the first being the magnatude  and
    direction (MnD) in x, the second is the MnD in y, and the third is the MnD in z.
    :return: a np array of a 4x4 jacobian of the slide
    """
    return np.array([[1, 0, 0, location3[0]], [0, 1, 0, location3[1]], [0, 0, 1, location3[2]], [0, 0, 0, 1]])


def twist(angle, orientation):
    """
    this will describe a twist in x, y or z with a np array as the jacobian
    :param angle: this will be an angle descibed as a float of in terms of radians
    :param orientation: will take a string of "x", "y", or "z" to decribe the orientation of the angle
    :return: a np array of a 4x4 jacobian of the twist
    """
    cos = np.cos(angle)
    sin = np.sin(angle)
    if orientation == "x":
        return np.array([[1, 0, 0, 0], [0, cos, sin, 0], [0, -sin, cos, 0], [0, 0, 0, 1]])

    if orientation == "y":
        return np.array([[cos, 0, -sin, 0], [0, 1, 0, 0], [sin, 0, cos, 0], [0, 0, 0, 1]])

    if orientation == "z":
        return np.array([[cos, sin, 0, 0], [-sin, cos, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    return None


def transform(location3,angle, orientation):
    """
    this will describe a jacobian that slides in 3space and has a twist a it's end effector
    :param location3: is a tuple of 3 floats that describe a slide in 3 spance with the first being the magnatude  and
    direction (MnD) in x, the second is the MnD in y, and the third is the MnD in z.
    :param angle: this will be an angle descibed as a float of in terms of radians
    :param orientation: will take a string of "x", "y", or "z" to decribe the orientation of the angle
    :return: a np array of a 4x4 jacobian of the transform
    """
    return np.dot(slide(location3), twist(angle, orientation))


def main():
    baseAngle = 0
    elbowAngle = 0
    baseHeight = 0
    iden = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    swb = (0, 0, baseHeight)
    thetawb = 0, "z"    #rigid
    sbs = (0, 0, .031)
    thetabs = (baseAngle, "z")  #mobile
    ssI = (0, -.0375, .055)
    thetasI = np.pi/2, "x"  #rigid
    sIa = 0, 0, 0.031
    thetaIa = 0, "z"    #fixed
    ext = 0.325
    turn = np.pi/2
    etc = .02
    sae = ext, -np.sin(turn) * etc, etc * (1+np.cos(turn))
    thetaae = turn, "x" #r n igid
    sef = sbs[:]
    thetaef = elbowAngle, "z"     #mobile
    turn = 0
    sfp = ext, -np.sin(turn) * etc, etc * (1+np.cos(turn))
    thetafp = turn, "x" #rigid

    spe = 0, 0, .1
    thetape = 0, "x" #rigid


    origin = np.array([[0], [0], [0], [1]])

    twb = Jacobian(swb, thetawb[0], thetawb[1])
    tbs = Jacobian(sbs, thetabs[0], thetabs[1])
    tsI = Jacobian(ssI, thetasI[0], thetasI[1])
    tIa = Jacobian(sIa, thetaIa[0], thetaIa[1])
    tae = Jacobian(sae, thetaae[0], thetaae[1])
    tef = Jacobian(sef, thetaef[0], thetaef[1])
    tfp = Jacobian(sfp, thetafp[0], thetafp[1])
    tpe = Jacobian(spe, thetape[0], thetape[1])

    with np.printoptions(precision=8, suppress=True):
        tbe = tbs * tsI * tIa * tae

        print("\nbase to elbow")
        print("transform:\n", tbe)
        print("location3:", tbe.getlocation3())
        print("quaternion:", tbe.getQuaternion())

        tep = tef * tfp
        #print("\nelbow to paddle")
        #print("transform:\n", tep)
        #print("location3:", tep.getlocation3())
        #print("quaternion:", tep.getQuaternion())

        tbp1 = tbe * tep
        #print("\nbase to paddle 1")
        #print("location3:", tbp1.getlocation3())
        #print("quaternion:", tbp1.getQuaternion())

        tbp2 = tbs * tsI * tIa * tae * tef * tfp
        #print("\nbase to paddle 2")
        #print("location3:", tbp2.getlocation3())
        #print("quaternion:", tbp2.getQuaternion(), "\n\n")

    len1 = ((.106**2)+(.325**2))**(1/2)
    len2 = .325
    angleOffset1 = np.arctan2(.106, .325)
    estimate = .45, 1.2
    desired = .48, -.19

    """
    #code below this block referenceing F and J were generated with this script
    th, ph = sym.symbols("x[0] x[1]")
    Fx = (len1 * sym.cos(-th + angleOffset1)) + (len2 * sym.cos(-th + ph))
    Fy = (len1 * sym.sin(-th + angleOffset1)) + (len2 * sym.sin(-th + ph))
    print(ph, "\n", th, "\n", Fx, "\n", Fy)
    print(sym.diff(Fx, ph))
    
    for func in (Fx,Fy):
        for diff in th,ph:
            print("diff of", func, "by",diff, "is:\n", sym.diff(func, diff))
    """
    F = lambda x: np.array([
        0.325*np.cos(x[1] - x[0]) + 0.325280263772643*np.cos(x[0] - 0.0415145954811903),
        0.325*np.sin(x[1] - x[0]) - 0.325280263772643*np.sin(x[0] - 0.0415145954811903)
    ])
    J = lambda x: np.array([[
        0.325 * np.sin(x[1] - x[0]) - 0.325280263772643 * np.sin(x[0] - 0.0415145954811903),
        -0.325 * np.sin(x[1] - x[0])
        ], [
        -0.325 * np.cos(x[1] - x[0]) - 0.325280263772643 * np.cos(x[0] - 0.0415145954811903),
        0.325 * np.cos(x[1] - x[0])
    ]])

    #print(F((0, -np.pi/2)))
    tol = .001
    x, n = newtonMethod(np.array(estimate), F, 0, tol, J, np.array(desired))
    #print (n, x)
    error_norm = np.linalg.norm(estimate - x, ord=2)
    #print ('Anorm of error =%g' % error_norm)
    estimate = x
    desired = .38, -.21

    x, n = newtonMethod(np.array(estimate), F, 0, tol, J, np.array(desired))
    #print (n, x)
    error_norm = np.linalg.norm(estimate - x, ord=2)
    #print ('Bnorm of error =%g' % error_norm)
    estimate = x
    desired = .3, -.13

    x, n = newtonMethod(np.array(estimate), F, 0, tol, J, np.array(desired))
    #print (n, x)
    error_norm = np.linalg.norm(estimate - x, ord=2)
    #print ('Cnorm of error =%g' % error_norm)
    estimate = x
    desired = .21, -.05

    x, n = newtonMethod(np.array(estimate), F, 0, tol, J, np.array(desired))
    #print (n, x)
    error_norm = np.linalg.norm(estimate - x, ord=2)
    #print ('Dnorm of error =%g' % error_norm)
    estimate = x
    desired = .41, .08

    x, n = newtonMethod(np.array(estimate), F, 0, tol, J, np.array(desired))
    #print (n, x)
    error_norm = np.linalg.norm(estimate - x, ord=2)
    #print ('Enorm of error =%g' % error_norm)
    estimate = x
    desired = -.044, -.19

    x, n = newtonMethod(np.array(estimate), F, 0, tol, J, np.array(desired))
    #print (n, x)
    error_norm = np.linalg.norm(estimate - x, ord=2)
    #print ('Fnorm of error =%g' % error_norm)
    estimate = x
    desired = .30, -.24

    x, n = newtonMethod(np.array(estimate), F, 0, tol, J, np.array(desired))
    #print (n, x)
    error_norm = np.linalg.norm(estimate - x, ord=2)
    #print ('Gnorm of error =%g' % error_norm)
    estimate = x

    desired = .89, -.32
    x, n = newtonMethod(np.array(estimate), F, 0, tol, J, np.array(desired))
    #print (n, x)
    error_norm = np.linalg.norm(estimate - x, ord=2)
    #print ('Hnorm of error =%g' % error_norm)
    iter = 20000
    scans = 0
    successes = 0
    fails = 0
    #print("\n")
    #hist = [0]*61
    for __ in range(7):
        start = time.time()
        for _ in range(iter):
            desired =-.65+ 1.3 * random.random(), -.65+1.3 * random.random()
            #print("location to find", desired)
            x, n = newtonMethod(np.array(estimate), F, 0, tol, J, np.array(desired))
            #print(n, x)
            #if isinstance(n, int):
                #scans += n
                #successes += 1
                #hist[n] +=1
                #if n == 60:
                    #scans -= 60
                    #fails += 1
            #error_norm = np.linalg.norm(estimate - x, ord=2)
            #print("norm of error =", error_norm, _)
        end = time.time()-start
        #print("took", end, "seconds to do", iter,"finds\n")
    #print("took", scans/successes, "scans on average in", successes, "trials. With", fails, "failures.")
    #print(hist, "\n\n")


if __name__ == "__main__":
    main()