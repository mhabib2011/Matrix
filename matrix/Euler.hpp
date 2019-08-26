/**
 * @file Euler.hpp
 *
 * All rotations and axis systems follow the right-hand rule
 *
 * An instance of this class defines a rotation from coordinate frame 1 to coordinate frame 2.
 * It follows the convention of a 3-2-1 intrinsic Tait-Bryan rotation sequence.
 * In order to go from frame 1 to frame 2 we apply the following rotations consecutively.
 * 1) We rotate about our initial Z axis by an angle of _psi.
 * 2) We rotate about the newly created Y' axis by an angle of _theta.
 * 3) We rotate about the newly created X'' axis by an angle of _phi.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Quaternion;

/**
 * Euler angles class
 *
 * This class describes the rotation from frame 1
 * to frame 2 via 3-2-1 intrinsic Tait-Bryan rotation sequence.
 */
template<typename Type>
class Euler : public Vector<Type, 3>
{
public:
    /**
     * Standard constructor
     */
    Euler() = default;

    /**
     * Copy constructor
     *
     * @param other vector to copy
     */
    Euler(const Vector<Type, 3> &other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from Matrix31
     *
     * @param other Matrix31 to copy
     */
    Euler(const Matrix<Type, 3, 1> &other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from euler angles
     *
     * Instance is initialized from an 3-2-1 intrinsic Tait-Bryan
     * rotation sequence representing transformation from frame 1
     * to frame 2.
     *
     * @param phi_ rotation angle about X axis
     * @param theta_ rotation angle about Y axis
     * @param psi_ rotation angle about Z axis
     */
    Euler(Type phi_, Type theta_, Type psi_) : Vector<Type, 3>()
    {
        phi() = phi_;
        theta() = theta_;
        psi() = psi_;
    }

    /**
     * Constructor from DCM matrix
     *
     * Instance is set from Dcm representing transformation from
     * frame 2 to frame 1.
     * This instance will hold the angles defining the 3-2-1 intrinsic
     * Tait-Bryan rotation sequence from frame 1 to frame 2.
     * 
     * we should not be based the autopilot on euler, because of gimbal lock
     * but we can still use euler for logging and cases that are well studied
     *
     *
     * @param dcm Direction cosine matrix
    */
    Euler(const Dcm<Type> &dcm)
    {
        Type phi_val = Type(atan2(dcm(2, 1), dcm(2, 2)));
        Type theta_val = Type(asin(-dcm(2, 0)));
        Type psi_val = Type(atan2(dcm(1, 0), dcm(0, 0)));
        Type pi = Type(M_PI);

        Type theta_val_fw = Type(-acos(dcm(2,2))); 
        // printf ("          ************** DCM TO EULER theta_val_fw %f %f %f ***************\n", double(theta_val_fw));


        if (Type(fabs(theta_val_fw - pi/2)) < Type(1.0e-3)) {
            phi_val = Type(0.0);
            psi_val = Type(atan2(dcm(1, 2), dcm(0, 2)));

        // The vehicle is a tailsitter in FW with max pitch is 15deg, 
        // we are using here 23deg to decide weather we are in FW mode 
        } else if (Type(fabs(theta_val_fw + pi/2)) < Type(0.4)) 
        {
            // The tailsitter is FW, so the transformation is based on 3,2,3 rotation. 
            // First we are MC and we yaw around z(3), then we pitch down around y(2). 
            // Now we are fw, then we roll around z(3).
            // 3,2,3 dcm
            // cPsi*cTheta*cPhi-sPsi*sPhi -cPsi*cTheta*sPhi-sPsi*cPhi cPsi*sTheta
            // sPsi*cTheta*cPhi+cPsi*sPhi -sPsi*cTheta*cPhi+cPsi*cPhi sPsi*sTheta
            // -sTheta*cPhi sTheta*sPhi cTheta
            // then we can reconstruct the Euler angels from the dcm
            theta_val = Type(-acos(dcm(2,2)));
            // theta_val will never be 0 zero given the condition of the if statement above
            phi_val = Type(asin(dcm(2, 1)/sin(theta_val)));
            psi_val = Type(asin(dcm(1, 2)/sin(theta_val)));

        //     printf ("          **************\n");
        //     printf ("          ************** DCM TO EULER TRANSFORMATION FOR TFW %f %f %f ***************\n", double(phi_val), double(theta_val), double(psi_val));
        //     printf ("          **************\n");
        }

        phi() = phi_val;
        theta() = theta_val;
        psi() = psi_val;
    }

    /**
     * Constructor from quaternion instance.
     *
     * Instance is set from a quaternion representing transformation
     * from frame 2 to frame 1.
     * This instance will hold the angles defining the 3-2-1 intrinsic
     * Tait-Bryan rotation sequence from frame 1 to frame 2.
     *
     * @param q quaternion
    */
    Euler(const Quaternion<Type> &q)
    {
        *this = Euler(Dcm<Type>(q));
    }

    inline Type phi() const
    {
        return (*this)(0);
    }
    inline Type theta() const
    {
        return (*this)(1);
    }
    inline Type psi() const
    {
        return (*this)(2);
    }

    inline Type &phi()
    {
        return (*this)(0);
    }
    inline Type &theta()
    {
        return (*this)(1);
    }
    inline Type &psi()
    {
        return (*this)(2);
    }

};

typedef Euler<float> Eulerf;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
