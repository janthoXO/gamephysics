#include "vectorbase.h"

#ifndef MATRIX3_H

namespace GamePhysics
{
    // a basic simple 3x3 matrix class
    template <class Scalar>
    class mat3
    {
    public:
        inline mat3();
        inline mat3(Scalar, Scalar, Scalar,
                    Scalar, Scalar, Scalar,
                    Scalar, Scalar, Scalar);
        inline mat3(const mat3<Scalar> &v );

        inline const mat3<Scalar>& operator=(const mat3<Scalar>& v);

        // unary operator
        inline mat3<Scalar> operator-() const;
        
        inline mat3<Scalar> operator* (Scalar) const;
        
        inline mat3<Scalar> operator+(const mat3<Scalar>&) const;
        inline mat3<Scalar> operator*(const mat3<Scalar>&) const;

        inline vector3Dim<Scalar> operator* (const vector3Dim<Scalar>&) const;

        inline void transpose();

        //! public to avoid [][] operators
        Scalar value[3][3]; //< Storage of maxtrix values 
    };

    /*************************************************************************
  Outputs the object in human readable form using the format
  */
    template <class Scalar>
    std::ostream&
    operator<<(std::ostream& os, const mat3<Scalar>& m)
    {
        for (int i = 0; i < 3; i++)
        {
            os << '<' << m.value[0][i] << "," << m.value[1][i] << "," << m.value[2][i] << '>' << std::endl;
        }
        return os;
    }

    template <class Scalar>
    inline mat3<Scalar>::mat3()
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                value[i][j] = 0;
            }
        }
    }
    
    template <class Scalar>
    inline mat3<Scalar>::mat3(Scalar s00, Scalar s01, Scalar s02,
                              Scalar s10, Scalar s11, Scalar s12,
                              Scalar s20, Scalar s21, Scalar s22)
    {
        value[0][0] = s00;
        value[0][1] = s01;
        value[0][2] = s02;

        value[1][0] = s10;
        value[1][1] = s11;
        value[1][2] = s12;

        value[2][0] = s20;
        value[2][1] = s21;
        value[2][2] = s22;
    }

    template <class Scalar>
    inline mat3<Scalar>::mat3(const mat3<Scalar>& v)
    {
        value[0][0] = v.value[0][0]; value[0][1] = v.value[0][1]; value[0][2] = v.value[0][2];
        value[1][0] = v.value[1][0]; value[1][1] = v.value[1][1]; value[1][2] = v.value[1][2];
        value[2][0] = v.value[2][0]; value[2][1] = v.value[2][1]; value[2][2] = v.value[2][2];
    }

    template<class Scalar>
inline const mat3<Scalar>&
mat3<Scalar>::operator=( const mat3<Scalar> &v )
    {
        value[0][0] = v.value[0][0]; value[0][1] = v.value[0][1]; value[0][2] = v.value[0][2];
        value[1][0] = v.value[1][0]; value[1][1] = v.value[1][1]; value[1][2] = v.value[1][2];
        value[2][0] = v.value[2][0]; value[2][1] = v.value[2][1]; value[2][2] = v.value[2][2]; 
        return *this;
    }

    template<class Scalar>
inline mat3<Scalar>
mat3<Scalar>::operator-() const
    {
        mat3<Scalar> nv;
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                nv.value[i][j] = -value[i][j];
            }
        }
        return nv;
    }

    template<class Scalar>
    inline mat3<Scalar>
    mat3<Scalar>::operator*(Scalar s) const
    {
        mat3<Scalar> nv;
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                nv.value[i][j] = ((Real)value[i][j]) * s;
            }
        }
        return nv;
    }
    
    template<class Scalar>
    inline mat3<Scalar>
    mat3<Scalar>::operator+( const mat3<Scalar> &v ) const
    {
        mat3<Scalar> nv;
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                nv.value[i][j] = value[i][j] + v.value[i][j];
            }
        }
        return nv;
    }

    template<class Scalar>
    inline mat3<Scalar>
    mat3<Scalar>::operator*( const mat3<Scalar>& v) const
    {
        mat3<Scalar> nv;
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {

                for(int k=0;k<3;k++)
                    nv.value[i][j] += (value[i][k] * v.value[k][j]);
            }
        }
        return nv;
    }

    template<class Scalar>
    inline vector3Dim<Scalar>
    mat3<Scalar>::operator*( const vector3Dim<Scalar>& v) const
    {
        vector3Dim<Scalar> nvec(0.0);
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                nvec[i] += (v[j] * value[i][j]);
            }
        }
        return nvec;
    }
    
    template<class Scalar>
    inline void 
    mat3<Scalar>::transpose()
    {
        for (int i=0;i<3;i++)
            for (int j=i+1;j<3;j++)
            {
                Scalar a=value[i][j];
                value[i][j]=value[j][i];
                value[j][i]=a;
            }
    }
    
}

#define MATRIX3_H
#endif
