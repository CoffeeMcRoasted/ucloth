#ifndef UCLOTH_STRUCTURES_H_
#define UCLOTH_STRUCTURES_H_

struct Ucloth_vector3f
{
    /// constructors
    Ucloth_vector3f()
    {
    }

    Ucloth_vector3f(const float x, const float y, const float z)
    {
        x_ = x;
        y_ = y;
        z_ = z;
    }

    /// set
    void set(const float x, const float y, const float z)
    {
        x_ = x;
        y_ = y;
        z_ = z;
    }

    /// set
    void set(const Ucloth_vector3f& v)
    {
        x_ = v.x_;
        y_ = v.y_;
        z_ = v.z_;
    }

    /// setCast
    template <class T_VECTOR_FLOAT_3>
    void setCast(const T_VECTOR_FLOAT_3& vectorFloat3)
    {
        *this = *((Ucloth_vector3f*)&vectorFloat3);
    }

    /// data members
    float x_;
    float y_;
    float z_;
};

#endif  //! UCLOTH_STRUCTURES_H_