#ifndef UCLOTH_STRUCTURES_H_
#define UCLOTH_STRUCTURES_H_

struct UCLOTH_VECTOR3F
{
    /// constructors
    UCLOTH_VECTOR3F()
    {
    }

    UCLOTH_VECTOR3F(const float x, const float y, const float z)
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
    void set(const UCLOTH_VECTOR3F& v)
    {
        x_ = v.x_;
        y_ = v.y_;
        z_ = v.z_;
    }

    /// setCast
    template <class T_VECTOR_FLOAT_3>
    void setCast(const T_VECTOR_FLOAT_3& vectorFloat3)
    {
        *this = *((UCLOTH_VECTOR3F*)&vectorFloat3);
    }

    /// data members
    float x_;
    float y_;
    float z_;
};

#endif  //! UCLOTH_STRUCTURES_H_