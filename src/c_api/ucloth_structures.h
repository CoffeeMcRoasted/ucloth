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

struct Ucloth_face
{
    /// constructors
    Ucloth_face()
    {
    }

    Ucloth_face(const unsigned int p1, const unsigned int p2, const unsigned int p3)
    {
        p1_ = p1;
        p2_ = p2;
        p3_ = p3;
    }

    /// set
    void set(const unsigned int p1, const unsigned int p2, const unsigned int p3)
    {
        p1_ = p1;
        p2_ = p2;
        p3_ = p3;
    }

    /// set
    void set(const Ucloth_face& v)
    {
        p1_ = v.p1_;
        p2_ = v.p2_;
        p3_ = v.p3_;
    }

    // /// setCast
    // template <class T_VECTOR_FLOAT_3>
    // void setCast(const T_VECTOR_FLOAT_3& vectorFloat3)
    // {
    //     *this = *((Ucloth_face*)&vectorFloat3);
    // }

    /// data members
    unsigned int p1_;
    unsigned int p2_;
    unsigned int p3_;
};

#endif  //! UCLOTH_STRUCTURES_H_