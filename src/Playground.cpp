struct Base
{
    virtual void reimplementMe(int a) {}
};
struct Derived : public Base
{
    void reimplementMe(int a) override {}
};
