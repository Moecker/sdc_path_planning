struct Base
{
    virtual void ReimplementMe(int a) {}
};
struct Derived : public Base
{
    void ReimplementMe(int a) override {}
};
