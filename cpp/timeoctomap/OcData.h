
namespace octomap {
    template <class Data>
    using OcMData = Data;

    using  OCData = OcMData<opencog::Handle>;

    template <class T>
    class OCMdata : public T {
    
    };
}
