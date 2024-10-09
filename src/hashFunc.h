#include <stdlib.h>

struct feature
{
  int x;
  int y;
  int z;

  bool operator==(const feature &other) const
  { return (x == other.x
            && y == other.y
            && z == other.z);
  }
};

template <>
struct std::hash<feature>
{
  std::size_t operator()(const feature& f) const
  {
    using std::size_t;
    using std::hash;
    using std::string;

    // Compute individual hash values for first,
    // second and third and combine them using XOR
    // and bit shifting:

    return ((hash<int>()(f.x) +(hash<int>()(f.y) 
             + (hash<int>()(f.z) ))));
  }
};
